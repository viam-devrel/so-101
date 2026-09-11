// Command plan_stream plans a move with the machine's motion service, time-parameterises
// the plan with trajex (an ML model service on the machine), streams it through
// MoveThroughJointPositionsStreamed, returns to the start, executes the SAME plan through
// motion's paced `execute`, and scores both runs against the planned joint-space path.
//
//	go run ./tools/plan_stream -address <machine> -arm follower-arm -goal 200,0,150
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/mlmodel"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
	"google.golang.org/protobuf/encoding/protojson"

	"so_arm/internal/planning"
)

type goal struct {
	pos    r3.Vector
	orient *spatialmath.OrientationVector // nil keeps the arm's current orientation
}

type goalList []goal

func (g *goalList) String() string { return fmt.Sprintf("%d goals", len(*g)) }

func (g *goalList) Set(s string) error {
	parsed, err := parseGoal(s)
	if err != nil {
		return err
	}
	*g = append(*g, parsed)
	return nil
}

// parseGoal reads "x,y,z" or "x,y,z,ox,oy,oz" (mm; orientation vector with theta 0).
func parseGoal(s string) (goal, error) {
	fields := strings.Split(s, ",")
	if len(fields) != 3 && len(fields) != 6 {
		return goal{}, fmt.Errorf("goal %q: want x,y,z or x,y,z,ox,oy,oz", s)
	}
	v := make([]float64, len(fields))
	for i, f := range fields {
		x, err := strconv.ParseFloat(strings.TrimSpace(f), 64)
		if err != nil {
			return goal{}, fmt.Errorf("goal %q: %w", s, err)
		}
		v[i] = x
	}
	g := goal{pos: r3.Vector{X: v[0], Y: v[1], Z: v[2]}}
	if len(v) == 6 {
		g.orient = &spatialmath.OrientationVector{OX: v[3], OY: v[4], OZ: v[5]}
		if err := g.orient.IsValid(); err != nil {
			return goal{}, fmt.Errorf("goal %q: %w", s, err)
		}
	}
	return g, nil
}

type deps struct {
	arm                 arm.Arm
	motion              motion.Service
	trajex              mlmodel.Service
	armName, motionName string
	cloud               planning.GoalCloudConfig
	vel, acc, pathTol   float64 // degrees
	hz, sampleHz        float64
	batch               int
}

func main() {
	address := flag.String("address", "", "machine address")
	apiKey := flag.String("api-key", os.Getenv("VIAM_API_KEY"), "API key ($VIAM_API_KEY)")
	apiKeyID := flag.String("api-key-id", os.Getenv("VIAM_API_KEY_ID"), "API key ID ($VIAM_API_KEY_ID)")
	armName := flag.String("arm", "arm", "arm component name")
	motionName := flag.String("motion", "builtin", "motion service name")
	trajexName := flag.String("trajex", "trajex", "trajex ML model service name")
	var goals goalList
	flag.Var(&goals, "goal", "x,y,z[,ox,oy,oz] in mm in the <arm>_origin frame; repeatable")
	velDeg := flag.Float64("vel-deg", 0, "per-joint velocity limit, deg/s (default: the arm's get_motion_params speed)")
	accDeg := flag.Float64("acc-deg", 0, "per-joint acceleration limit, deg/s^2 (default: the arm's get_motion_params acceleration)")
	hz := flag.Float64("hz", 100, "trajex sampling frequency")
	pathTolDeg := flag.Float64("path-tol-deg", 0.5, "trajex path tolerance, deg")
	sampleHz := flag.Float64("sample-hz", 50, "JointPositions sampling rate during each run")
	orientTol := flag.Float64("orient-tol-deg", 0, "goal cone half-angle, deg (0 = module default)")
	posTol := flag.Float64("pos-tol-mm", 0, "goal position tolerance, mm (0 = module default)")
	batch := flag.Int("batch", 10, "points per streamed batch")
	flag.Parse()
	if *address == "" || len(goals) == 0 || *hz <= 0 || *sampleHz <= 0 || *batch < 1 {
		flag.Usage()
		os.Exit(2)
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()
	logger := logging.NewLogger("plan_stream")
	machine, err := client.New(ctx, *address, logger, client.WithDialOptions(
		rpc.WithEntityCredentials(*apiKeyID, rpc.Credentials{Type: rpc.CredentialsTypeAPIKey, Payload: *apiKey})))
	if err != nil {
		log.Fatal(err)
	}
	defer machine.Close(context.Background()) //nolint:errcheck
	d := &deps{
		armName: *armName, motionName: *motionName,
		cloud: planning.ResolveGoalCloudConfig(*orientTol, *posTol, logger),
		vel:   *velDeg, acc: *accDeg, pathTol: *pathTolDeg, hz: *hz, sampleHz: *sampleHz, batch: *batch,
	}
	if d.arm, err = arm.FromProvider(machine, *armName); err != nil {
		log.Fatalf("arm %q: %v", *armName, err)
	}
	if d.motion, err = motion.FromProvider(machine, *motionName); err != nil {
		log.Fatalf("motion %q: %v", *motionName, err)
	}
	if d.trajex, err = mlmodel.FromProvider(machine, *trajexName); err != nil {
		log.Fatalf("trajex %q: %v", *trajexName, err)
	}
	if d.vel <= 0 || d.acc <= 0 {
		resp, err := d.arm.DoCommand(ctx, map[string]any{"get_motion_params": true})
		if err != nil {
			log.Fatalf("get_motion_params: %v", err)
		}
		speed, _ := resp["current_speed_degs_per_sec"].(float64)
		accel, _ := resp["current_acceleration_degs_per_sec_per_sec"].(float64)
		if d.vel <= 0 {
			d.vel = speed
		}
		if d.acc <= 0 {
			d.acc = accel
		}
		if d.vel <= 0 || d.acc <= 0 {
			log.Fatalf("get_motion_params returned %v; pass -vel-deg and -acc-deg", resp)
		}
	}
	log.Printf("limits: %.1f deg/s, %.1f deg/s^2; trajex %g Hz, path tol %g deg", d.vel, d.acc, d.hz, d.pathTol)

	for i, g := range goals {
		if err := runGoal(ctx, d, i, g); err != nil {
			log.Fatalf("goal %d: %v", i+1, err)
		}
	}
}

// runGoal is the spec's per-goal flow: plan, trajex, streamed run, return, paced run.
func runGoal(ctx context.Context, d *deps, i int, g goal) error {
	start, err := d.arm.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("joint positions: %w", err)
	}
	pose, err := d.arm.EndPosition(ctx, nil)
	if err != nil {
		return fmt.Errorf("end position: %w", err)
	}
	orient := pose.Orientation()
	if g.orient != nil {
		orient = g.orient
	}
	// Same cone the module's own MoveToPosition sends; extra=nil is the plain cone path.
	dest, extra, _, err := planning.BuildMoveDestination(
		fmt.Sprintf("%v_origin", d.armName), spatialmath.NewPose(g.pos, orient), d.cloud, nil)
	if err != nil {
		return err
	}

	// motion's `plan` DoCommand takes a protojson MoveRequest string and returns the
	// Trajectory as generic JSON. Keys are literals: their constants live in
	// services/motion/builtin, which a tool must not import.
	req, err := motion.MoveReq{ComponentName: d.armName, Destination: dest, Extra: extra}.ToProto(d.motionName)
	if err != nil {
		return fmt.Errorf("plan request: %w", err)
	}
	reqJSON, err := protojson.Marshal(req)
	if err != nil {
		return fmt.Errorf("plan request: %w", err)
	}
	resp, err := d.motion.DoCommand(ctx, map[string]any{"plan": string(reqJSON)})
	if err != nil {
		return fmt.Errorf("plan: %w", err)
	}
	rawPlan, ok := resp["plan"]
	if !ok {
		return fmt.Errorf("plan: response has no \"plan\" key: %v", resp)
	}
	waypoints, err := armWaypoints(rawPlan, d.armName)
	if err != nil {
		return err
	}

	out, err := d.trajex.Infer(ctx, trajexInputs(waypoints, d.vel, d.acc, d.pathTol, d.hz))
	if err != nil {
		return fmt.Errorf("trajex: %w", err)
	}
	points, err := trajexPoints(out)
	if err != nil {
		return err
	}
	if len(points) == 0 {
		return fmt.Errorf("trajex returned no samples")
	}
	fmt.Printf("goal %d (%g,%g,%g): %d waypoints -> %d samples @%g Hz, trajex %.2fs, path tol %g deg\n",
		i+1, g.pos.X, g.pos.Y, g.pos.Z, len(waypoints), len(points), d.hz,
		points[len(points)-1].Time.Seconds(), d.pathTol)

	// The arm is already at start (the planner seeds from current inputs), so the module's
	// 5 deg stream-start gate is a no-op.
	trace, wall, err := d.sampled(ctx, func(ctx context.Context) error { return d.stream(ctx, points) })
	if err != nil {
		return fmt.Errorf("streamed: %w", err)
	}
	report("streamed:", wall, pathDeviation(trace, waypoints))

	if err := d.arm.MoveToJointPositions(ctx, start, nil); err != nil {
		return fmt.Errorf("return to start: %w", err)
	}

	// executeCheckStart is an L-inf epsilon in RADIANS; <= 0 selects rdk's 0.01 rad, which
	// servo droop trips every run. 0.1 rad = 5.7 deg sits above the droop and matches the
	// module's stream-start gate.
	trace, wall, err = d.sampled(ctx, func(ctx context.Context) error {
		_, err := d.motion.DoCommand(ctx, map[string]any{"execute": rawPlan, "executeCheckStart": 0.1})
		return err
	})
	if err != nil {
		return fmt.Errorf("paced execute: %w", err)
	}
	report("paced:   ", wall, pathDeviation(trace, waypoints))
	return nil
}

// stream feeds points in batches and drains acks concurrently (sending every batch before
// reading any can wedge on gRPC flow control). Same shape as tools/stream_trajectory.
func (d *deps) stream(ctx context.Context, points []arm.TrajectoryPoint) error {
	batches := make(chan []arm.TrajectoryPoint)
	responses := make(chan arm.Response)
	go func() {
		defer close(batches)
		for i := 0; i < len(points); i += d.batch {
			select {
			case batches <- points[i:min(i+d.batch, len(points))]:
			case <-ctx.Done():
				return
			}
		}
	}()
	drained := make(chan struct{})
	go func() {
		defer close(drained)
		for range responses {
		}
	}()
	// The client does not close responses; the caller closes it once the call has returned.
	err := d.arm.MoveThroughJointPositionsStreamed(ctx, batches, responses, nil)
	close(responses)
	<-drained
	return err
}

// sampled runs fn while polling JointPositions at sampleHz. A failed read is logged and
// skipped; the trace and wall time are returned alongside fn's error.
func (d *deps) sampled(ctx context.Context, fn func(context.Context) error) ([]sample, time.Duration, error) {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	var trace []sample
	done := make(chan struct{})
	started := time.Now()
	go func() {
		defer close(done)
		tick := time.NewTicker(time.Duration(float64(time.Second) / d.sampleHz))
		defer tick.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-tick.C:
			}
			q, err := d.arm.JointPositions(ctx, nil)
			if err != nil {
				if ctx.Err() == nil {
					log.Printf("sample: %v", err)
				}
				continue
			}
			trace = append(trace, sample{t: time.Since(started), q: q})
		}
	}()
	err := fn(ctx)
	wall := time.Since(started)
	cancel()
	<-done
	return trace, wall, err
}

func report(label string, wall time.Duration, dev deviation) {
	fmt.Printf("  %s wall %.2fs  dev mean %.2f p95 %.2f max %.1f deg  final %s deg\n",
		label, wall.Seconds(), dev.mean, dev.p95, dev.max, fmtDeg(dev.finalErr))
}

func fmtDeg(v []float64) string {
	parts := make([]string, len(v))
	for i, x := range v {
		parts[i] = strconv.FormatFloat(x, 'f', 1, 64)
	}
	return "[" + strings.Join(parts, " ") + "]"
}
