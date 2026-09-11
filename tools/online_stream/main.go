//go:build nlopt

// Command online_stream streams a trajex-timed plan to the arm and, when new input arrives
// mid-stream (a goal switch, an obstacle, or both), replans in-process from where the arm
// WILL be at a stitch time and splices the new trajectory into the live stream without
// stopping. A baseline runs the same scenario as today's stack: paced move, Stop, replan
// from the actual pose, paced move. One line per segment per mode. -loop instead cycles
// through 2-4 goals until Ctrl-C, planning each leg while the previous one executes.
//
//	go run -tags nlopt ./tools/online_stream -address <machine> -arm follower-arm \
//	  -goal 200,0,150 -goal 200,120,150 -switch-after 0.6 -obstacle-after 0.5,300,0,205,60,60,120
package main

import (
	"cmp"
	"context"
	"errors"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"slices"
	"strconv"
	"strings"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/mlmodel"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	"so_arm/internal/planning"
	"so_arm/tools/internal/streamx"
)

type goalList []streamx.Goal

func (g *goalList) String() string { return fmt.Sprintf("%d goals", len(*g)) }

func (g *goalList) Set(s string) error {
	parsed, err := streamx.ParseGoal(s)
	if err != nil {
		return err
	}
	*g = append(*g, parsed)
	return nil
}

// obstacleList collects -obstacle boxes, all in the <arm>_origin frame.
type obstacleList []spatialmath.Geometry

func (o *obstacleList) String() string { return fmt.Sprintf("%d obstacles", len(*o)) }

func (o *obstacleList) Set(s string) error {
	box, err := streamx.ParseObstacle(s, len(*o))
	if err != nil {
		return err
	}
	*o = append(*o, box)
	return nil
}

// eventList collects -obstacle-after events. Labels are re-numbered after flag.Parse, once
// the count of initial -obstacle boxes is known, so no two geometries share a name.
type eventList struct{ events []streamx.Event }

func (e *eventList) String() string { return fmt.Sprintf("%d obstacle events", len(e.events)) }

func (e *eventList) Set(s string) error {
	ev, err := streamx.ParseObstacleAfter(s, len(e.events))
	if err != nil {
		return err
	}
	e.events = append(e.events, ev)
	return nil
}

type deps struct {
	arm                             arm.Arm
	trajex                          mlmodel.Service
	planner                         *planner
	armName                         string
	vel, acc, pathTol, hz, sampleHz float64 // degrees
	armVel, armAcc                  float64 // the arm's configured limits, restored after the baseline
}

// scenario is the input both modes replay.
type scenario struct {
	poses                         []spatialmath.Pose     // goal A, optionally goal B, in <arm>_origin
	obstacles                     []spatialmath.Geometry // present from the start
	events                        []streamx.Event
	runway, sendEvery, planMargin time.Duration
}

// segment is one stretch of a run, scored against one plan.
type segment struct {
	label      string
	start, end time.Duration
	path       [][]float64 // radians
	event      string      // the event that ended this segment; empty for the last
}

func main() {
	address := flag.String("address", "", "machine address")
	apiKey := flag.String("api-key", os.Getenv("VIAM_API_KEY"), "API key ($VIAM_API_KEY)")
	apiKeyID := flag.String("api-key-id", os.Getenv("VIAM_API_KEY_ID"), "API key ID ($VIAM_API_KEY_ID)")
	armName := flag.String("arm", "arm", "arm component name")
	trajexName := flag.String("trajex", "trajex", "trajex ML model service name")
	var goals goalList
	flag.Var(&goals, "goal", "x,y,z[,ox,oy,oz] in mm in the <arm>_origin frame; goal A first, goal B second")
	var obstacles obstacleList
	flag.Var(&obstacles, "obstacle", "x,y,z,dx,dy,dz box (centre, side lengths) in mm in the <arm>_origin frame, present from the start; repeatable")
	switchAfter := flag.Float64("switch-after", -1, "seconds into the stream at which the target becomes goal B (needs two -goal)")
	var obstacleAfter eventList
	flag.Var(&obstacleAfter, "obstacle-after", "seconds,x,y,z,dx,dy,dz: the box appears then and the current goal is replanned around it; repeatable")
	runwayMs := flag.Int("runway-ms", 300, "how far ahead of the clock points are sent")
	sendMs := flag.Int("send-ms", 20, "send tick")
	planMarginMs := flag.Int("plan-margin-ms", 300, "time budgeted for planning + trajex; the stitch lands runway+margin after the event")
	planTimeout := flag.Duration("plan-timeout", 10*time.Second, "armplanning timeout")
	velDeg := flag.Float64("vel-deg", 0, "per-joint velocity limit, deg/s (default: the arm's get_motion_params speed)")
	accDeg := flag.Float64("acc-deg", 0, "per-joint acceleration limit, deg/s^2 (default: the arm's get_motion_params acceleration)")
	hz := flag.Float64("hz", 100, "trajex sampling frequency")
	pathTolDeg := flag.Float64("path-tol-deg", 0.5, "trajex path tolerance, deg")
	sampleHz := flag.Float64("sample-hz", 50, "JointPositions sampling rate during each run")
	orientTol := flag.Float64("orient-tol-deg", 0, "goal cone half-angle, deg (0 = module default)")
	posTol := flag.Float64("pos-tol-mm", 0, "goal position tolerance, mm (0 = module default)")
	loop := flag.Bool("loop", false, "cycle through the -goal poses (2-4) until Ctrl-C; streamed only, -baseline ignored")
	baseline := flag.Bool("baseline", true, "also run the Stop/replan/move baseline (-baseline=false to skip)")
	flag.Parse()

	maxGoals := 2
	if *loop {
		maxGoals = 4
	}
	if *address == "" || len(goals) == 0 || len(goals) > maxGoals || *hz <= 0 || *sampleHz <= 0 ||
		*runwayMs <= 0 || *sendMs <= 0 || *planMarginMs <= 0 || *planTimeout <= 0 {
		flag.Usage()
		os.Exit(2)
	}
	if *loop {
		if len(goals) < 2 {
			log.Fatal("-loop needs 2 to 4 -goal poses to cycle through")
		}
		if *switchAfter >= 0 || len(obstacleAfter.events) > 0 {
			log.Fatal("-switch-after / -obstacle-after are incompatible with -loop")
		}
	}
	sc := &scenario{
		obstacles:  obstacles,
		runway:     time.Duration(*runwayMs) * time.Millisecond,
		sendEvery:  time.Duration(*sendMs) * time.Millisecond,
		planMargin: time.Duration(*planMarginMs) * time.Millisecond,
	}
	if *switchAfter >= 0 {
		if len(goals) < 2 {
			log.Fatal("-switch-after needs a second -goal")
		}
		sc.events = append(sc.events, streamx.Event{At: time.Duration(*switchAfter * float64(time.Second)), SwitchGoal: true})
	}
	for i, ev := range obstacleAfter.events {
		ev.Obstacle.SetLabel(fmt.Sprintf("obstacle%d", len(obstacles)+i+1))
		sc.events = append(sc.events, ev)
	}
	if !*loop && len(sc.events) == 0 {
		log.Fatal("at least one of -switch-after / -obstacle-after is required")
	}
	slices.SortFunc(sc.events, func(a, b streamx.Event) int { return cmp.Compare(a.At, b.At) })
	// A nonsense cloud is worse than none: the solver can never land inside it.
	if err := planning.ValidateGoalCloudTolerances(*orientTol, *posTol); err != nil {
		log.Fatal(err)
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()
	logger := logging.NewLogger("online_stream")
	machine, err := client.New(ctx, *address, logger, client.WithDialOptions(
		rpc.WithEntityCredentials(*apiKeyID, rpc.Credentials{Type: rpc.CredentialsTypeAPIKey, Payload: *apiKey})))
	if err != nil {
		log.Fatal(err)
	}
	defer machine.Close(context.Background()) //nolint:errcheck
	d := &deps{armName: *armName, vel: *velDeg, acc: *accDeg, pathTol: *pathTolDeg, hz: *hz, sampleHz: *sampleHz}
	if d.arm, err = arm.FromProvider(machine, *armName); err != nil {
		log.Fatalf("arm %q: %v", *armName, err)
	}
	if d.trajex, err = mlmodel.FromProvider(machine, *trajexName); err != nil {
		log.Fatalf("trajex %q: %v", *trajexName, err)
	}
	fsCfg, err := machine.FrameSystemConfig(ctx)
	if err != nil {
		log.Fatalf("frame system config: %v", err)
	}
	if d.planner, err = newPlanner(fsCfg.Parts, *armName, planning.ResolveGoalCloudConfig(*orientTol, *posTol, logger), *planTimeout, logger); err != nil {
		log.Fatal(err)
	}
	resp, err := d.arm.DoCommand(ctx, map[string]any{"get_motion_params": true})
	if err != nil {
		log.Fatalf("get_motion_params: %v", err)
	}
	d.armVel, _ = resp["current_speed_degs_per_sec"].(float64)
	d.armAcc, _ = resp["current_acceleration_degs_per_sec_per_sec"].(float64)
	if d.vel <= 0 {
		d.vel = d.armVel
	}
	if d.acc <= 0 {
		d.acc = d.armAcc
	}
	if d.vel <= 0 || d.acc <= 0 {
		log.Fatalf("get_motion_params returned %v; pass -vel-deg and -acc-deg", resp)
	}
	log.Printf("limits: %.1f deg/s, %.1f deg/s^2; trajex %g Hz, path tol %g deg; runway %v, send %v, plan margin %v",
		d.vel, d.acc, d.hz, d.pathTol, sc.runway, sc.sendEvery, sc.planMargin)

	q0, err := d.arm.JointPositions(ctx, nil)
	if err != nil {
		log.Fatalf("joint positions: %v", err)
	}
	pose0, err := d.arm.EndPosition(ctx, nil)
	if err != nil {
		log.Fatalf("end position: %v", err)
	}
	for _, g := range goals {
		sc.poses = append(sc.poses, g.Pose(pose0.Orientation()))
	}
	planA, latencyA, err := d.planner.plan(ctx, q0, sc.poses[0], sc.obstacles)
	if err != nil {
		log.Fatalf("plan A: %v", err)
	}
	ptsA, err := d.trajexPoints(ctx, planA)
	if err != nil {
		log.Fatalf("plan A: %v", err)
	}
	endA := ptsA[len(ptsA)-1].Time
	for _, e := range sc.events {
		if e.At >= endA {
			log.Fatalf("event at %.2fs is not before plan A's end (%.2fs)", e.At.Seconds(), endA.Seconds())
		}
	}
	fmt.Printf("plan A: %d waypoints -> %d samples @%g Hz, %.2fs; obstacles %d; plan %dms\n",
		len(planA), len(ptsA), d.hz, endA.Seconds(), len(sc.obstacles), latencyA.Milliseconds())

	if *loop {
		if err := runLoop(ctx, d, sc, planA, latencyA, ptsA); err != nil {
			log.Fatalf("loop: %v", err)
		}
		return
	}
	if err := runStreamed(ctx, d, sc, planA, ptsA); err != nil {
		log.Fatalf("streamed: %v", err)
	}
	// Both modes start from the same pose.
	if err := d.arm.MoveToJointPositions(ctx, q0, nil); err != nil {
		log.Fatalf("return to start: %v", err)
	}
	if *baseline {
		if err := runBaselineAtLimits(ctx, d, sc, planA); err != nil {
			log.Fatalf("baseline: %v", err)
		}
	}
}

// runStreamed is the online mode: the producer streams plan A and splices each replan in.
// The goal/obstacle state lives in the replan closure, which the producer calls from one
// goroutine at a time.
func runStreamed(ctx context.Context, d *deps, sc *scenario, planA [][]float64, ptsA []arm.TrajectoryPoint) error {
	target, obstacles := sc.poses[0], slices.Clone(sc.obstacles)
	replan := func(ctx context.Context, from []float64, due []streamx.Event) ([][]float64, []arm.TrajectoryPoint, time.Duration, error) {
		started := time.Now()
		target, obstacles = apply(due, sc, target, obstacles)
		wps, _, err := d.planner.plan(ctx, from, target, obstacles)
		if err != nil {
			return nil, nil, 0, err
		}
		pts, err := d.trajexPoints(ctx, wps)
		return wps, pts, time.Since(started), err
	}
	p := newProducer(d.arm, ptsA, sc.events, replan, sc.runway, sc.sendEvery, sc.planMargin)
	s, wall, err := d.sampled(ctx, func(ctx context.Context, _ *sampler) error { return p.run(ctx) })
	if err != nil {
		return err
	}
	cur, splices := p.result()
	fmt.Println("streamed:")
	printSegments(streamedSegments(planA, cur, splices), s.all())
	fmt.Printf("  wall %.2fs\n", wall.Seconds())
	return nil
}

// apply folds due events into the goal/obstacle state.
func apply(due []streamx.Event, sc *scenario, target spatialmath.Pose, obstacles []spatialmath.Geometry) (spatialmath.Pose, []spatialmath.Geometry) {
	for _, e := range due {
		if e.SwitchGoal {
			target = sc.poses[1]
		}
		if e.Obstacle != nil {
			obstacles = append(obstacles, e.Obstacle)
		}
	}
	return target, obstacles
}

// streamedSegments cuts the run at each stitch. A segment a splice ended is scored against
// the EXECUTED part of its plan (truncated at q_s), so a pre-stitch sample cannot project
// onto a segment that was never driven; the last is scored against its whole plan.
func streamedSegments(planA [][]float64, cur []arm.TrajectoryPoint, splices []splice) []segment {
	label, path := "goal A", planA
	var start time.Duration
	var segs []segment
	for _, s := range splices {
		segs = append(segs, segment{
			label: label, start: start, end: s.tStitch, path: streamx.ExecutedPrefix(path, s.qs),
			event: fmt.Sprintf("event @%s: %s; stitch @%.2fs; plan %dms (%d waypoints, %.2fs); stitch late %dms (client clock; leads the arm by the RPC + gate offset)",
				eventTimes(s.events), describeEvents(s.events), s.tStitch.Seconds(), s.planLatency.Milliseconds(),
				len(s.waypoints), s.newPts[len(s.newPts)-1].Time.Seconds(), s.stitchLate.Milliseconds()),
		})
		label, path, start = labelAfter(label, s.events), s.waypoints, s.tStitch
	}
	end := start
	if len(cur) > 0 {
		end = cur[len(cur)-1].Time
	}
	return append(segs, segment{label: label, start: start, end: end, path: path})
}

// legPlan is the plan one leg executes: plan A for leg 0, the previous splice's for the rest.
type legPlan struct {
	start, dur, latency, stitchLate time.Duration
	path                            [][]float64
}

// loopStats accumulates the printed legs for the Ctrl-C summary.
type loopStats struct {
	legs               int
	planTotal, planMax time.Duration
	late               int
	lateWorst          time.Duration
	devTotal, devWorst float64
}

func (st *loopStats) add(lp legPlan, dev streamx.Deviation) {
	st.legs++
	st.planTotal += lp.latency
	st.planMax = max(st.planMax, lp.latency)
	if lp.stitchLate > 0 {
		st.late++
	}
	st.lateWorst = max(st.lateWorst, lp.stitchLate)
	st.devTotal += dev.Mean
	st.devWorst = max(st.devWorst, dev.Mean)
}

// runLoop cycles through the goals until Ctrl-C: the producer's next hook schedules each
// leg runway+margin before the current trajectory ends, so the replan is spliced in where
// the previous leg rests. The leg counter lives in the replan closure alone - replan runs
// without the producer's mutex and next under it, so they must not share state.
func runLoop(ctx context.Context, d *deps, sc *scenario, planA [][]float64, latencyA time.Duration,
	ptsA []arm.TrajectoryPoint,
) error {
	leg := 0
	replan := func(ctx context.Context, from []float64, _ []streamx.Event) ([][]float64, []arm.TrajectoryPoint, time.Duration, error) {
		started := time.Now()
		leg++
		wps, _, err := d.planner.plan(ctx, from, sc.poses[leg%len(sc.poses)], sc.obstacles)
		if err != nil {
			return nil, nil, 0, err
		}
		pts, err := d.trajexPoints(ctx, wps)
		return wps, pts, time.Since(started), err
	}
	p := newProducer(d.arm, ptsA, nil, replan, sc.runway, sc.sendEvery, sc.planMargin)
	p.next = func(cur []arm.TrajectoryPoint) (streamx.Event, bool) {
		return streamx.Event{At: max(0, cur[len(cur)-1].Time-sc.runway-sc.planMargin)}, true
	}

	first := legPlan{dur: ptsA[len(ptsA)-1].Time, latency: latencyA, path: planA}
	var st loopStats
	fmt.Printf("loop: %d goals, obstacles %d; Ctrl-C to stop\n", len(sc.poses), len(sc.obstacles))
	_, wall, err := d.sampled(ctx, func(ctx context.Context, s *sampler) error {
		stats, stop := make(chan loopStats, 1), make(chan struct{})
		go func() { stats <- printLegs(p, s, first, len(sc.poses), stop) }()
		runErr := p.run(ctx)
		close(stop)
		st = <-stats
		return runErr
	})
	// Ctrl-C is the expected end; run wraps it as "stream: context canceled".
	if err != nil && !errors.Is(err, context.Canceled) {
		return err
	}
	if st.legs == 0 {
		fmt.Printf("loop: no leg completed in %.1fs\n", wall.Seconds())
		return nil
	}
	fmt.Printf("loop: %d legs in %.1fs; plan latency mean %dms max %dms; late stitches %d (worst %dms);\n"+
		"      dev mean over legs %.1f deg (worst leg %.1f); module late count: see the arm log's per-stream summary\n",
		st.legs, wall.Seconds(), (st.planTotal / time.Duration(st.legs)).Milliseconds(), st.planMax.Milliseconds(),
		st.late, st.lateWorst.Milliseconds(), st.devTotal/float64(st.legs), st.devWorst)
	return nil
}

// printLegs prints each finished leg exactly once, polling result() every 250ms. Leg k
// spans [tStitch_{k-1}, tStitch_k) and runs the plan splice k-1 brought in (plan A for leg
// 0), so it is printable once splice k has landed AND its stitch time is past - the cursor
// only advances, so a splice that lands late prints on the next poll, never twice. The leg
// in flight at Ctrl-C is not scored.
func printLegs(p *producer, s *sampler, first legPlan, goals int, stop <-chan struct{}) loopStats {
	var st loopStats
	printed := 0
	flush := func() {
		_, splices := p.result()
		for ; printed < len(splices); printed++ {
			if s.elapsed() < splices[printed].tStitch {
				return
			}
			lp := first
			if printed > 0 {
				prev := splices[printed-1]
				lp = legPlan{
					start: prev.tStitch, dur: prev.newPts[len(prev.newPts)-1].Time,
					latency: prev.planLatency, stitchLate: prev.stitchLate, path: prev.waypoints,
				}
			}
			dev := streamx.PathDeviation(s.window(lp.start, splices[printed].tStitch), lp.path)
			fmt.Printf("leg %d -> goal %s: plan %dms, %d wp, %.2fs; stitch late %dms; dev mean %.1f p95 %.1f max %.1f deg; final %s deg\n",
				printed, goalLabel(printed%goals), lp.latency.Milliseconds(), len(lp.path), lp.dur.Seconds(),
				lp.stitchLate.Milliseconds(), dev.Mean, dev.P95, dev.Max, fmtDeg(dev.FinalErr))
			st.add(lp, dev)
		}
	}
	tick := time.NewTicker(250 * time.Millisecond)
	defer tick.Stop()
	for {
		select {
		case <-tick.C:
			flush()
		case <-stop:
			flush()
			return st
		}
	}
}

func goalLabel(i int) string { return string(rune('A' + i)) }

// runBaselineAtLimits runs the baseline at trajex's limits (the paced path uses the arm's
// configured ones) and restores them afterwards.
func runBaselineAtLimits(ctx context.Context, d *deps, sc *scenario, planA [][]float64) error {
	if d.vel != d.armVel || d.acc != d.armAcc {
		if err := d.setArmLimits(ctx, d.vel, d.acc); err != nil {
			return err
		}
		defer func() {
			if err := d.setArmLimits(context.Background(), d.armVel, d.armAcc); err != nil {
				log.Printf("restore arm limits: %v", err)
			}
		}()
	}
	return runBaseline(ctx, d, sc, planA)
}

// runBaseline is today's stack: paced move; at t_e Stop, read the pose, replan from it,
// paced move again. Stop cancels the running op, so the cancelled move returns gRPC
// Canceled BY DESIGN: that one code is the expected outcome, anything else is fatal.
// The segment boundary is the Stop return, not a stitch time.
func runBaseline(ctx context.Context, d *deps, sc *scenario, planA [][]float64) error {
	fired := make([]bool, len(sc.events))
	target, obstacles := sc.poses[0], slices.Clone(sc.obstacles)
	label, path := "goal A", planA
	var segs []segment
	s, wall, err := d.sampled(ctx, func(ctx context.Context, _ *sampler) error {
		started := time.Now()
		var segStart time.Duration
		for {
			moveErr := make(chan error, 1)
			go func(p [][]float64) { moveErr <- d.arm.MoveThroughJointPositions(ctx, p, nil, nil) }(path)
			next, ok := nextEvent(sc.events, fired)
			if !ok {
				if err := <-moveErr; err != nil {
					return fmt.Errorf("paced move: %w", err)
				}
				segs = append(segs, segment{label: label, start: segStart, end: time.Since(started), path: path})
				return nil
			}
			timer := time.After(next - time.Since(started))
			running := true
			select {
			case err := <-moveErr:
				if err != nil {
					return fmt.Errorf("paced move: %w", err)
				}
				running = false // finished before the event; the arm rests until it fires
				select {
				case <-timer:
				case <-ctx.Done():
					return ctx.Err()
				}
			case <-timer:
			}
			stopAt := time.Now()
			if err := d.arm.Stop(ctx, nil); err != nil {
				return fmt.Errorf("stop: %w", err)
			}
			if running {
				// status.Code, not errors.Is: context.Canceled does not survive gRPC as a value.
				if err := <-moveErr; err != nil && status.Code(err) != codes.Canceled {
					return fmt.Errorf("paced move: %w", err)
				}
			}
			segEnd := time.Since(started)
			due := indexEvents(sc.events, streamx.Due(sc.events, fired, segEnd))
			target, obstacles = apply(due, sc, target, obstacles)
			q, err := d.arm.JointPositions(ctx, nil)
			if err != nil {
				return fmt.Errorf("joint positions: %w", err)
			}
			var newPath [][]float64
			var latency time.Duration
			for {
				var lat time.Duration
				if newPath, lat, err = d.planner.plan(ctx, q, target, obstacles); err != nil {
					return err
				}
				latency += lat
				// Fold in what came due while planning, as the producer folds everything due
				// on one tick into one replan; the arm is stopped, so q still holds.
				more := indexEvents(sc.events, streamx.Due(sc.events, fired, time.Since(started)))
				if len(more) == 0 {
					break
				}
				due = append(due, more...)
				target, obstacles = apply(more, sc, target, obstacles)
			}
			segs = append(segs, segment{
				label: label, start: segStart, end: segEnd, path: streamx.ExecutedPrefix(path, q),
				event: fmt.Sprintf("event @%s: %s; Stop -> replan %dms -> move; event-to-move gap %.2fs",
					eventTimes(due), describeEvents(due), latency.Milliseconds(), time.Since(stopAt).Seconds()),
			})
			label, path, segStart = labelAfter(label, due), newPath, segEnd
		}
	})
	if err != nil {
		return err
	}
	fmt.Println("baseline:")
	printSegments(segs, s.all())
	fmt.Printf("  wall %.2fs\n", wall.Seconds())
	return nil
}

// nextEvent is the earliest unfired event's time.
func nextEvent(events []streamx.Event, fired []bool) (time.Duration, bool) {
	var best time.Duration
	found := false
	for i, e := range events {
		if !fired[i] && (!found || e.At < best) {
			best, found = e.At, true
		}
	}
	return best, found
}

func indexEvents(events []streamx.Event, idx []int) []streamx.Event {
	out := make([]streamx.Event, len(idx))
	for i, k := range idx {
		out[i] = events[k]
	}
	return out
}

func labelAfter(label string, events []streamx.Event) string {
	for _, e := range events {
		if e.SwitchGoal {
			return "goal B"
		}
	}
	return label
}

func describeEvents(events []streamx.Event) string {
	parts := make([]string, len(events))
	for i, e := range events {
		if e.SwitchGoal {
			parts[i] = "switch -> goal B"
		} else {
			parts[i] = e.Obstacle.Label()
		}
	}
	return strings.Join(parts, " + ")
}

func eventTimes(events []streamx.Event) string {
	parts := make([]string, len(events))
	for i, e := range events {
		parts[i] = fmt.Sprintf("%.2fs", e.At.Seconds())
	}
	return strings.Join(parts, "/")
}

// printSegments scores each segment's samples (start <= T < end; the last takes everything
// after its start, so the settle at the end counts) and prints it, then the event that
// ended it.
func printSegments(segs []segment, trace []streamx.Sample) {
	for i, s := range segs {
		last := i == len(segs)-1
		hi := s.end
		if last {
			hi = math.MaxInt64
		}
		var part []streamx.Sample
		for _, smp := range trace {
			if smp.T >= s.start && smp.T < hi {
				part = append(part, smp)
			}
		}
		dev := streamx.PathDeviation(part, s.path)
		line := fmt.Sprintf("  seg %d (%s, %.2f-%.2fs):  dev mean %.1f p95 %.1f max %.1f deg",
			i+1, s.label, s.start.Seconds(), s.end.Seconds(), dev.Mean, dev.P95, dev.Max)
		if last {
			line += fmt.Sprintf("  final %s deg", fmtDeg(dev.FinalErr))
		}
		fmt.Println(line)
		if s.event != "" {
			fmt.Println("  " + s.event)
		}
	}
}

// trajexPoints time-parameterises wps (radians) at the tool's limits.
func (d *deps) trajexPoints(ctx context.Context, wps [][]float64) ([]arm.TrajectoryPoint, error) {
	out, err := d.trajex.Infer(ctx, streamx.Inputs(wps, d.vel, d.acc, d.pathTol, d.hz))
	if err != nil {
		return nil, fmt.Errorf("trajex: %w", err)
	}
	pts, err := streamx.Points(out)
	if err != nil {
		return nil, err
	}
	if len(pts) == 0 {
		return nil, errors.New("trajex returned no samples")
	}
	return pts, nil
}

// setArmLimits is the arm's set_speed / set_acceleration DoCommand (deg/s, deg/s^2; the arm
// clamps to 3-180 and 50-500).
func (d *deps) setArmLimits(ctx context.Context, velDeg, accDeg float64) error {
	_, err := d.arm.DoCommand(ctx, map[string]any{"set_speed": velDeg, "set_acceleration": accDeg})
	if err != nil {
		return fmt.Errorf("set arm limits to %.1f deg/s, %.1f deg/s^2: %w", velDeg, accDeg, err)
	}
	return nil
}

// sampler collects JointPositions reads. The mutex makes the trace readable WHILE the run
// is live (the loop scores each leg as it lands); the other modes just take all() at the end.
// One sampler for every mode: a second one would double the JointPositions load on the very
// stream being measured.
type sampler struct {
	started time.Time
	mu      sync.Mutex
	trace   []streamx.Sample
}

func (s *sampler) elapsed() time.Duration { return time.Since(s.started) }

func (s *sampler) add(smp streamx.Sample) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.trace = append(s.trace, smp)
}

// window copies the samples with lo <= T < hi.
func (s *sampler) window(lo, hi time.Duration) []streamx.Sample {
	s.mu.Lock()
	defer s.mu.Unlock()
	var out []streamx.Sample
	for _, smp := range s.trace {
		if smp.T >= lo && smp.T < hi {
			out = append(out, smp)
		}
	}
	return out
}

func (s *sampler) all() []streamx.Sample { return s.window(0, math.MaxInt64) }

// sampled runs fn while polling JointPositions at sampleHz. A failed read is logged and
// skipped; the sampler and wall time are returned alongside fn's error.
func (d *deps) sampled(ctx context.Context, fn func(context.Context, *sampler) error) (*sampler, time.Duration, error) {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	s := &sampler{started: time.Now()}
	done := make(chan struct{})
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
			s.add(streamx.Sample{T: s.elapsed(), Q: q})
		}
	}()
	err := fn(ctx, s)
	wall := s.elapsed()
	cancel()
	<-done
	return s, wall, err
}

func fmtDeg(v []float64) string {
	parts := make([]string, len(v))
	for i, x := range v {
		parts[i] = strconv.FormatFloat(x, 'f', 1, 64)
	}
	return "[" + strings.Join(parts, " ") + "]"
}
