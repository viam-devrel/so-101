// Command so101-calibrate walks the devrel:so101:calibration workflow from a terminal, for a
// machine with no app.viam.com setup app (a local, cloud-less viam-server, say).
//
//	go run ./cmd/so101-calibrate -addr localhost:8080 -insecure -sensor left-calibration
//
// It drives the sensor's DoCommands (start → set_homing → start_range_recording →
// stop_range_recording → save_calibration), prompting between steps and printing the live
// readings while the range is being recorded. Ctrl-C at any point sends `abort` (or `reset`
// once the recording is complete), which leaves torque off: rest the arm first.
package main

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"sort"
	"strings"
	"syscall"
	"time"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}

func run() error {
	var (
		addr       = flag.String("addr", "localhost:8080", "machine address")
		insecure   = flag.Bool("insecure", false, "dial without TLS or credentials (local viam-server with no cloud config)")
		keyID      = flag.String("key-id", os.Getenv("VIAM_API_KEY_ID"), "API key id (or VIAM_API_KEY_ID)")
		key        = flag.String("key", os.Getenv("VIAM_API_KEY"), "API key (or VIAM_API_KEY)")
		sensorName = flag.String("sensor", "", "name of the devrel:so101:calibration sensor (required)")
	)
	flag.Parse()
	if *sensorName == "" {
		flag.Usage()
		return fmt.Errorf("-sensor is required")
	}

	logger := logging.NewLogger("so101-calibrate")
	logger.SetLevel(logging.WARN)
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	opts := []client.RobotClientOption{client.WithRefreshEvery(0)}
	switch {
	case *insecure:
		opts = append(opts, client.WithDialOptions(rpc.WithInsecure()))
	case *keyID != "":
		opts = append(opts, client.WithDialOptions(rpc.WithEntityCredentials(*keyID, rpc.Credentials{
			Type: rpc.CredentialsTypeAPIKey, Payload: *key,
		})))
	}
	robot, err := client.New(ctx, *addr, logger, opts...)
	if err != nil {
		return err
	}
	defer robot.Close(context.Background())
	s, err := sensor.FromRobot(robot, *sensorName)
	if err != nil {
		return err
	}
	w := &workflow{s: s, in: bufio.NewReader(os.Stdin)}

	// Anything that leaves the workflow early should not leave the arm half-configured.
	defer func() {
		if w.state == "" || w.state == "idle" {
			return
		}
		bg, c := context.WithTimeout(context.Background(), 5*time.Second)
		defer c()
		cmd := "abort"
		if w.state == "completed" {
			cmd = "reset"
		}
		fmt.Printf("\nleaving at state %q: sending %s (torque stays OFF; rest the arm)\n", w.state, cmd)
		_, _ = s.DoCommand(bg, map[string]interface{}{"command": cmd})
	}()

	if _, err := w.do(ctx, "reset"); err != nil {
		return err
	}
	if err := w.step(ctx, "start",
		"Torque will be switched OFF on every servo: the arm goes limp. Hold it or rest it on the table."); err != nil {
		return err
	}
	if err := w.step(ctx, "set_homing",
		"Move the arm to its ZERO pose: arm extended straight forward, forearm and gripper horizontal,\n"+
			"pan centred, gripper half open. This pose becomes 0 degrees on every joint, so take care.\n"+
			"Hold it there while you press enter."); err != nil {
		return err
	}
	if err := w.step(ctx, "start_range_recording",
		"Next: recording. After you press enter, move EVERY joint slowly through its FULL range, to\n"+
			"both stops, and open and close the gripper fully. Live ranges print while you do."); err != nil {
		return err
	}
	if err := w.recordUntilEnter(ctx); err != nil {
		return err
	}
	if err := w.step(ctx, "save_calibration",
		"This writes the limits into the servos, saves the calibration file, seeds goals and turns\n"+
			"torque back ON at the arm's current pose."); err != nil {
		return err
	}
	fmt.Println("done. Restart the viam-server so the arm component reloads the calibration file.")
	return nil
}

type workflow struct {
	s     sensor.Sensor
	in    *bufio.Reader
	state string
}

func (w *workflow) do(ctx context.Context, cmd string) (map[string]interface{}, error) {
	resp, err := w.s.DoCommand(ctx, map[string]interface{}{"command": cmd})
	if err != nil {
		return nil, fmt.Errorf("%s: %w", cmd, err)
	}
	if st, ok := resp["state"].(string); ok {
		w.state = st
	}
	if ok, has := resp["success"].(bool); has && !ok {
		return resp, fmt.Errorf("%s: %v", cmd, resp["error"])
	}
	return resp, nil
}

func (w *workflow) step(ctx context.Context, cmd, prompt string) error {
	fmt.Printf("\n== %s\n%s\n[enter to continue, Ctrl-C to abort] ", cmd, prompt)
	if err := w.waitEnter(ctx); err != nil {
		return err
	}
	resp, err := w.do(ctx, cmd)
	if err != nil {
		return err
	}
	b, _ := json.MarshalIndent(resp, "", "  ")
	fmt.Println(string(b))
	return nil
}

func (w *workflow) waitEnter(ctx context.Context) error {
	done := make(chan error, 1)
	go func() {
		_, err := w.in.ReadString('\n')
		done <- err
	}()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case err := <-done:
		return err
	}
}

// recordUntilEnter prints the live per-joint ranges from Readings until enter, then stops.
func (w *workflow) recordUntilEnter(ctx context.Context) error {
	fmt.Println("\nrecording... press enter when every joint has visited both stops.")
	enter := make(chan struct{})
	go func() { _, _ = w.in.ReadString('\n'); close(enter) }()
	ticker := time.NewTicker(500 * time.Millisecond)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-enter:
			resp, err := w.do(ctx, "stop_range_recording")
			if err != nil {
				return err
			}
			b, _ := json.MarshalIndent(resp["ranges"], "", "  ")
			fmt.Println("\nrecorded ranges:", string(b))
			return nil
		case <-ticker.C:
			r, err := w.s.Readings(ctx, nil)
			if err != nil {
				fmt.Println("readings:", err)
				continue
			}
			fmt.Print("\r" + summarizeJoints(r) + "   ")
		}
	}
}

func summarizeJoints(r map[string]interface{}) string {
	joints, _ := r["joints"].(map[string]interface{})
	names := make([]string, 0, len(joints))
	for n := range joints {
		names = append(names, n)
	}
	sort.Strings(names)
	parts := make([]string, 0, len(names))
	for _, n := range names {
		j, _ := joints[n].(map[string]interface{})
		parts = append(parts, fmt.Sprintf("%s %v [%v..%v]", n, j["current_position"], j["recorded_min"], j["recorded_max"]))
	}
	return strings.Join(parts, " | ")
}
