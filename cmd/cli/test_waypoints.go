// Command test_waypoints exercises MoveThroughJointPositions against a real arm — Step 4b of
// the hardware verification in docs/superpowers/plans/2026-08-20-per-joint-motion-profiles.md.
//
// Why this exists: MoveToJointPositions reads a fresh position reference and was correct all
// along. MoveThroughJointPositions is the path motion.Move actually uses, and it is where the
// final quality review found the arm could end in the WRONG CONFIGURATION while reporting
// success. This drives that path deliberately and checks the outcome.
//
// The trajectory is shaped to trigger the bug if it is present. One joint's travel is
// concentrated in the FIRST HALF of the path, so its delta in the final segment is ~0. Because
// every SetGoals overwrites the previous goal and the whole stream is issued in tens of
// milliseconds, the arm's motion is governed by the last write alone — and under the bug that
// joint gets k~0, floors to 1 step/s (0.088 deg/s), and crawls toward a target it is nowhere
// near while the call returns "success".
//
//	go run cmd/cli/test_waypoints.go \
//	  -address=so101-teleop-station-main.eb5ammnhkd.viam.cloud \
//	  -api-key=<KEY> -api-key-id=<KEY_ID> -arm=follower-arm
//
// Add -dry-run first to print the trajectory and the safety check without moving anything.
package main

import (
	"context"
	"flag"
	"fmt"
	"math"
	"os"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

const (
	// settleToleranceDeg is how close a joint must end to its commanded target. The servos
	// dither +/-3 encoder steps (~0.26 deg) at rest, measured on 2026-08-19, so anything
	// tighter than that would fail on healthy hardware.
	settleToleranceDeg = 0.6

	// stillMovingToleranceDeg is how much motion after the call returns counts as "the call
	// returned early". Set above the dither band for the same reason.
	stillMovingToleranceDeg = 0.5
)

func main() {
	address := flag.String("address", "", "machine address, e.g. name.abcdef.viam.cloud (required)")
	apiKey := flag.String("api-key", "", "API key (required)")
	apiKeyID := flag.String("api-key-id", "", "API key ID (required)")
	armName := flag.String("arm", "follower-arm", "arm component name")
	earlyJoint := flag.Int("early-joint", 1, "0-indexed joint whose travel is concentrated EARLY (this is the one the bug strands)")
	spreadJoint := flag.Int("spread-joint", 4, "0-indexed joint whose travel is spread across the whole path")
	earlyDeg := flag.Float64("early-deg", 25, "degrees the early joint travels")
	spreadDeg := flag.Float64("spread-deg", 20, "degrees the spread joint travels")
	numWaypoints := flag.Int("waypoints", 12, "number of waypoints (>=10 recommended)")
	dryRun := flag.Bool("dry-run", false, "print the trajectory and safety check, then exit without moving")
	settleWait := flag.Duration("settle-wait", 5*time.Second, "how long to watch for motion after the call returns")
	flag.Parse()

	if err := run(*address, *apiKey, *apiKeyID, *armName, *earlyJoint, *spreadJoint,
		*earlyDeg, *spreadDeg, *numWaypoints, *dryRun, *settleWait); err != nil {
		fmt.Fprintf(os.Stderr, "\nerror: %v\n", err)
		os.Exit(1)
	}
}

func run(address, apiKey, apiKeyID, armName string, earlyJoint, spreadJoint int,
	earlyDeg, spreadDeg float64, numWaypoints int, dryRun bool, settleWait time.Duration,
) error {
	if address == "" || apiKey == "" || apiKeyID == "" {
		return fmt.Errorf("-address, -api-key and -api-key-id are all required")
	}

	ctx := context.Background()
	logger := logging.NewLogger("test_waypoints")

	fmt.Printf("connecting to %s ...\n", address)
	machine, err := client.New(ctx, address, logger, client.WithDialOptions(
		rpc.WithEntityCredentials(apiKeyID, rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: apiKey,
		}),
	))
	if err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer machine.Close(ctx)

	a, err := arm.FromRobot(machine, armName)
	if err != nil {
		return fmt.Errorf("find arm %q: %w", armName, err)
	}

	start, err := a.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("read starting joint positions: %w", err)
	}
	dof := len(start)
	fmt.Printf("arm %q has %d joints, currently at %s\n", armName, dof, degStr(start))

	if earlyJoint < 0 || earlyJoint >= dof || spreadJoint < 0 || spreadJoint >= dof {
		return fmt.Errorf("joint indices must be in [0,%d)", dof)
	}
	if earlyJoint == spreadJoint {
		return fmt.Errorf("-early-joint and -spread-joint must differ")
	}
	if numWaypoints < 4 {
		return fmt.Errorf("need at least 4 waypoints to shape the trajectory")
	}

	traj := buildTrajectory(start, earlyJoint, spreadJoint, earlyDeg, spreadDeg, numWaypoints)
	final := traj[len(traj)-1]

	printTrajectory(traj, earlyJoint, spreadJoint)

	// The failure mode depends on the early joint having ~zero delta in the LAST segment.
	// Verify the trajectory actually has that shape before trusting the result.
	lastDelta := math.Abs(final[earlyJoint]-traj[len(traj)-2][earlyJoint]) * 180 / math.Pi
	fmt.Printf("\nearly joint %d moves %.3f deg in the final segment", earlyJoint, lastDelta)
	if lastDelta > 0.01 {
		fmt.Printf("  <-- WARNING: not ~0, so this run does not probe the bug\n")
	} else {
		fmt.Printf("  (~0, as intended: this is what strands it under the bug)\n")
	}

	if dryRun {
		fmt.Println("\n-dry-run set; not moving. Re-run without it to execute.")
		return nil
	}

	fmt.Printf("\nCLEAR THE WORKSPACE. Executing in 3s ...\n")
	time.Sleep(3 * time.Second)

	issued := time.Now()
	moveErr := a.MoveThroughJointPositions(ctx, traj, nil, nil)
	elapsed := time.Since(issued)
	if moveErr != nil {
		return fmt.Errorf("MoveThroughJointPositions returned after %v: %w", elapsed, moveErr)
	}
	fmt.Printf("\ncall returned after %v\n", elapsed.Round(time.Millisecond))

	atReturn, err := a.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("read positions at return: %w", err)
	}

	// If the arm keeps moving after the call returned, the call returned early — which is
	// exactly what the collapsed timeout did under the bug.
	fmt.Printf("watching for %v to see whether the arm is still moving ...\n", settleWait)
	time.Sleep(settleWait)
	settled, err := a.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("read settled positions: %w", err)
	}

	return report(final, atReturn, settled, earlyJoint, spreadJoint)
}

// buildTrajectory shapes a path where the early joint finishes its travel by the midpoint and
// the spread joint moves linearly throughout. Every other joint holds station.
func buildTrajectory(start []referenceframe.Input, earlyJoint, spreadJoint int,
	earlyDeg, spreadDeg float64, n int,
) [][]referenceframe.Input {
	earlyRad := earlyDeg * math.Pi / 180
	spreadRad := spreadDeg * math.Pi / 180
	half := n / 2

	traj := make([][]referenceframe.Input, 0, n)
	for i := 1; i <= n; i++ {
		wp := make([]referenceframe.Input, len(start))
		copy(wp, start)

		// Early joint: reaches its full travel at the midpoint, then holds.
		earlyFrac := float64(i) / float64(half)
		if earlyFrac > 1 {
			earlyFrac = 1
		}
		wp[earlyJoint] = start[earlyJoint] + earlyRad*earlyFrac

		// Spread joint: linear across the whole path.
		wp[spreadJoint] = start[spreadJoint] + spreadRad*(float64(i)/float64(n))

		traj = append(traj, wp)
	}
	return traj
}

func printTrajectory(traj [][]referenceframe.Input, earlyJoint, spreadJoint int) {
	fmt.Printf("\n%d waypoints (degrees, showing joints %d and %d):\n", len(traj), earlyJoint, spreadJoint)
	fmt.Printf("  %-4s %12s %12s\n", "wp", fmt.Sprintf("j%d(early)", earlyJoint), fmt.Sprintf("j%d(spread)", spreadJoint))
	for i, wp := range traj {
		fmt.Printf("  %-4d %12.2f %12.2f\n", i,
			wp[earlyJoint]*180/math.Pi, wp[spreadJoint]*180/math.Pi)
	}
}

func report(target, atReturn, settled []referenceframe.Input, earlyJoint, spreadJoint int) error {
	fmt.Printf("\n%-6s %12s %12s %12s %12s\n", "joint", "target", "at return", "settled", "error")
	worstErr, worstDrift := 0.0, 0.0
	for i := range target {
		tgt := target[i] * 180 / math.Pi
		ret := atReturn[i] * 180 / math.Pi
		set := settled[i] * 180 / math.Pi
		errDeg := math.Abs(set - tgt)
		drift := math.Abs(set - ret)

		label := fmt.Sprintf("%d", i)
		switch i {
		case earlyJoint:
			label += " E"
		case spreadJoint:
			label += " S"
		}
		fmt.Printf("%-6s %12.2f %12.2f %12.2f %12.2f\n", label, tgt, ret, set, errDeg)

		if errDeg > worstErr {
			worstErr = errDeg
		}
		if drift > worstDrift {
			worstDrift = drift
		}
	}

	fmt.Printf("\n  worst final error : %.2f deg   (tolerance %.2f)\n", worstErr, settleToleranceDeg)
	fmt.Printf("  worst post-return drift: %.2f deg   (tolerance %.2f)\n", worstDrift, stillMovingToleranceDeg)

	var failures []string
	if worstErr > settleToleranceDeg {
		failures = append(failures, fmt.Sprintf(
			"a joint ended %.2f deg from its commanded target: the arm finished in the wrong configuration", worstErr))
	}
	if worstDrift > stillMovingToleranceDeg {
		failures = append(failures, fmt.Sprintf(
			"the arm moved %.2f deg AFTER the call returned: the call returned early", worstDrift))
	}

	if len(failures) == 0 {
		fmt.Printf("\nPASS: every joint reached its target, and the call did not return early.\n")
		return nil
	}
	fmt.Printf("\nFAIL:\n")
	for _, f := range failures {
		fmt.Printf("  - %s\n", f)
	}
	fmt.Printf("\nIf the EARLY joint (marked E) is the one stranded, that is the stale-reference\n" +
		"bug: the final waypoint's profile was computed against the previous commanded\n" +
		"waypoint instead of the arm's actual position, flooring that joint to 1 step/s.\n")
	return fmt.Errorf("hardware verification failed")
}

func degStr(in []referenceframe.Input) string {
	s := "["
	for i, v := range in {
		if i > 0 {
			s += " "
		}
		s += fmt.Sprintf("%.1f", v*180/math.Pi)
	}
	return s + "]"
}
