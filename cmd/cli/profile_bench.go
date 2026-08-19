// Command profile_bench characterizes STS3215 servo motion-profile behavior and the
// USB-serial bus, so coordinated arrival, acceleration enforcement, and trajectory
// execution can be scoped from measurements rather than estimates.
//
// See docs/superpowers/specs/2026-08-19-servo-motion-profile-bench-design.md
//
// Run one test at a time:
//
//	go run cmd/cli/profile_bench.go -port=/dev/tty.usbmodemXXXX -test=writerate
//	go run cmd/cli/profile_bench.go -port=/dev/tty.usbmodemXXXX -test=goaltime -move
//	go run cmd/cli/profile_bench.go -selftest
package main

import (
	"flag"
	"fmt"
	"os"
	"strings"
	"time"
)

// noGap is the "no command gap" sentinel. feetech.NewBus coerces a zero MinCommandGap
// back to 1ms silently (bus.go:63), so passing 0 would mislabel the fastest sweep row as
// a duplicate of the 1ms row and cap sampling at ~1kHz. Never pass 0.
const noGap = time.Nanosecond

// stepsPerDegree is the STS3215 encoder resolution: 4096 steps per full revolution.
const stepsPerDegree = 4096.0 / 360.0

type config struct {
	port         string
	test         string
	servo        int
	travelDeg    float64
	move         bool
	fullArm      bool
	assumeLimits string
	outDir       string
	selftest     bool
}

func main() {
	var cfg config
	flag.StringVar(&cfg.port, "port", "", "serial port (e.g. /dev/tty.usbmodem1101)")
	flag.StringVar(&cfg.test, "test", "writerate", "writerate|goaltime|accel|all")
	flag.IntVar(&cfg.servo, "servo", 1, "servo ID for single-joint tests")
	flag.Float64Var(&cfg.travelDeg, "travel", 20, "degrees of travel per trial (accel only; goaltime uses its own 10/20/40 sweep)")
	flag.BoolVar(&cfg.move, "move", false, "REQUIRED to allow any test that moves the arm")
	flag.BoolVar(&cfg.fullArm, "full-arm", false, "coordinated 5-joint goaltime run (needs -move)")
	flag.StringVar(&cfg.assumeLimits, "assume-limits", "", "<min>:<max> raw-tick bounds, for arms whose angle-limit registers read invalid")
	flag.StringVar(&cfg.outDir, "out", ".", "directory for CSV output")
	flag.BoolVar(&cfg.selftest, "selftest", false, "run built-in checks of the pure functions and exit (no hardware)")
	flag.Parse()

	if err := run(cfg); err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
}

func run(cfg config) error {
	if cfg.selftest {
		return runSelftests()
	}
	if cfg.port == "" {
		return fmt.Errorf("-port is required (or use -selftest)")
	}

	tests, err := selectTests(cfg.test, cfg.move)
	if err != nil {
		return err
	}
	if len(tests) == 0 {
		return fmt.Errorf("no tests to run: %q requires -move", cfg.test)
	}
	fmt.Printf("running: %s\n", strings.Join(tests, ", "))
	return fmt.Errorf("not yet implemented")
}

// selectTests resolves the -test flag into the list that will actually run. Motion tests
// are dropped without -move; "all" without -move degrades to writerate only and says so,
// rather than erroring, so "all" stays useful as a first run.
func selectTests(name string, move bool) ([]string, error) {
	motion := map[string]bool{"goaltime": true, "accel": true}
	var requested []string
	switch name {
	case "all":
		requested = []string{"writerate", "goaltime", "accel"}
	case "writerate", "goaltime", "accel":
		requested = []string{name}
	default:
		return nil, fmt.Errorf("unknown -test %q (want writerate|goaltime|accel|all)", name)
	}

	var out, skipped []string
	for _, t := range requested {
		if motion[t] && !move {
			skipped = append(skipped, t)
			continue
		}
		out = append(out, t)
	}
	if len(skipped) > 0 {
		fmt.Printf("skipping %s (motion tests require -move)\n", strings.Join(skipped, ", "))
	}
	return out, nil
}

// selftestCase is one hardware-free check of a pure function.
type selftestCase struct {
	name string
	fn   func() error
}

// selftests is appended to by each section of this file as it is built.
var selftests []selftestCase

func runSelftests() error {
	failed := 0
	for _, c := range selftests {
		if err := c.fn(); err != nil {
			fmt.Printf("FAIL %s: %v\n", c.name, err)
			failed++
			continue
		}
		fmt.Printf("PASS %s\n", c.name)
	}
	fmt.Printf("\n%d/%d passed\n", len(selftests)-failed, len(selftests))
	if failed > 0 {
		return fmt.Errorf("%d selftest(s) failed", failed)
	}
	return nil
}
