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
	"context"
	"encoding/csv"
	"flag"
	"fmt"
	"math"
	"os"
	"os/signal"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"syscall"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.bug.st/serial"
)

// noGap is the "no command gap" sentinel. feetech.NewBus coerces a zero MinCommandGap
// back to 1ms silently (bus.go:63), so passing 0 would mislabel the fastest sweep row as
// a duplicate of the 1ms row and cap sampling at ~1kHz. Never pass 0.
const noGap = time.Nanosecond

// stepsPerDegree is the STS3215 encoder resolution: 4096 steps per full revolution.
const stepsPerDegree = 4096.0 / 360.0

type config struct {
	port           string
	test           string
	servo          int
	travelDeg      float64
	move           bool
	fullArm        bool
	assumeLimits   string
	outDir         string
	stockTransport bool
	selftest       bool
}

func main() {
	var cfg config
	flag.StringVar(&cfg.port, "port", "", "serial port (e.g. /dev/tty.usbmodem1101)")
	flag.StringVar(&cfg.test, "test", "writerate", "writerate|goaltime|accel|all")
	flag.IntVar(&cfg.servo, "servo", 1, "servo ID for single-joint tests")
	flag.Float64Var(&cfg.travelDeg, "travel", 20, "degrees of travel per trial (accel only; goaltime uses its own 10/20/40 sweep)")
	flag.BoolVar(&cfg.move, "move", false, "REQUIRED to allow any test that moves the arm")
	flag.BoolVar(&cfg.fullArm, "full-arm", false, "coordinated 5-joint goaltime run (needs -move)")
	flag.StringVar(&cfg.assumeLimits, "assume-limits", "", "<min>:<max> raw encoder-step bounds, for arms whose angle-limit registers read invalid")
	flag.StringVar(&cfg.outDir, "out", ".", "directory for CSV output")
	flag.BoolVar(&cfg.stockTransport, "stock-transport", false, "force feetech's own transport for the motion tests (they default to the fast-flush one for ~10x the sampling rate)")
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

	if err := validateConfig(cfg); err != nil {
		return err
	}

	tests, skipped, err := selectTests(cfg.test, cfg.move)
	if err != nil {
		return err
	}
	if len(skipped) > 0 {
		fmt.Printf("skipping %s (motion tests require -move)\n", strings.Join(skipped, ", "))
	}
	if len(tests) == 0 {
		return fmt.Errorf("no tests to run: %q requires -move", cfg.test)
	}
	fmt.Printf("running: %s\n", strings.Join(tests, ", "))
	for _, t := range tests {
		var err error
		switch t {
		case "writerate":
			err = runWriteRate(cfg)
		case "goaltime":
			err = runGoalTime(cfg)
		case "accel":
			err = runAccel(cfg)
		}
		if err != nil {
			return fmt.Errorf("%s: %w", t, err)
		}
	}
	return nil
}

// --- test: goaltime ---

var goalTimeSweepMs = []int{200, 400, 800, 1600, 3000}
var goalTimeTravelDeg = []float64{10, 20, 40}

// goalTimeAcc is held fixed across the GoalTime sweep so timing is the only variable.
const goalTimeAcc = 20

func runGoalTime(cfg config) error {
	// commanded is the set this test actually drives; the rig always contains the whole
	// arm so holdPose can keep the untested joints powered. Without that, a single-joint
	// accel run measures a collapsing chain rather than a posed arm.
	commanded := []int{cfg.servo}
	if cfg.fullArm {
		commanded = armServoIDs
	}
	ids := commanded

	r, err := openRig(cfg.port, noGap, armServoIDs, !cfg.stockTransport) // fast-flush samples ~10x faster
	if err != nil {
		return err
	}
	defer r.close()

	summary, err := newCSV(cfg.outDir, csvName("goaltime", "", cfg.servo, cfg.fullArm, transportName(!cfg.stockTransport)), []string{
		"trial", "servo_id", "transport", "n_samples", "commanded_ms", "travel_deg", "measured_ms",
		"motion_start_sec", "motion_end_sec", "moved", "sampling_timed_out",
		"ratio_measured_over_commanded", "goal_time_exceeded", "peak_vel_steps_per_sec", "overshoot_steps",
	})
	if err != nil {
		return err
	}
	defer summary.close()

	samplesCSV, err := newCSV(cfg.outDir, csvName("goaltime", "_samples", cfg.servo, cfg.fullArm, transportName(!cfg.stockTransport)), sampleHeader)
	if err != nil {
		return err
	}
	defer samplesCSV.close()

	fmt.Printf("\n%-22s %10s %10s %8s %8s\n", "trial", "commanded", "measured", "ratio", "clipped")

	runErr := r.withSafeShutdown(func(ctx context.Context) error {
		if err := r.holdPose(ctx); err != nil {
			return err
		}

		for _, travelDeg := range goalTimeTravelDeg {
			for _, ms := range goalTimeSweepMs {
				if ctx.Err() != nil {
					return ctx.Err()
				}
				trial := fmt.Sprintf("t%.0fdeg_%dms", travelDeg, ms)
				if err := goalTimeTrial(ctx, r, cfg, ids, travelDeg, ms, trial, summary, samplesCSV); err != nil {
					return err
				}
			}
		}
		return nil
	})

	// Surface CSV write failures: the files are this harness's durable artifact, so a
	// silent failure would look like a successful run with no data.
	return firstErr(runErr, summary.close(), samplesCSV.close())
}

// firstErr returns the first non-nil error, so a run error is not masked by a close error
// and vice versa.
func firstErr(errs ...error) error {
	for _, e := range errs {
		if e != nil {
			return e
		}
	}
	return nil
}

// goalTimeTrial runs one out-and-back at a commanded time. In -full-arm mode each joint
// gets a DIFFERENT travel under the SAME commanded time, which is what makes per-joint
// arrival spread meaningful.
func goalTimeTrial(
	ctx context.Context,
	r *rig,
	cfg config,
	ids []int,
	travelDeg float64,
	ms int,
	trial string,
	summary, samplesCSV *csvWriter,
) error {
	steps := degToSteps(travelDeg)
	plans, err := prepareTravel(ctx, r, ids, steps, cfg.assumeLimits)
	if err != nil {
		return err
	}

	goals := make(map[int]feetech.GoalRequest, len(plans))
	targets := make(map[int]int, len(plans))
	for i, p := range plans {
		target := p.target
		if cfg.fullArm {
			// Stagger travel per joint so arrival spread is observable: joint i travels
			// (i+1)/len of the nominal distance, all under the same commanded time.
			//
			// No re-validation needed, and deliberately no second planTravel call: Go
			// truncates integer division toward zero, so scaled is bounded by steps with
			// a matching sign, putting p.start+scaled inside (p.start, p.target] — a
			// range planTravel already validated above. A second call with its own limits
			// could fail and silently fall back to the un-staggered target, collapsing the
			// spread to ~0 and reporting perfect coordinated arrival from a test that
			// never staggered anything.
			scaled := steps * (i + 1) / len(plans)
			target = p.start + scaled
		}
		targets[p.id] = target
		goals[p.id] = feetech.GoalRequest{Position: target, Time: ms, Acc: goalTimeAcc}
	}

	if err := r.group.SetGoals(ctx, goals); err != nil {
		return fmt.Errorf("%s: SetGoals: %w", trial, err)
	}
	commanded := time.Duration(ms) * time.Millisecond
	series, timedOut, err := sampleUntilStopped(ctx, r, ids, moveTimeout(commanded))
	if err != nil {
		// Write what was collected before failing. A transient decode error late in a sweep
		// would otherwise discard that trial's whole series along with the run.
		for _, p := range plans {
			writeSamples(samplesCSV, trial, p.id, series[p.id])
		}
		return fmt.Errorf("%s: %w", trial, err)
	}

	var arrivals []time.Duration
	for _, p := range plans {
		s := series[p.id]
		writeSamples(samplesCSV, trial, p.id, s)
		prof := analyzeMove(s, targets[p.id])
		startSec, endSec := "", ""
		if prof.moved {
			startSec = strconv.FormatFloat(s[prof.startIdx].t.Seconds(), 'f', 6, 64)
			endSec = strconv.FormatFloat(s[prof.endIdx].t.Seconds(), 'f', 6, 64)
		}
		ratio := 0.0
		if commanded > 0 {
			ratio = prof.duration.Seconds() / commanded.Seconds()
		}
		// Arrival spread must be measured in ABSOLUTE time, not per-joint duration.
		// Every joint shares this trial's sampling t0, so the coordinated-arrival
		// question is "did they all finish at the same moment", which is the spread of
		// end timestamps. prof.duration subtracts each joint's own motion-onset
		// detection latency, and under -full-arm that latency is systematically
		// joint-dependent (same commanded time, different travel -> gentler ramps cross
		// velEpsilon later), so a duration spread is biased.
		if prof.moved {
			arrivals = append(arrivals, s[prof.endIdx].t)
		}
		summary.write([]string{
			trial, strconv.Itoa(p.id),
			transportName(!cfg.stockTransport), strconv.Itoa(len(s)),
			strconv.Itoa(ms),
			// The ACTUAL per-joint travel. Under -full-arm each joint gets a different
			// fraction of the nominal, and that difference IS the experiment -- writing
			// travelDeg here would record 40.0 for joints that moved 8, 16, 24 and 32.
			strconv.FormatFloat(stepsToDeg(targets[p.id]-p.start), 'f', 2, 64),
			strconv.FormatFloat(prof.duration.Seconds()*1000, 'f', 1, 64),
			// Absolute motion bounds: the arrival spread the -full-arm run exists to
			// measure is a group-by over motion_end_sec, and must not be recomputed from
			// the deliberately-biased duration column.
			startSec, endSec,
			strconv.FormatBool(prof.moved),
			strconv.FormatBool(timedOut),
			strconv.FormatFloat(ratio, 'f', 3, 64),
			strconv.FormatBool(prof.exceededGoalTime(commanded)),
			strconv.Itoa(prof.peakVel),
			strconv.Itoa(prof.overshootSteps),
		})
		if len(ids) == 1 {
			fmt.Printf("%-22s %10v %10v %8.3f %8v\n",
				trial, commanded, prof.duration, ratio, prof.exceededGoalTime(commanded))
		}
	}

	if len(ids) > 1 {
		if len(arrivals) < len(plans) {
			// Never report a spread over a subset: a joint that did not move at all
			// would silently tighten the number that decides coordinated arrival.
			fmt.Printf("%-22s %10v arrival spread: UNRELIABLE, only %d of %d joints moved\n",
				trial, commanded, len(arrivals), len(plans))
		} else {
			min, max := arrivals[0], arrivals[0]
			for _, a := range arrivals[1:] {
				if a < min {
					min = a
				}
				if a > max {
					max = a
				}
			}
			fmt.Printf("%-22s %10v arrival spread across %d joints: %v (absolute)\n",
				trial, commanded, len(arrivals), max-min)
		}
	}

	// Return to start under the same commanded time, so the next trial begins where this
	// one did.
	back := make(map[int]feetech.GoalRequest, len(plans))
	for _, p := range plans {
		back[p.id] = feetech.GoalRequest{Position: p.start, Time: ms, Acc: goalTimeAcc}
	}
	if err := r.group.SetGoals(ctx, back); err != nil {
		return fmt.Errorf("%s: return SetGoals: %w", trial, err)
	}
	if _, _, err := sampleUntilStopped(ctx, r, ids, moveTimeout(commanded)); err != nil {
		return fmt.Errorf("%s: return: %w", trial, err)
	}
	return verifyAtStart(ctx, r, plans)
}

// --- test: accel ---

// Descending: Acc=0 is an unlimited ramp and the jerkiest command in the harness, so it
// runs LAST rather than as the first motion of a bench session.
var accSweep = []int{255, 200, 100, 50, 20, 10, 5, 2, 1, 0}

// accelSpeedSteps caps the velocity target during the acceleration sweep. Acc=0 means an
// unlimited ramp; pairing that with a high velocity target is the one genuinely jerky
// combination in this harness, so the ceiling stays modest.
const accelSpeedSteps = 1000

func runAccel(cfg config) error {
	// Only cfg.servo is commanded, but the rig holds the whole arm so holdPose can keep
	// the other joints powered -- otherwise this sweep measures a sagging chain, which is
	// exactly wrong for the gravity-loaded joints the mapping depends on.
	ids := []int{cfg.servo}
	r, err := openRig(cfg.port, noGap, armServoIDs, !cfg.stockTransport) // fast-flush samples ~10x faster
	if err != nil {
		return err
	}
	defer r.close()

	summary, err := newCSV(cfg.outDir, csvName("accel", "", cfg.servo, false, transportName(!cfg.stockTransport)), []string{
		"trial", "servo_id", "transport", "n_samples", "acc_units", "travel_deg", "peak_vel_steps_per_sec",
		"time_to_peak_sec", "implied_accel_steps_per_sec2", "steps_per_sec2_per_unit",
		"ramp_below_sampling_resolution", "sampling_timed_out", "duration_sec", "overshoot_steps",
	})
	if err != nil {
		return err
	}
	defer summary.close()

	samplesCSV, err := newCSV(cfg.outDir, csvName("accel", "_samples", cfg.servo, false, transportName(!cfg.stockTransport)), sampleHeader)
	if err != nil {
		return err
	}
	defer samplesCSV.close()

	fmt.Printf("\n%-10s %10s %12s %14s %12s\n",
		"acc", "peak_vel", "time_to_peak", "implied_a", "a_per_unit")

	runErr := r.withSafeShutdown(func(ctx context.Context) error {
		if err := r.holdPose(ctx); err != nil {
			return err
		}

		steps := degToSteps(cfg.travelDeg)
		for _, acc := range accSweep {
			if ctx.Err() != nil {
				return ctx.Err()
			}
			trial := fmt.Sprintf("acc%d", acc)

			plans, err := prepareTravel(ctx, r, ids, steps, cfg.assumeLimits)
			if err != nil {
				return err
			}
			p := plans[0]

			goal := map[int]feetech.GoalRequest{
				p.id: {Position: p.target, Speed: accelSpeedSteps, Acc: acc},
			}
			if err := r.group.SetGoals(ctx, goal); err != nil {
				return fmt.Errorf("%s: SetGoals: %w", trial, err)
			}
			series, timedOut, err := sampleUntilStopped(ctx, r, ids, accelTimeout(steps))
			if err != nil {
				writeSamples(samplesCSV, trial, p.id, series[p.id])
				return fmt.Errorf("%s: %w", trial, err)
			}

			s := series[p.id]
			writeSamples(samplesCSV, trial, p.id, s)
			prof := analyzeMove(s, p.target)
			implied := prof.impliedAccel()
			perUnit := 0.0
			if acc > 0 {
				perUnit = implied / float64(acc)
			}
			fmt.Printf("%-10d %10d %12v %14.1f %12.1f\n",
				acc, prof.peakVel, prof.timeToPeak, implied, perUnit)
			summary.write([]string{
				trial, strconv.Itoa(p.id),
				transportName(!cfg.stockTransport), strconv.Itoa(len(s)),
				strconv.Itoa(acc),
				strconv.FormatFloat(cfg.travelDeg, 'f', 1, 64),
				strconv.Itoa(prof.peakVel),
				strconv.FormatFloat(prof.timeToPeak.Seconds(), 'f', 6, 64),
				strconv.FormatFloat(implied, 'f', 1, 64),
				strconv.FormatFloat(perUnit, 'f', 1, 64),
				strconv.FormatBool(prof.rampBelowSamplingResolution()),
				strconv.FormatBool(timedOut),
				strconv.FormatFloat(prof.duration.Seconds(), 'f', 6, 64),
				strconv.Itoa(prof.overshootSteps),
			})

			// Return to start at a fixed, gentle profile so the return leg does not
			// contaminate the next trial.
			back := map[int]feetech.GoalRequest{
				p.id: {Position: p.start, Speed: 500, Acc: 20},
			}
			if err := r.group.SetGoals(ctx, back); err != nil {
				return fmt.Errorf("%s: return SetGoals: %w", trial, err)
			}
			if _, _, err := sampleUntilStopped(ctx, r, ids, accelTimeout(steps)); err != nil {
				return fmt.Errorf("%s: return: %w", trial, err)
			}
			if err := verifyAtStart(ctx, r, plans); err != nil {
				return fmt.Errorf("%s: %w", trial, err)
			}
		}

		fmt.Printf("\nthe library documents Acc as ~100 steps/s^2 per unit; " +
			"compare the a_per_unit column and note where it saturates.\n")
		fmt.Printf("rows with ramp_below_sampling_resolution=true reached peak within one " +
			"sample: implied_accel reads 0 because it was too fast to measure, NOT because " +
			"there was no acceleration. Read peak_vel for those.\n")
		return nil
	})

	return firstErr(runErr, summary.close(), samplesCSV.close())
}

// selectTests resolves the -test flag into the list that will actually run. Motion tests
// are dropped without -move; "all" without -move degrades to writerate only and says so,
// rather than erroring, so "all" stays useful as a first run.
func selectTests(name string, move bool) (run, skipped []string, err error) {
	motion := map[string]bool{"goaltime": true, "accel": true}
	var requested []string
	switch name {
	case "all":
		// writerate LAST: it deliberately leaves the arm limp, and a motion test running
		// after it would holdPose the collapsed pose and measure the sweep from there.
		requested = []string{"goaltime", "accel", "writerate"}
	case "writerate", "goaltime", "accel":
		requested = []string{name}
	default:
		return nil, nil, fmt.Errorf("unknown -test %q (want writerate|goaltime|accel|all)", name)
	}

	for _, t := range requested {
		if motion[t] && !move {
			skipped = append(skipped, t)
			continue
		}
		run = append(run, t)
	}
	return run, skipped, nil
}

// isArmServo reports whether id is part of the 5-DOF arm chain. Servo 6 is the gripper.
func isArmServo(id int) bool {
	for _, a := range armServoIDs {
		if a == id {
			return true
		}
	}
	return false
}

// validateConfig rejects flag combinations that would run to completion and produce a
// confidently wrong result rather than an error.
func validateConfig(cfg config) error {
	if !isArmServo(cfg.servo) {
		return fmt.Errorf("-servo %d is not an arm servo %v (servo 6 is the gripper, and an "+
			"acceleration sweep would drive its jaws into their stop)", cfg.servo, armServoIDs)
	}
	if degToSteps(cfg.travelDeg) == 0 {
		return fmt.Errorf("-travel %g deg rounds to 0 encoder steps; every trial would "+
			"complete without moving and report a CSV of zeros", cfg.travelDeg)
	}
	return nil
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

// --- statistics ---

// latencyStats summarizes a set of durations. Nearest-rank percentiles, so every reported
// value is an actually-observed sample rather than an interpolation between two.
type latencyStats struct {
	n      int
	mean   time.Duration
	p50    time.Duration
	p95    time.Duration
	p99    time.Duration
	max    time.Duration
	rateHz float64 // 1/mean
}

// percentile returns the nearest-rank pth percentile of an already-sorted slice.
// p is in [0,100]. Panics on an empty slice, which is a programming error here.
func percentile(sorted []time.Duration, p float64) time.Duration {
	if len(sorted) == 0 {
		panic("percentile: empty slice")
	}
	idx := int(math.Ceil(p/100*float64(len(sorted)))) - 1
	if idx < 0 {
		idx = 0
	}
	if idx >= len(sorted) {
		idx = len(sorted) - 1
	}
	return sorted[idx]
}

func summarize(d []time.Duration) latencyStats {
	if len(d) == 0 {
		return latencyStats{}
	}
	sorted := append([]time.Duration(nil), d...)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i] < sorted[j] })

	var total time.Duration
	for _, v := range sorted {
		total += v
	}
	mean := total / time.Duration(len(sorted))

	st := latencyStats{
		n:    len(sorted),
		mean: mean,
		p50:  percentile(sorted, 50),
		p95:  percentile(sorted, 95),
		p99:  percentile(sorted, 99),
		max:  sorted[len(sorted)-1],
	}
	if mean > 0 {
		st.rateHz = float64(time.Second) / float64(mean)
	}
	return st
}

func init() {
	selftests = append(selftests,
		selftestCase{"percentile/nearest-rank", func() error {
			// 1..100 ms. Nearest-rank: p50 -> ceil(50)=50 -> idx 49 -> 50ms.
			var d []time.Duration
			for i := 1; i <= 100; i++ {
				d = append(d, time.Duration(i)*time.Millisecond)
			}
			for _, tc := range []struct {
				p    float64
				want time.Duration
			}{
				{50, 50 * time.Millisecond},
				{95, 95 * time.Millisecond},
				{99, 99 * time.Millisecond},
				{100, 100 * time.Millisecond},
				{0, 1 * time.Millisecond},
			} {
				if got := percentile(d, tc.p); got != tc.want {
					return fmt.Errorf("p%v = %v, want %v", tc.p, got, tc.want)
				}
			}
			return nil
		}},
		selftestCase{"summarize/unsorted-input", func() error {
			// Deliberately unsorted: summarize must sort its own copy and must not
			// mutate the caller's slice.
			in := []time.Duration{
				5 * time.Millisecond, 1 * time.Millisecond, 3 * time.Millisecond,
				2 * time.Millisecond, 4 * time.Millisecond,
			}
			orig := append([]time.Duration(nil), in...)
			st := summarize(in)
			for i := range in {
				if in[i] != orig[i] {
					return fmt.Errorf("summarize mutated its input at %d", i)
				}
			}
			if st.n != 5 {
				return fmt.Errorf("n = %d, want 5", st.n)
			}
			if st.mean != 3*time.Millisecond {
				return fmt.Errorf("mean = %v, want 3ms", st.mean)
			}
			if st.max != 5*time.Millisecond {
				return fmt.Errorf("max = %v, want 5ms", st.max)
			}
			// mean 3ms -> 333.33Hz
			if st.rateHz < 333.0 || st.rateHz > 334.0 {
				return fmt.Errorf("rateHz = %v, want ~333.3", st.rateHz)
			}
			return nil
		}},
		selftestCase{"summarize/empty", func() error {
			if got := summarize(nil); got.n != 0 {
				return fmt.Errorf("n = %d, want 0", got.n)
			}
			return nil
		}},
	)
}

// --- sample decoding ---

// sample is one timestamped reading of a servo's position and velocity.
type sample struct {
	t   time.Duration // since the start of the trial
	pos int           // raw encoder steps
	vel int           // raw steps/sec, signed
}

// decodeSignMagnitude mirrors feetech's unexported helper (servo.go:390). The high bit at
// signBit is a sign flag, not a two's-complement extension: the remaining bits are the
// magnitude.
func decodeSignMagnitude(value, signBit int) int {
	if signBit == 0 {
		return value
	}
	signMask := 1 << signBit
	if value&signMask != 0 {
		return -(value & (signMask - 1))
	}
	return value
}

// decodePosVel splits the 4-byte payload of a SyncRead at RegPresentPosition into
// position and velocity. Returns an error rather than silently decoding garbage if the
// servo returned a short payload.
func decodePosVel(proto *feetech.Protocol, data []byte) (pos, vel int, err error) {
	if len(data) < 4 {
		return 0, 0, fmt.Errorf("short payload: %d bytes, want 4", len(data))
	}
	pos = decodeSignMagnitude(int(proto.DecodeWord(data[0:2])), feetech.RegPresentPosition.SignBit)
	vel = decodeSignMagnitude(int(proto.DecodeWord(data[2:4])), feetech.RegPresentVelocity.SignBit)
	return pos, vel, nil
}

func degToSteps(deg float64) int   { return int(math.Round(deg * stepsPerDegree)) }
func stepsToDeg(steps int) float64 { return float64(steps) / stepsPerDegree }

func init() {
	selftests = append(selftests,
		selftestCase{"decodeSignMagnitude", func() error {
			for _, tc := range []struct{ in, bit, want int }{
				{100, 15, 100},     // positive
				{0x8064, 15, -100}, // sign bit set -> negative magnitude
				{0, 15, 0},         // zero
				{0x8000, 15, 0},    // negative zero decodes to 0
				{2048, 15, 2048},   // mid-range positive
				{43, 0, 43},        // signBit 0 -> passthrough (odd: also detects a missing guard)
			} {
				if got := decodeSignMagnitude(tc.in, tc.bit); got != tc.want {
					return fmt.Errorf("decodeSignMagnitude(%#x, %d) = %d, want %d",
						tc.in, tc.bit, got, tc.want)
				}
			}
			return nil
		}},
		selftestCase{"decodePosVel", func() error {
			proto := feetech.NewProtocol(feetech.ProtocolSTS)
			// STS is little-endian: pos = 0x0064 = 100, vel = 0x8064 = -100.
			data := []byte{0x64, 0x00, 0x64, 0x80}
			pos, vel, err := decodePosVel(proto, data)
			if err != nil {
				return err
			}
			if pos != 100 {
				return fmt.Errorf("pos = %d, want 100", pos)
			}
			if vel != -100 {
				return fmt.Errorf("vel = %d, want -100", vel)
			}
			if _, _, err := decodePosVel(proto, []byte{0x00, 0x01}); err == nil {
				return fmt.Errorf("short payload accepted, want error")
			}
			return nil
		}},
		selftestCase{"degToSteps roundtrip", func() error {
			// 360 deg = 4096 steps exactly.
			if got := degToSteps(360); got != 4096 {
				return fmt.Errorf("degToSteps(360) = %d, want 4096", got)
			}
			if got := degToSteps(20); got != 228 { // 20 * 11.3777... = 227.55 -> 228
				return fmt.Errorf("degToSteps(20) = %d, want 228", got)
			}
			if d := stepsToDeg(4096); math.Abs(d-360) > 1e-9 {
				return fmt.Errorf("stepsToDeg(4096) = %v, want 360", d)
			}
			return nil
		}},
	)
}

// --- transport ---

// fastFlushTransport implements feetech.Transport over go.bug.st/serial, differing from the
// library's own transport in exactly one respect: Flush uses ResetInputBuffer, an ioctl that
// returns immediately, instead of a read-and-discard loop that waits out a read timeout.
//
// This matters more than it sounds. feetech v0.6.0's SerialTransport.Flush
// (transports/serial_os.go:82) loops on Read until it returns 0, and go.bug.st/serial's
// unixPort.Read blocks in Select and returns 0 only once the deadline expires
// (serial_unix.go:59) -- so on an idle bus every flush costs the full 10ms. sendPacketLocked
// calls Flush before EVERY packet, which puts a ~95Hz floor on all bus traffic regardless of
// MinCommandGap. Sweeping both transports is what separates "this bus is slow" from "this
// library's flush is slow".
type fastFlushTransport struct {
	port    serial.Port
	timeout time.Duration
}

func openFastFlush(portPath string, baud int, timeout time.Duration) (*fastFlushTransport, error) {
	p, err := serial.Open(portPath, &serial.Mode{BaudRate: baud})
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", portPath, err)
	}
	if err := p.SetReadTimeout(timeout); err != nil {
		p.Close()
		return nil, fmt.Errorf("set read timeout on %s: %w", portPath, err)
	}
	return &fastFlushTransport{port: p, timeout: timeout}, nil
}

func (t *fastFlushTransport) Read(b []byte) (int, error)  { return t.port.Read(b) }
func (t *fastFlushTransport) Write(b []byte) (int, error) { return t.port.Write(b) }
func (t *fastFlushTransport) Close() error                { return t.port.Close() }

func (t *fastFlushTransport) SetReadTimeout(d time.Duration) error {
	t.timeout = d
	return t.port.SetReadTimeout(d)
}

// Flush discards buffered input with an ioctl, returning immediately.
func (t *fastFlushTransport) Flush() error { return t.port.ResetInputBuffer() }

// --- bus lifecycle ---

// armServoIDs is the arm chain. Servo 6 is the gripper and is never commanded here.
var armServoIDs = []int{1, 2, 3, 4, 5}

type rig struct {
	bus   *feetech.Bus
	group *feetech.ServoGroup
	proto *feetech.Protocol
	ids   []int
}

// openRig connects to the bus with an explicit command gap. Pass noGap, never 0 — a zero
// value is silently coerced to 1ms by feetech.NewBus (bus.go:63).
func openRig(port string, gap time.Duration, ids []int, fastFlush bool) (*rig, error) {
	if gap == 0 {
		return nil, fmt.Errorf("internal error: MinCommandGap 0 is coerced to 1ms; pass noGap")
	}
	cfg := feetech.BusConfig{
		Port:          port,
		BaudRate:      1000000,
		Protocol:      feetech.ProtocolSTS,
		Timeout:       time.Second,
		MinCommandGap: gap,
	}
	// A supplied Transport wins over cfg.Port inside NewBus.
	if fastFlush {
		tr, err := openFastFlush(port, cfg.BaudRate, cfg.Timeout)
		if err != nil {
			return nil, err
		}
		cfg.Transport = tr
	}
	bus, err := feetech.NewBus(cfg)
	if err != nil {
		return nil, fmt.Errorf("open bus on %s: %w", port, err)
	}

	servos := make([]*feetech.Servo, 0, len(ids))
	for _, id := range ids {
		servos = append(servos, feetech.NewServo(bus, id, &feetech.ModelSTS3215))
	}
	return &rig{
		bus:   bus,
		group: feetech.NewServoGroup(bus, servos...),
		proto: bus.Protocol(),
		ids:   ids,
	}, nil
}

func (r *rig) close() error { return r.bus.Close() }

// disableTorque drops torque on every servo in the rig, best-effort. Used both as the
// deliberate resting state for writerate and as the teardown for motion tests.
func (r *rig) disableTorque(ctx context.Context) {
	for _, id := range r.ids {
		s := r.group.ServoByID(id)
		if s == nil {
			continue
		}
		if err := s.SetTorqueEnabled(ctx, false); err != nil {
			fmt.Fprintf(os.Stderr, "warning: disable torque on servo %d: %v\n", id, err)
		}
	}
}

// holdSeedSpeedSteps / holdSeedAcc shape the goal seeded just before torque is enabled.
// Gentle on purpose: this is the one write that lands while the operator may have hands on
// the arm.
const (
	holdSeedSpeedSteps = 300
	holdSeedAcc        = 20
)

// holdPose enables torque on every servo in the rig so the arm holds its posture while a
// single joint is exercised. Without it the untested joints hang limp and the joint under
// test accelerates a collapsing chain -- which corrupts exactly the gravity-loaded
// measurement the acceleration sweep exists to make, with no visible symptom in the data.
//
// It also warns that torque is released on exit: withSafeShutdown drops it on the normal
// return path too, so a gravity-loaded arm falls the moment the summary finishes printing.
func (r *rig) holdPose(ctx context.Context) error {
	// Seed every goal register with where the servo actually IS before energising.
	// SetTorqueEnabled makes an STS3215 seek its goal-position register, which still holds
	// whatever was last written -- so on an arm that viam-server drove and the operator then
	// back-drove by hand, a bare torque-enable snaps all five joints to a stale goal at full
	// speed, on an arm someone may be holding.
	positions, err := r.group.Positions(ctx)
	if err != nil {
		return fmt.Errorf("reading positions before holding pose: %w", err)
	}
	seed := make(map[int]feetech.GoalRequest, len(positions))
	for id, pos := range positions {
		seed[id] = feetech.GoalRequest{Position: pos, Speed: holdSeedSpeedSteps, Acc: holdSeedAcc}
	}
	if err := r.group.SetGoals(ctx, seed); err != nil {
		return fmt.Errorf("seeding goal positions before holding pose: %w", err)
	}

	for _, id := range r.ids {
		s := r.group.ServoByID(id)
		if s == nil {
			return fmt.Errorf("servo %d not in group", id)
		}
		if err := s.SetTorqueEnabled(ctx, true); err != nil {
			return fmt.Errorf("enable torque on servo %d: %w", id, err)
		}
	}
	fmt.Printf("torque ON for servos %v -- RELEASED when this run ends, so support the arm\n", r.ids)
	return nil
}

// withSafeShutdown runs fn with a context cancelled on SIGINT/SIGTERM, and guarantees
// torque is dropped on every exit path — normal return, error, or signal.
func (r *rig) withSafeShutdown(fn func(ctx context.Context) error) error {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	defer func() {
		// A fresh context: the shutdown context may already be cancelled, and the
		// torque-off write must still go out.
		r.disableTorque(context.Background())
	}()
	return fn(ctx)
}

// --- limits and travel planning ---

// travelPlan is a validated out-and-back move for one servo.
type travelPlan struct {
	id     int
	start  int // raw encoder steps, read at startup
	target int // raw encoder steps
}

// limitsValid mirrors the guard at config.go:424. Angle limits can legitimately read 0/0
// on a multi-turn servo or one with limits disabled; trusting those would clamp every
// target to zero.
func limitsValid(min, max int) bool {
	return min < max && max <= 4095
}

// parseAssumeLimits parses the -assume-limits "<min>:<max>" override.
func parseAssumeLimits(s string) (min, max int, err error) {
	if s == "" {
		return 0, 0, fmt.Errorf("empty")
	}
	parts := strings.Split(s, ":")
	if len(parts) != 2 {
		return 0, 0, fmt.Errorf("want <min>:<max>, got %q", s)
	}
	if min, err = strconv.Atoi(strings.TrimSpace(parts[0])); err != nil {
		return 0, 0, fmt.Errorf("min: %w", err)
	}
	if max, err = strconv.Atoi(strings.TrimSpace(parts[1])); err != nil {
		return 0, 0, fmt.Errorf("max: %w", err)
	}
	if !limitsValid(min, max) {
		return 0, 0, fmt.Errorf("invalid bounds %d:%d (need min < max <= 4095)", min, max)
	}
	return min, max, nil
}

// planTravel validates that start +/- travel fits inside [min,max] and returns the plan.
// It REFUSES rather than clamping: a silently shortened trial reports a real duration for
// a move that never happened, which is worse than no measurement.
func planTravel(id, start, travelSteps, min, max int) (travelPlan, error) {
	if !limitsValid(min, max) {
		return travelPlan{}, fmt.Errorf(
			"servo %d: angle limits read %d/%d, which fail the min<max<=4095 guard; "+
				"pass -assume-limits=<min>:<max> to proceed", id, min, max)
	}
	if start < min || start > max {
		return travelPlan{}, fmt.Errorf(
			"servo %d: start position %d is outside limits [%d,%d]", id, start, min, max)
	}
	target := start + travelSteps
	if target < min || target > max {
		return travelPlan{}, fmt.Errorf(
			"servo %d: travel of %d steps from %d would reach %d, outside limits [%d,%d]; "+
				"reduce -travel or reposition the arm", id, travelSteps, start, target, min, max)
	}
	return travelPlan{id: id, start: start, target: target}, nil
}

func init() {
	selftests = append(selftests,
		selftestCase{"limitsValid", func() error {
			for _, tc := range []struct {
				min, max int
				want     bool
			}{
				{0, 4095, true},
				{100, 3000, true},
				{0, 0, false},      // limits disabled / multi-turn
				{3000, 100, false}, // inverted
				{0, 4096, false},   // beyond resolution
				{500, 500, false},  // degenerate
			} {
				if got := limitsValid(tc.min, tc.max); got != tc.want {
					return fmt.Errorf("limitsValid(%d,%d) = %v, want %v",
						tc.min, tc.max, got, tc.want)
				}
			}
			return nil
		}},
		selftestCase{"planTravel/refuses-rather-than-clamps", func() error {
			// Fits: 2048 + 228 = 2276, inside [0,4095].
			p, err := planTravel(1, 2048, 228, 0, 4095)
			if err != nil {
				return fmt.Errorf("valid plan rejected: %w", err)
			}
			if p.target != 2276 {
				return fmt.Errorf("target = %d, want 2276", p.target)
			}
			// Would clip: must error, NOT clamp to 4095.
			if _, err := planTravel(1, 4000, 228, 0, 4095); err == nil {
				return fmt.Errorf("clipping travel accepted, want refusal")
			}
			// Start outside the limits must be refused even when the target lands
			// back inside: -100 + 228 = 128, which looks valid. Only the start guard
			// catches this, so this case is what keeps that guard honest.
			if _, err := planTravel(1, -100, 228, 0, 4095); err == nil {
				return fmt.Errorf("start outside limits accepted, want refusal")
			}
			// Negative direction clips low.
			if _, err := planTravel(1, 100, -228, 0, 4095); err == nil {
				return fmt.Errorf("negative clipping travel accepted, want refusal")
			}
			// Invalid limits (0/0) must be reported as such, not clamp everything to 0.
			_, err = planTravel(1, 2048, 228, 0, 0)
			if err == nil {
				return fmt.Errorf("invalid limits accepted, want refusal")
			}
			if !strings.Contains(err.Error(), "assume-limits") {
				return fmt.Errorf("invalid-limits error should name -assume-limits, got: %v", err)
			}
			return nil
		}},
		selftestCase{"parseAssumeLimits", func() error {
			min, max, err := parseAssumeLimits("100:3000")
			if err != nil || min != 100 || max != 3000 {
				return fmt.Errorf("got (%d,%d,%v), want (100,3000,nil)", min, max, err)
			}
			for _, bad := range []string{"", "100", "100:200:300", "100:50", "a:b", "0:5000"} {
				if _, _, err := parseAssumeLimits(bad); err == nil {
					return fmt.Errorf("parseAssumeLimits(%q) accepted, want error", bad)
				}
			}
			return nil
		}},
	)
}

// --- motion setup ---

// prepareTravel reads the start position and angle limits for each servo and validates
// the requested travel. Returns an error rather than clamping (see planTravel).
func prepareTravel(ctx context.Context, r *rig, ids []int, travelSteps int, assume string) ([]travelPlan, error) {
	positions, err := r.group.Positions(ctx)
	if err != nil {
		return nil, fmt.Errorf("read positions: %w", err)
	}

	var overrideMin, overrideMax int
	haveOverride := false
	if assume != "" {
		overrideMin, overrideMax, err = parseAssumeLimits(assume)
		if err != nil {
			return nil, fmt.Errorf("-assume-limits: %w", err)
		}
		haveOverride = true
	}

	plans := make([]travelPlan, 0, len(ids))
	for _, id := range ids {
		start, ok := positions[id]
		if !ok {
			return nil, fmt.Errorf("servo %d did not report a position", id)
		}
		min, max := overrideMin, overrideMax
		if !haveOverride {
			s := r.group.ServoByID(id)
			if s == nil {
				return nil, fmt.Errorf("servo %d not in group", id)
			}
			if min, max, err = s.PositionLimits(ctx); err != nil {
				return nil, fmt.Errorf("read limits for servo %d: %w", id, err)
			}
		}
		p, err := planTravel(id, start, travelSteps, min, max)
		if err != nil {
			return nil, err
		}
		plans = append(plans, p)
	}
	return plans, nil
}

// returnToleranceSteps is how far a joint may sit from its start position after a return
// leg before the sweep is aborted. ~3 degrees: the gravity-loaded joints 2 and 3 that Task
// 12 targets droop under steady-state load, and aborting a 15-trial sweep over normal droop
// would waste the bench session. Still far tighter than the 10-40 degree travels in play,
// so a genuinely stuck or contaminated joint is still caught.
const returnToleranceSteps = 34

// verifyAtStart re-reads positions and fails if any joint did not make it back.
//
// The return leg can finish without actually arriving: sampleUntilStopped returns nil on
// timeout once motion has been seen, so a timed-out return is indistinguishable from a
// completed one. prepareTravel would then read a contaminated position as the NEXT trial's
// start, making its travel, target and overshoot all wrong with nothing in the CSV to say so.
// One bad trial would quietly contaminate the whole remaining sweep.
func verifyAtStart(ctx context.Context, r *rig, plans []travelPlan) error {
	positions, err := r.group.Positions(ctx)
	if err != nil {
		return fmt.Errorf("verifying return to start: %w", err)
	}
	for _, p := range plans {
		got, ok := positions[p.id]
		if !ok {
			return fmt.Errorf("verifying return to start: servo %d did not report a position", p.id)
		}
		if d := got - p.start; d > returnToleranceSteps || d < -returnToleranceSteps {
			return fmt.Errorf("servo %d did not return to start: at %d, want %d (off by %d steps, "+
				"tolerance %d); continuing would measure every later trial from a contaminated "+
				"position", p.id, got, p.start, d, returnToleranceSteps)
		}
	}
	return nil
}

// accelTimeout bounds an accel trial. Derived, not hardcoded: a low-Acc trial is
// triangular, so it takes ~2*sqrt(travel/a). If the register's real scale turns out to be
// ~10 steps/s^2 per unit rather than the documented ~100, Acc=1 over 228 steps takes ~9.6s
// -- right on the old hardcoded 10s. The timeout would then have expired exactly where the
// interesting answer lives, and gravity-loaded joints are slower still.
func accelTimeout(travelSteps int) time.Duration {
	if travelSteps < 0 {
		travelSteps = -travelSteps
	}
	// Worst case assumed: 1 steps/s^2 per Acc unit at Acc=1.
	worst := 2 * math.Sqrt(float64(travelSteps))
	t := time.Duration(worst*3) * time.Second
	if t < 10*time.Second {
		t = 10 * time.Second
	}
	if t > 60*time.Second {
		t = 60 * time.Second
	}
	return t
}

// moveTimeout bounds a trial generously: commanded time plus slack, floored so that a
// clipped move still has room to finish and be recognised as clipped.
func moveTimeout(commanded time.Duration) time.Duration {
	t := commanded*3 + 2*time.Second
	if t < 5*time.Second {
		t = 5 * time.Second
	}
	return t
}

// --- profile recovery ---

// velEpsilon is the |velocity| below which a servo is considered stopped. The STS3215
// reports velocity in whole steps/sec, so anything non-zero is real motion.
const velEpsilon = 1

// moveProfile is what one commanded move looks like after the fact.
type moveProfile struct {
	startIdx, endIdx int           // sample indices bounding the motion
	duration         time.Duration // motion start to motion end
	peakVel          int           // max |velocity| observed, steps/sec
	velAtPeakTime    int           // |velocity| actually sampled at timeToPeak
	timeToPeak       time.Duration // motion start to first sample within 95% of peak
	overshootSteps   int           // furthest travel beyond target, 0 if none
	moved            bool          // false if the servo never moved at all
}

// analyzeMove recovers the motion profile from a sample series. target is the commanded
// position in raw encoder steps; direction is inferred from the first and last positions.
func analyzeMove(samples []sample, target int) moveProfile {
	var p moveProfile
	if len(samples) == 0 {
		return p
	}

	first, last := -1, -1
	for i, s := range samples {
		if abs(s.vel) >= velEpsilon {
			if first < 0 {
				first = i
			}
			last = i
		}
	}
	if first < 0 {
		return p // never moved
	}
	p.moved = true
	p.startIdx, p.endIdx = first, last
	p.duration = samples[last].t - samples[first].t

	for _, s := range samples {
		if v := abs(s.vel); v > p.peakVel {
			p.peakVel = v
		}
	}

	// Time to reach 95% of peak, measured from motion start.
	threshold := p.peakVel * 95 / 100
	for i := first; i <= last; i++ {
		if v := abs(samples[i].vel); v >= threshold {
			p.timeToPeak = samples[i].t - samples[first].t
			p.velAtPeakTime = v
			break
		}
	}

	// Overshoot is travel past the target in the direction of motion.
	forward := samples[last].pos >= samples[first].pos
	for _, s := range samples {
		var over int
		if forward {
			over = s.pos - target
		} else {
			over = target - s.pos
		}
		if over > p.overshootSteps {
			p.overshootSteps = over
		}
	}
	return p
}

// impliedAccel returns the implied acceleration in steps/sec^2 from the ramp to peak
// velocity. Returns 0 when the ramp was instantaneous (below sampling resolution), which
// is itself the finding for Acc=0.
func (p moveProfile) impliedAccel() float64 {
	if p.timeToPeak <= 0 {
		return 0
	}
	// Divide the velocity ACTUALLY sampled at timeToPeak, not peakVel. timeToPeak is when
	// velocity first crossed 95% of peak, so peakVel/timeToPeak overstates a linear ramp's
	// acceleration by 1/0.95 -- a systematic ~5.3% high bias in every row of the column
	// Task 12 turns into a calibration constant.
	return float64(p.velAtPeakTime) / p.timeToPeak.Seconds()
}

// rampBelowSamplingResolution reports that peak velocity was reached within a single
// sample period, so impliedAccel has no ramp to measure and returns 0. That zero means
// "faster than we can see", not "no acceleration" -- without this flag the fastest Acc rows
// plot at the bottom of the axis, indistinguishable from a move that never happened.
func (p moveProfile) rampBelowSamplingResolution() bool {
	return p.moved && p.timeToPeak <= 0
}

// exceededGoalTime reports whether the servo could not honor a commanded GoalTime. The 1.15x + 20ms
// allowance absorbs sampling granularity and the servo's own settling.
//
// Integer math, deliberately: time.Duration(float64(commanded)*1.15) truncates and lands a
// nanosecond short for several round values (450ms yields 537.499999ms, not 537.5ms), which
// makes exact-boundary reasoning wrong in a way that only shows up at the boundary.
func (p moveProfile) exceededGoalTime(commanded time.Duration) bool {
	if !p.moved || commanded <= 0 {
		return false
	}
	allowed := commanded*115/100 + 20*time.Millisecond
	return p.duration > allowed
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func init() {
	// synthProfile builds a trapezoidal series: 10ms sampling, ramp 0->1000 steps/s over
	// samples 1..10, cruise, then stop. Hand-computed expectations below.
	synthProfile := func() []sample {
		var out []sample
		pos := 0
		add := func(i, vel int) {
			out = append(out, sample{t: time.Duration(i) * 10 * time.Millisecond, pos: pos, vel: vel})
			pos += vel / 100 // 10ms of travel at vel steps/s
		}
		add(0, 0) // stopped
		for i := 1; i <= 10; i++ {
			add(i, i*100) // ramp: 100..1000
		}
		for i := 11; i <= 15; i++ {
			add(i, 1000) // cruise
		}
		add(16, 0) // stopped
		return out
	}

	selftests = append(selftests,
		selftestCase{"analyzeMove/trapezoid", func() error {
			s := synthProfile()
			p := analyzeMove(s, 1<<30) // target far away -> no overshoot
			if !p.moved {
				return fmt.Errorf("moved = false, want true")
			}
			// Motion spans sample 1 (t=10ms) to sample 15 (t=150ms).
			if want := 140 * time.Millisecond; p.duration != want {
				return fmt.Errorf("duration = %v, want %v", p.duration, want)
			}
			if p.peakVel != 1000 {
				return fmt.Errorf("peakVel = %d, want 1000", p.peakVel)
			}
			// 95% of 1000 = 950; first sample >= 950 is vel=1000 at t=100ms.
			// Measured from motion start (t=10ms) -> 90ms.
			if want := 90 * time.Millisecond; p.timeToPeak != want {
				return fmt.Errorf("timeToPeak = %v, want %v", p.timeToPeak, want)
			}
			// 1000 steps/s over 0.09s.
			if got := p.impliedAccel(); math.Abs(got-11111.11) > 1 {
				return fmt.Errorf("impliedAccel = %v, want ~11111.11", got)
			}
			return nil
		}},
		selftestCase{"analyzeMove/never-moved", func() error {
			s := []sample{{t: 0, pos: 100, vel: 0}, {t: 10 * time.Millisecond, pos: 100, vel: 0}}
			p := analyzeMove(s, 100)
			if p.moved {
				return fmt.Errorf("moved = true, want false")
			}
			if p.impliedAccel() != 0 {
				return fmt.Errorf("impliedAccel = %v, want 0", p.impliedAccel())
			}
			if p.exceededGoalTime(time.Second) {
				return fmt.Errorf("clipped = true for a move that never happened")
			}
			return nil
		}},
		selftestCase{"analyzeMove/overshoot", func() error {
			// Forward motion overshooting a target of 100 by 5.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 50, vel: 500},
				{t: 20 * time.Millisecond, pos: 105, vel: 500},
				{t: 30 * time.Millisecond, pos: 100, vel: -10},
				{t: 40 * time.Millisecond, pos: 100, vel: 0},
			}
			p := analyzeMove(s, 100)
			if p.overshootSteps != 5 {
				return fmt.Errorf("overshootSteps = %d, want 5", p.overshootSteps)
			}
			return nil
		}},
		selftestCase{"moveProfile/exceededGoalTime", func() error {
			p := moveProfile{moved: true, duration: 500 * time.Millisecond}
			// Commanded 200ms, took 500ms -> clipped.
			if !p.exceededGoalTime(200 * time.Millisecond) {
				return fmt.Errorf("clipped = false, want true for 500ms vs commanded 200ms")
			}
			// Commanded 800ms, took 500ms -> honored.
			if p.exceededGoalTime(800 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false for 500ms vs commanded 800ms")
			}
			// Just inside the allowance: 500ms vs commanded 450ms -> 450*1.15+20 = 537ms.
			if p.exceededGoalTime(450 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false at the allowance boundary")
			}
			return nil
		}},
	)
}

func init() {
	selftests = append(selftests,
		selftestCase{"analyzeMove/reverse-motion", func() error {
			// Reverse move: negative velocities, overshooting a target of 100 by 5.
			s := []sample{
				{t: 0, pos: 200, vel: 0},
				{t: 10 * time.Millisecond, pos: 150, vel: -500},
				{t: 20 * time.Millisecond, pos: 95, vel: -500},
				{t: 30 * time.Millisecond, pos: 100, vel: 10},
				{t: 40 * time.Millisecond, pos: 100, vel: 0},
			}
			p := analyzeMove(s, 100)
			if !p.moved {
				return fmt.Errorf("moved = false, want true (negative velocities are motion)")
			}
			if p.peakVel != 500 {
				return fmt.Errorf("peakVel = %d, want 500", p.peakVel)
			}
			if want := 20 * time.Millisecond; p.duration != want {
				return fmt.Errorf("duration = %v, want %v", p.duration, want)
			}
			if p.overshootSteps != 5 {
				return fmt.Errorf("overshootSteps = %d, want 5 (reverse overshoot)", p.overshootSteps)
			}
			// Peak reached on the first moving sample: instantaneous ramp -> 0, not +Inf.
			if got := p.impliedAccel(); got != 0 {
				return fmt.Errorf("impliedAccel = %v, want 0 for an instantaneous ramp", got)
			}
			return nil
		}},
		selftestCase{"analyzeMove/velEpsilon-boundary", func() error {
			// |vel| == velEpsilon is motion, so the move spans t=10ms..30ms.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 0, vel: 1},
				{t: 20 * time.Millisecond, pos: 5, vel: 500},
				{t: 30 * time.Millisecond, pos: 10, vel: 1},
				{t: 40 * time.Millisecond, pos: 10, vel: 0},
			}
			p := analyzeMove(s, 1<<30)
			if want := 20 * time.Millisecond; p.duration != want {
				return fmt.Errorf("duration = %v, want %v", p.duration, want)
			}
			return nil
		}},
		selftestCase{"analyzeMove/95-percent-band", func() error {
			// 95% of 1000 is 950, met at t=20ms by vel=960 -- one sample before the peak.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 0, vel: 500},
				{t: 20 * time.Millisecond, pos: 5, vel: 960},
				{t: 30 * time.Millisecond, pos: 15, vel: 1000},
				{t: 40 * time.Millisecond, pos: 25, vel: 0},
			}
			p := analyzeMove(s, 1<<30)
			if want := 10 * time.Millisecond; p.timeToPeak != want {
				return fmt.Errorf("timeToPeak = %v, want %v", p.timeToPeak, want)
			}
			return nil
		}},
		selftestCase{"analyzeMove/peak-at-last-moving-sample", func() error {
			// The 95% band is first met by the final moving sample.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 0, vel: 100},
				{t: 20 * time.Millisecond, pos: 1, vel: 500},
				{t: 30 * time.Millisecond, pos: 6, vel: 1000},
				{t: 40 * time.Millisecond, pos: 16, vel: 0},
			}
			p := analyzeMove(s, 1<<30)
			if want := 20 * time.Millisecond; p.timeToPeak != want {
				return fmt.Errorf("timeToPeak = %v, want %v", p.timeToPeak, want)
			}
			if got := p.impliedAccel(); math.Abs(got-50000) > 1 {
				return fmt.Errorf("impliedAccel = %v, want ~50000", got)
			}
			return nil
		}},
		selftestCase{"analyzeMove/overshoot-after-motion-ends", func() error {
			// The furthest travel past target lands in a sample already reading vel=0.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 50, vel: 500},
				{t: 20 * time.Millisecond, pos: 98, vel: 500},
				{t: 30 * time.Millisecond, pos: 103, vel: 0},
				{t: 40 * time.Millisecond, pos: 103, vel: 0},
			}
			p := analyzeMove(s, 100)
			if p.overshootSteps != 3 {
				return fmt.Errorf("overshootSteps = %d, want 3", p.overshootSteps)
			}
			return nil
		}},
		selftestCase{"analyzeMove/bounds-indices", func() error {
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 5, vel: 500},
				{t: 20 * time.Millisecond, pos: 10, vel: 500},
				{t: 30 * time.Millisecond, pos: 10, vel: 0},
			}
			p := analyzeMove(s, 10)
			if p.startIdx != 1 || p.endIdx != 2 {
				return fmt.Errorf("startIdx,endIdx = %d,%d; want 1,2", p.startIdx, p.endIdx)
			}
			return nil
		}},
		selftestCase{"moveProfile/exceededGoalTime-allowance", func() error {
			p := moveProfile{moved: true, duration: 500 * time.Millisecond}
			// 400*1.15+20 = 480ms < 500ms -> clipped. Pins the 1.15 factor from above.
			if !p.exceededGoalTime(400 * time.Millisecond) {
				return fmt.Errorf("clipped = false, want true for 500ms vs commanded 400ms")
			}
			// 420*1.15 = 483ms, +20ms = 503ms > 500ms -> honored. Pins the +20ms.
			if p.exceededGoalTime(420 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false for 500ms vs commanded 420ms")
			}
			// Exactly at the allowance (300*1.15+20 = 365ms) is not clipped.
			if (moveProfile{moved: true, duration: 365 * time.Millisecond}).exceededGoalTime(300 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false exactly at the allowance")
			}
			// No GoalTime commanded -> nothing to clip.
			if p.exceededGoalTime(0) {
				return fmt.Errorf("clipped = true, want false for commanded 0")
			}
			// A profile that never moved is never clipped, whatever its duration.
			if (moveProfile{moved: false, duration: 500 * time.Millisecond}).exceededGoalTime(200 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false when moved = false")
			}
			return nil
		}},
		selftestCase{"analyzeMove/empty", func() error {
			if p := analyzeMove(nil, 0); p.moved {
				return fmt.Errorf("moved = true for an empty series")
			}
			return nil
		}},
	)
}

// --- CSV output ---

// csvWriter wraps a file and encoding/csv, accumulating the first error so callers can
// write a whole table and check once at the end.
type csvWriter struct {
	f      *os.File
	w      *csv.Writer
	err    error
	closed bool
}

func newCSV(dir, name string, header []string) (*csvWriter, error) {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return nil, fmt.Errorf("create out dir %s: %w", dir, err)
	}
	path := filepath.Join(dir, name)
	// O_EXCL rather than os.Create, which truncates. These files ARE the deliverable, and
	// the file is opened before the serial port is, so a mistyped -port would otherwise
	// destroy a good prior run before failing. Refusing here matches planTravel: this
	// harness does not silently discard data to keep going.
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0o644)
	if os.IsExist(err) {
		return nil, fmt.Errorf("%s already exists; refusing to overwrite bench data "+
			"(move it aside, or pass a different -out)", path)
	}
	if err != nil {
		return nil, fmt.Errorf("create %s: %w", path, err)
	}
	c := &csvWriter{f: f, w: csv.NewWriter(f)}
	c.write(header)
	fmt.Printf("writing %s\n", path)
	return c, nil
}

func (c *csvWriter) write(row []string) {
	if c.err != nil {
		return
	}
	c.err = c.w.Write(row)
}

// close is idempotent, so callers can both `defer c.close()` as a leak guard and call it
// explicitly to check the accumulated error. A CSV write failure must not be silent — the
// files are the durable artifact of this whole harness.
func (c *csvWriter) close() error {
	if c.closed {
		return c.err
	}
	c.closed = true
	c.w.Flush()
	if c.err == nil {
		c.err = c.w.Error()
	}
	if cerr := c.f.Close(); c.err == nil {
		c.err = cerr
	}
	return c.err
}

// writeSamples dumps a sample series with a trial label, for plotting.
func writeSamples(c *csvWriter, trial string, servoID int, samples []sample) {
	for _, s := range samples {
		c.write([]string{
			trial,
			strconv.Itoa(servoID),
			strconv.FormatFloat(s.t.Seconds(), 'f', 6, 64),
			strconv.Itoa(s.pos),
			strconv.FormatFloat(stepsToDeg(s.pos), 'f', 4, 64),
			strconv.Itoa(s.vel),
		})
	}
}

var sampleHeader = []string{"trial", "servo_id", "t_sec", "pos_steps", "pos_deg", "vel_steps_per_sec"}

// csvName builds a per-run filename. Task 12 sweeps several servos into a single -out
// directory, so the servo and mode must appear in the name or each run would collide with
// the one before it.
func csvName(test, suffix string, servo int, fullArm bool, transport string) string {
	var scope string
	switch {
	case fullArm:
		scope = "_fullarm"
	case servo > 0:
		scope = fmt.Sprintf("_servo%d", servo)
	}
	return fmt.Sprintf("%s%s_%s%s.csv", test, scope, transport, suffix)
}

// transportName labels which transport produced a measurement. It belongs in both the
// filename and the data: the transport changes the sample rate by 3-10x, and therefore
// changes measured_ms, peak_vel and time_to_peak_sec. A resolution-limited measurement
// should carry its resolution.
func transportName(fastFlush bool) string {
	if fastFlush {
		return "fastflush"
	}
	return "stock"
}

func init() {
	selftests = append(selftests,
		selftestCase{"csvWriter/roundtrip", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", sampleHeader)
			if err != nil {
				return err
			}
			writeSamples(c, "trial1", 3, []sample{
				{t: 1500 * time.Millisecond, pos: -42, vel: 7},
				{t: 2 * time.Second, pos: 100, vel: -8},
			})
			if err := c.close(); err != nil {
				return fmt.Errorf("close: %w", err)
			}
			got, err := os.ReadFile(filepath.Join(dir, "t.csv"))
			if err != nil {
				return err
			}
			want := "trial,servo_id,t_sec,pos_steps,pos_deg,vel_steps_per_sec\n" +
				"trial1,3,1.500000,-42,-3.6914,7\n" +
				"trial1,3,2.000000,100,8.7891,-8\n"
			if string(got) != want {
				return fmt.Errorf("file =\n%q\nwant\n%q", got, want)
			}
			return nil
		}},
		selftestCase{"csvWriter/close-is-idempotent", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			if err := c.close(); err != nil {
				return fmt.Errorf("first close: %w", err)
			}
			// Tasks 10 and 11 both defer close() AND call it explicitly inside
			// firstErr(...). The explicit call is evaluated first as a return argument,
			// so the deferred one's value is discarded -- the guard's real jobs are not
			// double-closing the fd, and returning the SAME stored error on a repeat
			// call, which csvWriter/flush-failure-surfaces pins.
			if err := c.close(); err != nil {
				return fmt.Errorf("second close: %w", err)
			}
			return nil
		}},
	)
}

func init() {
	selftests = append(selftests,
		selftestCase{"csvWriter/close-reports-write-failure", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			// Close the file out from under the writer, then write. A silent CSV failure
			// would look like a successful run that happened to produce no data.
			if err := c.f.Close(); err != nil {
				return err
			}
			c.write([]string{"b"})
			if err := c.close(); err == nil {
				return fmt.Errorf("close() = nil, want the underlying write failure")
			}
			return nil
		}},
		selftestCase{"csvWriter/first-error-wins", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			boom := fmt.Errorf("boom")
			c.err = boom
			c.write([]string{"b"}) // must be dropped, not masked by succeeding
			if got := c.close(); got != boom {
				return fmt.Errorf("close() = %v, want the first error (%v)", got, boom)
			}
			got, err := os.ReadFile(filepath.Join(dir, "t.csv"))
			if err != nil {
				return err
			}
			if string(got) != "a\n" {
				return fmt.Errorf("file = %q, want %q (a row written after an error must be dropped)", got, "a\n")
			}
			return nil
		}},
		selftestCase{"csvWriter/flush-failure-surfaces", func() error {
			// A pipe with the read end closed: writes fail with EPIPE while Close() on
			// the write end still succeeds, which isolates a flush error from a close error.
			r, w, err := os.Pipe()
			if err != nil {
				return err
			}
			if err := r.Close(); err != nil {
				return err
			}
			c := &csvWriter{f: w, w: csv.NewWriter(w)}
			c.write([]string{"a"}) // buffered; the failure only appears at flush
			first := c.close()
			if first == nil {
				return fmt.Errorf("close() = nil, want the flush failure")
			}
			// An idempotent close must REPORT the stored error again, not swallow it.
			if second := c.close(); second != first {
				return fmt.Errorf("second close() = %v, want the same error (%v)", second, first)
			}
			return nil
		}},
		selftestCase{"csvWriter/write-error-is-recorded", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			// An invalid delimiter makes csv.Writer.Write fail without touching the file,
			// so this is the one path where write() recording the error is what surfaces it.
			c.w.Comma = 0
			c.write([]string{"b"})
			c.w.Comma = ','
			if err := c.close(); err == nil {
				return fmt.Errorf("close() = nil, want the recorded write error")
			}
			return nil
		}},
		selftestCase{"newCSV/creates-missing-dir", func() error {
			root, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(root)

			// -out may name a directory that does not exist yet; newCSV must create it.
			dir := filepath.Join(root, "nested", "out")
			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return fmt.Errorf("newCSV into a missing dir: %w", err)
			}
			if err := c.close(); err != nil {
				return err
			}
			if _, err := os.Stat(filepath.Join(dir, "t.csv")); err != nil {
				return fmt.Errorf("file not created: %w", err)
			}
			return nil
		}},
	)
}

func init() {
	selftests = append(selftests,
		selftestCase{"selectTests", func() error {
			for _, tc := range []struct {
				name         string
				move         bool
				run, skipped []string
				wantErr      bool
			}{
				{"writerate", false, []string{"writerate"}, nil, false},
				{"all", false, []string{"writerate"}, []string{"goaltime", "accel"}, false},
				{"all", true, []string{"goaltime", "accel", "writerate"}, nil, false},
				{"goaltime", false, nil, []string{"goaltime"}, false},
				{"goaltime", true, []string{"goaltime"}, nil, false},
				{"bogus", true, nil, nil, true},
			} {
				run, skipped, err := selectTests(tc.name, tc.move)
				if (err != nil) != tc.wantErr {
					return fmt.Errorf("selectTests(%q,%v) err = %v, wantErr %v", tc.name, tc.move, err, tc.wantErr)
				}
				if strings.Join(run, ",") != strings.Join(tc.run, ",") {
					return fmt.Errorf("selectTests(%q,%v) run = %v, want %v", tc.name, tc.move, run, tc.run)
				}
				if strings.Join(skipped, ",") != strings.Join(tc.skipped, ",") {
					return fmt.Errorf("selectTests(%q,%v) skipped = %v, want %v", tc.name, tc.move, skipped, tc.skipped)
				}
			}
			return nil
		}},
		selftestCase{"validateConfig", func() error {
			ok := config{servo: 1, travelDeg: 20}
			if err := validateConfig(ok); err != nil {
				return fmt.Errorf("valid config rejected: %w", err)
			}
			// Servo 6 is the gripper; an Acc sweep would drive its jaws into their stop.
			if err := validateConfig(config{servo: 6, travelDeg: 20}); err == nil {
				return fmt.Errorf("-servo 6 (gripper) accepted, want refusal")
			}
			if err := validateConfig(config{servo: 0, travelDeg: 20}); err == nil {
				return fmt.Errorf("-servo 0 accepted, want refusal")
			}
			// Travel that rounds to zero steps would report a CSV of zeros, not an error.
			if err := validateConfig(config{servo: 1, travelDeg: 0}); err == nil {
				return fmt.Errorf("-travel 0 accepted, want refusal")
			}
			if err := validateConfig(config{servo: 1, travelDeg: 0.02}); err == nil {
				return fmt.Errorf("-travel 0.02 (rounds to 0 steps) accepted, want refusal")
			}
			return nil
		}},
		selftestCase{"csvName", func() error {
			for _, tc := range []struct {
				test, suffix string
				servo        int
				fullArm      bool
				transport    string
				want         string
			}{
				{"writerate", "", 0, false, "both", "writerate_both.csv"},
				{"accel", "", 2, false, "fastflush", "accel_servo2_fastflush.csv"},
				{"accel", "_samples", 2, false, "stock", "accel_servo2_stock_samples.csv"},
				{"goaltime", "", 1, true, "fastflush", "goaltime_fullarm_fastflush.csv"},
				{"goaltime", "_samples", 1, true, "stock", "goaltime_fullarm_stock_samples.csv"},
				// Same test and servo on different transports must not collide: the
				// transport changes the sample rate and therefore the measurements.
				{"accel", "", 2, false, "stock", "accel_servo2_stock.csv"},
			} {
				if got := csvName(tc.test, tc.suffix, tc.servo, tc.fullArm, tc.transport); got != tc.want {
					return fmt.Errorf("csvName(%q,%q,%d,%v,%q) = %q, want %q",
						tc.test, tc.suffix, tc.servo, tc.fullArm, tc.transport, got, tc.want)
				}
			}
			if csvName("accel", "", 2, false, "stock") == csvName("accel", "", 2, false, "fastflush") {
				return fmt.Errorf("csvName collides across transports")
			}
			return nil
		}},
		selftestCase{"newCSV/refuses-to-clobber", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			if err := c.close(); err != nil {
				return err
			}
			// Task 12 sweeps several servos into one -out dir, and the file is opened
			// before the serial port is -- so a silent truncate here would destroy a good
			// prior run before the bad one even failed.
			if _, err := newCSV(dir, "t.csv", []string{"a"}); err == nil {
				return fmt.Errorf("newCSV overwrote an existing file, want refusal")
			}
			return nil
		}},
		selftestCase{"rampBelowSamplingResolution", func() error {
			// Peak on the first moving sample: no ramp visible, impliedAccel is 0.
			fast := analyzeMove([]sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 10, vel: 1000},
				{t: 20 * time.Millisecond, pos: 20, vel: 0},
			}, 1<<30)
			if !fast.rampBelowSamplingResolution() {
				return fmt.Errorf("rampBelowSamplingResolution = false for an instantaneous ramp")
			}
			if fast.impliedAccel() != 0 {
				return fmt.Errorf("impliedAccel = %v, want 0", fast.impliedAccel())
			}
			// A measurable ramp must NOT be flagged.
			slow := analyzeMove([]sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 1, vel: 100},
				{t: 20 * time.Millisecond, pos: 6, vel: 1000},
				{t: 30 * time.Millisecond, pos: 16, vel: 0},
			}, 1<<30)
			if slow.rampBelowSamplingResolution() {
				return fmt.Errorf("rampBelowSamplingResolution = true for a measurable ramp")
			}
			// A move that never happened is not a fast ramp.
			if (moveProfile{}).rampBelowSamplingResolution() {
				return fmt.Errorf("rampBelowSamplingResolution = true for a move that never happened")
			}
			return nil
		}},
	)
}

// --- test: writerate ---

// gapSweep is the set of MinCommandGap values probed. The first entry is the "no gap"
// sentinel: a literal 0 is coerced to 1ms by feetech.NewBus (bus.go:63), which would make
// it a mislabeled duplicate of the 1ms row.
var gapSweep = []time.Duration{
	noGap,
	250 * time.Microsecond,
	500 * time.Microsecond,
	1 * time.Millisecond,
	2 * time.Millisecond,
}

const writeRateIterations = 500

func runWriteRate(cfg config) error {
	// Probe the port BEFORE creating the CSV. goaltime and accel open their rig first; doing
	// it in the other order here meant a mistyped -port left a header-only file behind, and
	// O_EXCL then refused the corrected retry.
	if probe, err := openRig(cfg.port, noGap, armServoIDs, false); err != nil {
		return err
	} else if err := probe.close(); err != nil {
		return err
	}

	out, err := newCSV(cfg.outDir, csvName("writerate", "", 0, false, "both"), []string{
		"transport", "gap_label", "gap_sec", "n", "mean_sec", "p50_sec", "p95_sec", "p99_sec", "max_sec", "rate_hz",
	})
	if err != nil {
		return err
	}
	defer out.close()

	fmt.Printf("\n%-10s %-10s %8s %10s %10s %10s %10s %10s\n",
		"transport", "gap", "rate_hz", "mean", "p50", "p95", "p99", "max")

	type mode struct {
		name      string
		fastFlush bool
	}
	// Both transports, because the difference between them IS the finding: feetech's own
	// Flush blocks for 10ms before every packet, so the stock rows measure that floor while
	// the fastflush rows measure the bus.
	for _, m := range []mode{{"stock", false}, {"fastflush", true}} {
		for _, gap := range gapSweep {
			st, err := measureWriteRate(cfg.port, gap, m.fastFlush)
			if err != nil {
				return fmt.Errorf("%s transport, gap %v: %w", m.name, gap, err)
			}
			label := gap.String()
			if gap == noGap {
				label = "none"
			}
			fmt.Printf("%-10s %-10s %8.1f %10v %10v %10v %10v %10v\n",
				m.name, label, st.rateHz, st.mean, st.p50, st.p95, st.p99, st.max)
			out.write([]string{
				m.name,
				label,
				strconv.FormatFloat(gap.Seconds(), 'g', -1, 64),
				strconv.Itoa(st.n),
				strconv.FormatFloat(st.mean.Seconds(), 'f', 9, 64),
				strconv.FormatFloat(st.p50.Seconds(), 'f', 9, 64),
				strconv.FormatFloat(st.p95.Seconds(), 'f', 9, 64),
				strconv.FormatFloat(st.p99.Seconds(), 'f', 9, 64),
				strconv.FormatFloat(st.max.Seconds(), 'f', 9, 64),
				strconv.FormatFloat(st.rateHz, 'f', 2, 64),
			})
		}
	}

	fmt.Printf("\ninterpreting this table:\n")
	fmt.Printf("  stock rows should be FLAT at ~95Hz whatever the gap. feetech's own\n")
	fmt.Printf("  SerialTransport.Flush waits out a 10ms read timeout before every packet,\n")
	fmt.Printf("  swamping every swept gap. That floor IS the finding, not a fault here.\n")
	fmt.Printf("  fastflush rows use ResetInputBuffer instead and should track the gap:\n")
	fmt.Printf("  ~1000Hz at 1ms, ~500Hz at 2ms. Only if a fastflush 'none' row matches its\n")
	fmt.Printf("  own 1ms row should you suspect the MinCommandGap zero-coercion.\n")
	fmt.Printf("  a 5-servo SetGoals is 48 bytes ~= 480us of wire time, so ~2080Hz is the\n")
	fmt.Printf("  physical ceiling; anything faster is host-side buffering, not the bus.\n")
	return out.close()
}

// measureWriteRate opens a fresh bus at the given gap and times back-to-back SetGoals
// writes that command each servo's current position, with torque off.
func measureWriteRate(port string, gap time.Duration, fastFlush bool) (latencyStats, error) {
	r, err := openRig(port, gap, armServoIDs, fastFlush)
	if err != nil {
		return latencyStats{}, err
	}
	defer r.close()

	var st latencyStats
	err = r.withSafeShutdown(func(ctx context.Context) error {
		// Torque off for the whole test: this measures the wire, not the motor.
		fmt.Printf("torque is being RELEASED on servos %v for the writerate sweep -- "+
			"support the arm; it will go limp now and stay limp\n", r.ids)
		r.disableTorque(ctx)

		positions, err := r.group.Positions(ctx)
		if err != nil {
			return fmt.Errorf("read positions: %w", err)
		}

		goals := make(map[int]feetech.GoalRequest, len(r.ids))
		for _, id := range r.ids {
			pos, ok := positions[id]
			if !ok {
				return fmt.Errorf("servo %d did not report a position", id)
			}
			// Commanding the current position: even with torque somehow enabled, there
			// is nowhere to move to.
			goals[id] = feetech.GoalRequest{Position: pos, Speed: 100, Acc: 10}
		}

		latencies := make([]time.Duration, 0, writeRateIterations)
		for i := 0; i < writeRateIterations; i++ {
			if ctx.Err() != nil {
				return ctx.Err()
			}
			start := time.Now()
			if err := r.group.SetGoals(ctx, goals); err != nil {
				return fmt.Errorf("SetGoals iteration %d: %w", i, err)
			}
			latencies = append(latencies, time.Since(start))
		}
		st = summarize(latencies)
		return nil
	})
	return st, err
}

// --- sampling ---

// sampleUntilStopped polls position and velocity as fast as the link allows until every
// servo has read zero velocity continuously for settleWindow, or timeout elapses.
//
// One SyncRead of 4 bytes at RegPresentPosition returns position AND velocity per servo,
// because addresses 56 and 58 are adjacent. Each sample carries its own measured
// timestamp, so sampling jitter appears in the data rather than being assumed uniform.
// The bool result reports that the trial hit its deadline with the servo still moving.
// It must reach the CSV: a truncated series yields duration ~= timeout, which
// exceededGoalTime then reads as the servo failing to honor GoalTime -- recording the
// harness giving up as the very boundary this test exists to find.
func sampleUntilStopped(
	ctx context.Context,
	r *rig,
	ids []int,
	timeout time.Duration,
) (map[int][]sample, bool, error) {
	out := make(map[int][]sample, len(ids))
	t0 := time.Now()
	deadline := t0.Add(timeout)
	sawMotion := false
	var lastMotionAt time.Duration

	for {
		if ctx.Err() != nil {
			return out, false, ctx.Err()
		}
		data, err := r.bus.SyncRead(ctx, feetech.RegPresentPosition.Address, 4, ids)
		now := time.Since(t0)
		if err != nil {
			return out, false, fmt.Errorf("sync read: %w", err)
		}

		allStopped := true
		for _, id := range ids {
			pos, vel, derr := decodePosVel(r.proto, data[id])
			if derr != nil {
				return out, false, fmt.Errorf("servo %d: %w", id, derr)
			}
			out[id] = append(out[id], sample{t: now, pos: pos, vel: vel})
			if abs(vel) >= velEpsilon {
				allStopped = false
				sawMotion = true
			}
		}

		if !allStopped {
			lastMotionAt = now
		}
		// Require motion to have started before treating "stopped" as "finished", so the
		// pre-motion latency window does not end the trial immediately.
		if sawMotion && now-lastMotionAt >= settleWindow {
			return out, false, nil
		}

		if time.Now().After(deadline) {
			if !sawMotion {
				return out, false, fmt.Errorf("no motion detected within %v", timeout)
			}
			return out, true, nil // timed out mid-move -- callers MUST record this
		}
	}
}

// settleWindow is how long velocity must read zero CONTINUOUSLY before a trial is treated
// as finished. Deliberately a duration, not a count of reads.
//
// A read count silently couples the settle criterion to the sample rate, and the motion
// tests default to the fast-flush transport, which samples 3-10x faster than the stock one:
// 5 reads is ~53ms at 95Hz but only ~5ms at 1kHz. Meanwhile the STS3215 reports velocity in
// whole steps/sec derived from position deltas, so at low speed it reads 0 for the whole gap
// between encoder ticks. Worst case in these sweeps is -full-arm joint 1 at 10 deg / 3000 ms,
// which travels 1/5 of nominal = 7.6 steps/s = one tick every ~132 ms. A count-based settle
// mistakes that for the end of the move, reports "GoalTime honored perfectly", and then
// commands a reversal into a still-moving servo.
//
// 300ms gives ~2.3x margin over that 132ms worst case.
const settleWindow = 300 * time.Millisecond

func init() {
	selftests = append(selftests,
		selftestCase{"impliedAccel/no-95-percent-bias", func() error {
			// Linear ramp a = 10000 steps/s^2 sampled every 10ms: 0,100,200,...,1000.
			// The 95% threshold (950) is first met by the 1000 sample at t=100ms, so
			// timeToPeak measured from motion start (t=10ms) is 90ms.
			//
			// Dividing peakVel by that time would read 1000/0.09 = 11111, overstating the
			// true 10000 by 1/0.95. Dividing the velocity ACTUALLY sampled at that instant
			// is exact whenever the crossing sample is the peak, and never biased high.
			var s []sample
			pos := 0
			for i := 0; i <= 10; i++ {
				s = append(s, sample{t: time.Duration(i) * 10 * time.Millisecond, pos: pos, vel: i * 100})
				pos += i
			}
			s = append(s, sample{t: 110 * time.Millisecond, pos: pos, vel: 0})

			p := analyzeMove(s, 1<<30)
			if p.velAtPeakTime != 1000 {
				return fmt.Errorf("velAtPeakTime = %d, want 1000", p.velAtPeakTime)
			}
			if want := 90 * time.Millisecond; p.timeToPeak != want {
				return fmt.Errorf("timeToPeak = %v, want %v", p.timeToPeak, want)
			}
			// Now a case where the crossing sample is NOT the peak: 0,960,1000 with the
			// threshold met at 960. peakVel/timeToPeak would read 1000/0.01 = 100000;
			// velAtPeakTime/timeToPeak reads 960/0.01 = 96000.
			// The 500 sample matters: motion must START before the crossing, or
			// timeToPeak is 0 and impliedAccel short-circuits to 0 via its guard.
			s2 := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 0, vel: 500},
				{t: 20 * time.Millisecond, pos: 5, vel: 960},
				{t: 30 * time.Millisecond, pos: 15, vel: 1000},
				{t: 40 * time.Millisecond, pos: 25, vel: 0},
			}
			p2 := analyzeMove(s2, 1<<30)
			if p2.velAtPeakTime != 960 {
				return fmt.Errorf("velAtPeakTime = %d, want 960 (the sample that crossed, not the peak)", p2.velAtPeakTime)
			}
			if got := p2.impliedAccel(); math.Abs(got-96000) > 1 {
				return fmt.Errorf("impliedAccel = %v, want 96000 (peakVel-based would give 100000)", got)
			}
			return nil
		}},
		selftestCase{"settleWindow/covers-worst-case-gap", func() error {
			// The slowest trial in either sweep is -full-arm joint 1 at 10 deg / 3000 ms:
			// it gets 1/5 of nominal travel, so ~7.6 steps/s, i.e. one encoder tick every
			// ~132ms. The settle window must exceed that or sampling ends mid-move.
			steps := degToSteps(10) / 5
			gap := time.Duration(float64(time.Second) * 3.0 / float64(steps))
			if settleWindow <= gap {
				return fmt.Errorf("settleWindow %v <= worst-case inter-tick gap %v; sampling would end mid-move",
					settleWindow, gap)
			}
			return nil
		}},
	)
}
