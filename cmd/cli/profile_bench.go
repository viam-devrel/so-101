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
	"math"
	"os"
	"sort"
	"strings"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
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
	pos int           // raw encoder ticks
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
				{42, 0, 42},        // signBit 0 -> passthrough
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
