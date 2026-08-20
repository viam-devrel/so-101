package main

// Hardware probe for the gripper travel guard: inspect servo 6's calibration registers,
// watch live position, move to a tick, and reset the limits to the uncalibrated 0/4095 state
// the guard is meant to catch.
//
//	go run ./cmd/cli/gripper_limits.go -port /dev/tty.usbmodemXXXX
//	go run ./cmd/cli/gripper_limits.go -port ... -goto center
//	go run ./cmd/cli/gripper_limits.go -port ... -uncalibrate -yes
//	go run ./cmd/cli/gripper_limits.go -port ... -limits 2030,3481

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

const (
	encoderMax = 4095
	// The jaw's closed stop on a vendor half-turn-homed kit, not mid-travel; separate from the
	// module's gripperClosedStopTick since this is its own package.
	encoderCenter = 2048
	// gripperTravelTicks(), from the URDF joint limits. A recorded calibration measured
	// 2030..3481 (span 1451); the module's safe window uses 1200, under that observed span.
	modeledTravel = 1251

	arrivalTolerance = 20  // ticks
	stallLoad        = 300 // load reading that means straining, not holding

	// Goal velocity is a 16-bit register with the sign at bit 15, so the magnitude tops out
	// here; above it the servo reads a negative velocity. 0 means unlimited, not stopped.
	maxSpeedSteps = 32767
)

func main() {
	port := flag.String("port", "", "serial port (required)")
	id := flag.Int("id", 6, "servo id")
	baud := flag.Int("baud", 1000000, "baud rate")
	goTo := flag.String("goto", "", "move to a tick, or 'center' (of recorded limits) or 'encoder-center' (2048)")
	speed := flag.Int("speed", 200, "move speed in steps/s for -goto")
	uncalibrate := flag.Bool("uncalibrate", false, "write limits 0/4095, the state the guard targets (needs -yes)")
	limits := flag.String("limits", "", "write limits, as min,max (needs -yes)")
	yes := flag.Bool("yes", false, "confirm a write to EEPROM limits")
	timeout := flag.Int("timeout", 0, "ms to wait for -goto; 0 derives it from distance and speed")
	force := flag.Bool("force", false, "allow -goto outside known-safe bounds")
	flag.Parse()

	if *port == "" {
		flag.Usage()
		os.Exit(1)
	}

	ctx := context.Background()
	bus, err := feetech.NewBus(feetech.BusConfig{Port: *port, BaudRate: *baud})
	if err != nil {
		fail("open bus: %v", err)
	}
	defer bus.Close()

	servo := feetech.NewServo(bus, *id, &feetech.ModelSTS3215)

	min, max, err := servo.PositionLimits(ctx)
	if err != nil {
		fail("read position limits: %v", err)
	}
	offset, err := readOffset(ctx, servo)
	if err != nil {
		fail("read position offset: %v", err)
	}
	report(*id, min, max, offset)

	switch {
	case *limits != "":
		writeLimits(ctx, servo, *limits, min, max, *yes)
	case *uncalibrate:
		writeLimits(ctx, servo, "0,4095", min, max, *yes)
	case *goTo != "":
		move(ctx, servo, *goTo, *speed, min, max, *timeout, *force)
	default:
		monitor(ctx, servo)
	}
}

func report(id, min, max, offset int) {
	fmt.Printf("servo %d\n", id)
	fmt.Printf("  min_angle_limit  %d\n", min)
	fmt.Printf("  max_angle_limit  %d\n", max)
	fmt.Printf("  position_offset  %d\n", offset)
	fmt.Printf("  span             %d ticks (modeled jaw travel %d)\n", max-min, modeledTravel)

	if max > min {
		center := (min + max) / 2
		fmt.Printf("  range center     %d  (encoder center %d, delta %+d)\n",
			center, encoderCenter, center-encoderCenter)
	}

	switch {
	case min == 0 && max == encoderMax && offset != 0:
		fmt.Println("\n=> homing offset set, limits at factory 0/4095: the state the guard targets.")
	case min == 0 && max == encoderMax:
		fmt.Println("\n=> limits at factory 0/4095, no homing offset: uncalibrated.")
	case max <= min:
		fmt.Println("\n=> limits unset or inverted; the module rejects this upstream.")
	default:
		fmt.Println("\n=> limits look recorded; the guard should not engage.")
	}
}

func writeLimits(ctx context.Context, servo *feetech.Servo, spec string, oldMin, oldMax int, yes bool) {
	min, max, err := parseLimits(spec)
	if err != nil {
		fail("%v", err)
	}
	fmt.Printf("\nwrite limits %d,%d (currently %d,%d)\n", min, max, oldMin, oldMax)
	if !yes {
		fmt.Println("refusing without -yes: this writes EEPROM.")
		os.Exit(1)
	}
	fmt.Printf("restore with:  -limits %d,%d -yes\n", oldMin, oldMax)

	if err := servo.SetPositionLimits(ctx, min, max); err != nil {
		fail("write limits: %v", err)
	}
	gotMin, gotMax, err := servo.PositionLimits(ctx)
	if err != nil {
		fail("read back: %v", err)
	}
	if gotMin != min || gotMax != max {
		fail("read back %d,%d after writing %d,%d", gotMin, gotMax, min, max)
	}
	fmt.Printf("ok: now %d,%d\n", gotMin, gotMax)
}

func move(ctx context.Context, servo *feetech.Servo, spec string, speed, min, max, timeoutMs int, force bool) {
	known := max > min && !(min == 0 && max == encoderMax)

	var target int
	switch spec {
	case "center":
		if !known {
			fail("-goto center needs recorded limits; these are %d,%d", min, max)
		}
		target = (min + max) / 2
	case "encoder-center":
		target = encoderCenter
	default:
		n, err := strconv.Atoi(spec)
		if err != nil {
			fail("-goto wants a tick, 'center', or 'encoder-center'")
		}
		target = n
	}

	switch {
	case speed <= 0:
		fail("-speed 0 means unlimited to the servo, not stopped; pass 1..%d", maxSpeedSteps)
	case speed > maxSpeedSteps:
		fail("-speed %d exceeds the register's %d magnitude; above it the servo reads a negative velocity",
			speed, maxSpeedSteps)
	case !known && !force:
		fail("limits are %d,%d so the stops are unknown; re-run with -force if you accept the risk", min, max)
	case known && (target < min || target > max) && !force:
		fail("target %d is outside recorded limits %d..%d; re-run with -force", target, min, max)
	}

	pos0, err := servo.Position(ctx)
	if err != nil {
		fail("read position: %v", err)
	}
	if timeoutMs <= 0 {
		timeoutMs = moveTimeout(abs(target-pos0), speed)
	}

	fmt.Printf("\nmoving %d -> %d (%d ticks) at %d steps/s, timeout %dms\n",
		pos0, target, abs(target-pos0), speed, timeoutMs)
	if err := servo.SetTorqueEnabled(ctx, true); err != nil {
		fail("enable torque: %v", err)
	}
	if err := servo.SetPositionWithSpeed(ctx, target, speed); err != nil {
		fail("set position: %v", err)
	}

	pos, peakLoad, stillMoving := waitForStop(ctx, servo, target, timeoutMs)
	fmt.Println()

	offBy := abs(pos - target)
	switch {
	case offBy <= arrivalTolerance:
		fmt.Printf("arrived at %d\n", pos)
	case stillMoving:
		fmt.Printf("still moving after %dms, %d ticks short of %d. Raise -timeout or -speed.\n",
			timeoutMs, offBy, target)
	case peakLoad >= stallLoad:
		fmt.Printf("STALLED %d ticks short of %d, at position %d with load %d.\n",
			offBy, target, pos, peakLoad)
		fmt.Println("The jaw is straining against a mechanical stop -- this is the failure the guard exists to prevent.")
	default:
		fmt.Printf("stopped %d ticks short of %d, at position %d, load %d (not straining).\n",
			offBy, target, pos, peakLoad)
		fmt.Println("The servo clamps a goal to its own min/max angle limits, so a target outside them stops here without strain.")
	}

	if temp, terr := servo.Temperature(ctx); terr == nil {
		fmt.Printf("temperature %d C\n", temp)
	}
}

// moveTimeout allows twice the nominal travel time, floored so a short hop still gets a
// chance and capped so a stall cannot hang the probe.
func moveTimeout(distance, speed int) int {
	ms := int(2000.0 * float64(distance) / float64(speed))
	if ms < 1000 {
		return 1000
	}
	if ms > 15000 {
		return 15000
	}
	return ms
}

// waitForStop polls the servo's Moving register the way the module's WaitForServosToStop does,
// rather than sleeping a fixed interval. A stalled jaw reports not-moving, so the caller
// distinguishes arrival from a stall by position error and load.
func waitForStop(ctx context.Context, servo *feetech.Servo, target, timeoutMs int) (pos, peakLoad int, stillMoving bool) {
	deadline := time.Now().Add(time.Duration(timeoutMs) * time.Millisecond)
	tick := time.NewTicker(50 * time.Millisecond)
	defer tick.Stop()

	// The flag can read clear in the moment between the goal write and motion starting, so
	// require several consecutive clear reads.
	const settleReads = 3
	clear := 0

	for {
		<-tick.C
		p, perr := servo.Position(ctx)
		if perr != nil {
			fmt.Printf("\rread position: %v", perr)
			continue
		}
		pos = p
		if load, lerr := servo.Load(ctx); lerr == nil && abs(load) > peakLoad {
			peakLoad = abs(load)
		}
		fmt.Printf("\rposition %4d  (target %4d, error %+5d)  peak load %5d", pos, target, pos-target, peakLoad)

		moving, merr := servo.Moving(ctx)
		if merr != nil {
			fmt.Printf("\rread moving: %v", merr)
			continue
		}
		if moving {
			clear = 0
		} else if clear++; clear >= settleReads {
			return pos, peakLoad, false
		}

		if time.Now().After(deadline) {
			return pos, peakLoad, moving
		}
	}
}

func monitor(ctx context.Context, servo *feetech.Servo) {
	if err := servo.SetTorqueEnabled(ctx, false); err != nil {
		fail("disable torque: %v", err)
	}
	fmt.Println("\nTorque disabled. Move the jaw to each stop; note the extremes. Ctrl-C to stop.")

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt)
	tick := time.NewTicker(200 * time.Millisecond)
	defer tick.Stop()

	seenMin, seenMax := 1<<31-1, -1
	for {
		select {
		case <-sig:
			if seenMax < 0 {
				fmt.Println("\n\nno positions read")
				return
			}
			fmt.Printf("\n\nobserved %d..%d (span %d ticks, center %d)\n",
				seenMin, seenMax, seenMax-seenMin, (seenMin+seenMax)/2)
			fmt.Printf("modeled travel %d, encoder center %d\n", modeledTravel, encoderCenter)
			return
		case <-tick.C:
			pos, err := servo.Position(ctx)
			if err != nil {
				fmt.Printf("\rread position: %v", err)
				continue
			}
			if pos < seenMin {
				seenMin = pos
			}
			if pos > seenMax {
				seenMax = pos
			}
			fmt.Printf("\rposition %4d   observed %4d..%-4d span %-4d", pos, seenMin, seenMax, seenMax-seenMin)
		}
	}
}

// readOffset decodes position_offset, which is sign-magnitude with the sign at bit 11.
func readOffset(ctx context.Context, servo *feetech.Servo) (int, error) {
	data, err := servo.ReadRegister(ctx, "position_offset")
	if err != nil {
		return 0, err
	}
	if len(data) != 2 {
		return 0, fmt.Errorf("expected 2 bytes, got %d", len(data))
	}
	raw := uint16(data[0]) | uint16(data[1])<<8
	if raw&(1<<11) != 0 {
		return -int(raw &^ (1 << 11)), nil
	}
	return int(raw), nil
}

func parseLimits(spec string) (int, int, error) {
	parts := strings.Split(spec, ",")
	if len(parts) != 2 {
		return 0, 0, fmt.Errorf("-limits wants min,max")
	}
	min, err := strconv.Atoi(strings.TrimSpace(parts[0]))
	if err != nil {
		return 0, 0, fmt.Errorf("bad min: %v", err)
	}
	max, err := strconv.Atoi(strings.TrimSpace(parts[1]))
	if err != nil {
		return 0, 0, fmt.Errorf("bad max: %v", err)
	}
	if min < 0 || max > encoderMax || min >= max {
		return 0, 0, fmt.Errorf("limits must satisfy 0 <= min < max <= %d", encoderMax)
	}
	return min, max, nil
}

func abs(n int) int {
	if n < 0 {
		return -n
	}
	return n
}

func fail(format string, args ...any) {
	fmt.Printf("\n"+format+"\n", args...)
	os.Exit(1)
}
