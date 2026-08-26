package testfake

import (
	"context"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// FakeServoOps records the single-servo operations servocmd.HandleServoCommand dispatches, so
// the servo_* DoCommand protocol can be tested without a serial bus. It implements
// servocmd.ServoOps.
type FakeServoOps struct {
	MovedID      int
	MovedPercent float64
	MovedSpeed   int
	MovedRaw     int
	StoppedID    int
	WaitedIDs    []int
	WaitedMs     int

	Percent float64
	Raw     int
	// Condition and LoadCondition are servo condition flags returned alongside a good reading,
	// the way an overloaded servo reports while still answering correctly.
	Condition     feetech.StatusError
	LoadCondition feetech.StatusError
	MoveErr       error
	PosErr        error
	MoveRaws      []int

	Moving    bool
	MovingErr error
	MovingIDs []int
	Load      int
	LoadErr   error
}

func (f *FakeServoOps) MoveServoPercent(_ context.Context, id int, percent float64, speed int) error {
	f.MovedID, f.MovedPercent, f.MovedSpeed = id, percent, speed
	return f.MoveErr
}

func (f *FakeServoOps) MoveServoRaw(_ context.Context, id, raw int) error {
	f.MovedID, f.MovedRaw = id, raw
	f.MoveRaws = append(f.MoveRaws, raw)
	return f.MoveErr
}

func (f *FakeServoOps) ServoPositionPercent(_ context.Context, id int) (float64, int, feetech.StatusError, error) {
	if f.PosErr != nil {
		return 0, 0, 0, f.PosErr
	}
	return f.Percent, f.Raw, f.Condition, nil
}

func (f *FakeServoOps) StopServo(_ context.Context, id int) error {
	f.StoppedID = id
	return nil
}

func (f *FakeServoOps) WaitForServosToStop(_ context.Context, ids []int, timeoutMs int) error {
	f.WaitedIDs, f.WaitedMs = ids, timeoutMs
	return nil
}

func (f *FakeServoOps) AnyServoMoving(_ context.Context, ids []int) (bool, error) {
	f.MovingIDs = ids
	return f.Moving, f.MovingErr
}

func (f *FakeServoOps) ServoLoad(_ context.Context, id int) (int, feetech.StatusError, error) {
	if f.LoadErr != nil {
		return 0, 0, f.LoadErr
	}
	return f.Load, f.LoadCondition, nil
}
