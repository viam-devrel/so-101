package arm

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"

	"so_arm/internal/servo"
	"so_arm/internal/testfake"
)

var streamEpoch = time.Date(2026, 9, 4, 12, 0, 0, 0, time.UTC)

// streamTestArm pins the clock to streamEpoch, makes every sleep return at once while
// recording its deadline, and parks the arm at `at` (radians) for the start gate. Both
// registers are seeded non-zero so "commanded 0" is distinguishable from "never written".
func streamTestArm(t *testing.T, ft *testfake.FakeTransport, at []float64) (*so101, func() []time.Time) {
	t.Helper()
	s := accelTestArm(t, ft)
	s.readJoints = func(context.Context) ([]float64, error) { return at, nil }
	var mu sync.Mutex
	var deadlines []time.Time
	s.clock = servo.Clock{
		Now: func() time.Time { return streamEpoch },
		SleepUntil: func(ctx context.Context, d time.Time) error {
			mu.Lock()
			deadlines = append(deadlines, d)
			mu.Unlock()
			return ctx.Err()
		},
	}
	for _, id := range s.armServoIDs {
		ft.SetRegister(id, feetech.RegAcceleration.Address, []byte{0x7F})
		ft.SetRegister(id, feetech.RegGoalVelocity.Address, testfake.EncodeWordLE(1234))
	}
	return s, func() []time.Time {
		mu.Lock()
		defer mu.Unlock()
		return append([]time.Time(nil), deadlines...)
	}
}

func pt(at time.Duration, q float64) arm.TrajectoryPoint {
	return arm.TrajectoryPoint{Time: at, Positions: []referenceframe.Input{q, q, q, q, q}}
}

// runStream owns both channels the way the rdk server does: feeds batches in order, closes
// the input, drains acks. Returns the call's error and the ack count.
func runStream(ctx context.Context, s *so101, batches ...[]arm.TrajectoryPoint) (error, int) {
	in := make(chan []arm.TrajectoryPoint)
	out := make(chan arm.Response)
	go func() {
		defer close(in)
		for _, b := range batches {
			select {
			case in <- b:
			case <-ctx.Done():
				return
			}
		}
	}()
	acks := 0
	drained := make(chan struct{})
	go func() {
		defer close(drained)
		for range out {
			acks++
		}
	}()
	err := s.MoveThroughJointPositionsStreamed(ctx, in, out, nil)
	close(out)
	<-drained
	return err, acks
}

func assertGoalWrites(t *testing.T, ft *testfake.FakeTransport, s *so101, want int, msgAndArgs ...interface{}) {
	t.Helper()
	for _, id := range s.armServoIDs {
		assert.Equal(t, want, ft.WriteCount(id, feetech.RegAcceleration.Address), msgAndArgs...)
	}
}

func assertUnboundedRegisters(t *testing.T, s *so101) {
	t.Helper()
	for _, id := range s.armServoIDs {
		acc, err := s.controller.ReadServoRegister(context.Background(), id, "acceleration")
		require.NoError(t, err)
		assert.Equal(t, byte(0), acc[0], "servo %d acceleration must be the UNLIMITED sentinel", id)
		vel, err := s.controller.ReadServoRegister(context.Background(), id, "goal_velocity")
		require.NoError(t, err)
		assert.Equal(t, []byte{0, 0}, vel, "servo %d speed must be the MAXIMUM sentinel", id)
	}
}

var zeros = []float64{0, 0, 0, 0, 0}

// Within the gate: one unbounded goal write per point, scheduled at now + Time, no reads.
func TestStreamedWritesOneUnboundedGoalPerPoint(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s, deadlines := streamTestArm(t, ft, zeros)

	before := ft.PacketCount()
	err, acks := runStream(context.Background(), s,
		[]arm.TrajectoryPoint{pt(0, 0.01), pt(10*time.Millisecond, 0.02), pt(20*time.Millisecond, 0.03)})
	moved := ft.PacketCount() - before
	require.NoError(t, err)

	assert.Equal(t, 1, acks)
	assertGoalWrites(t, ft, s, 3)
	assertUnboundedRegisters(t, s)
	// The gate read is the pinned seam, so the only non-goal packet is the final stop poll.
	assert.Equal(t, 3+1, moved, "three goal writes plus WaitForServosToStop's one Moving poll")
	assert.Equal(t, []time.Time{
		streamEpoch, streamEpoch.Add(10 * time.Millisecond), streamEpoch.Add(20 * time.Millisecond),
	}, deadlines())
}

// Outside the gate: exactly one profiled move (write + its stop poll) before the stream.
func TestStreamedGateProfilesALargeStartGap(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s, deadlines := streamTestArm(t, ft, zeros)

	before := ft.PacketCount()
	err, acks := runStream(context.Background(), s, []arm.TrajectoryPoint{pt(0, 0.3)}) // ~17 deg
	moved := ft.PacketCount() - before
	require.NoError(t, err)

	assert.Equal(t, 1, acks)
	assertGoalWrites(t, ft, s, 2)
	assertUnboundedRegisters(t, s) // the streamed write lands last
	assert.Equal(t, 2+2, moved, "profiled write + its stop poll, streamed write + final stop poll")
	assert.Equal(t, []time.Time{streamEpoch}, deadlines(), "the clock starts after the gate")
}

func TestStreamedEmptyBatchesGetNoAckAndDoNotSkipTheGate(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s, _ := streamTestArm(t, ft, zeros)

	err, acks := runStream(context.Background(), s,
		[]arm.TrajectoryPoint{},
		[]arm.TrajectoryPoint{pt(0, 0.3)},
		[]arm.TrajectoryPoint{},
		[]arm.TrajectoryPoint{pt(10*time.Millisecond, 0.31), pt(20*time.Millisecond, 0.32)})
	require.NoError(t, err)

	assert.Equal(t, 2, acks, "one ack per NON-EMPTY batch")
	assertGoalWrites(t, ft, s, 3+1, "gate still ran on the first point")
}

func TestStreamedRejectsNonZeroFirstTime(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s, _ := streamTestArm(t, ft, zeros)

	before := ft.PacketCount()
	err, acks := runStream(context.Background(), s, []arm.TrajectoryPoint{pt(10*time.Millisecond, 0.01)})
	require.Error(t, err)
	assert.Zero(t, acks)
	assert.Zero(t, ft.PacketCount()-before, "rejected before anything reached the bus")
}

func TestStreamedRejectsNonIncreasingTime(t *testing.T) {
	for name, bad := range map[string]time.Duration{"equal": 10 * time.Millisecond, "earlier": 5 * time.Millisecond} {
		t.Run(name, func(t *testing.T) {
			ft := testfake.NewFakeTransport()
			s, _ := streamTestArm(t, ft, zeros)

			err, _ := runStream(context.Background(), s,
				[]arm.TrajectoryPoint{pt(0, 0.01), pt(10*time.Millisecond, 0.02), pt(bad, 0.03)})
			require.Error(t, err)
			assertGoalWrites(t, ft, s, 2, "the two valid points were already on the bus")
		})
	}
}

func TestStreamedCancelMidStreamWritesNoFurtherGoal(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s, _ := streamTestArm(t, ft, zeros)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	sleeps := 0
	s.clock.SleepUntil = func(ctx context.Context, _ time.Time) error {
		sleeps++
		if sleeps == 2 {
			cancel() // the stream is cancelled while waiting for point 2
		}
		return ctx.Err()
	}

	err, _ := runStream(ctx, s,
		[]arm.TrajectoryPoint{pt(0, 0.01), pt(10*time.Millisecond, 0.02), pt(20*time.Millisecond, 0.03)})
	assert.True(t, errors.Is(err, context.Canceled), "got %v", err)
	assertGoalWrites(t, ft, s, 1)
}

// Stop must end a stream even while it is parked waiting for its producer: opMgr.CancelRunning
// blocks until the op returns, and cancelling the op's ctx does not close `batches`.
func TestStreamedStopEndsAStreamParkedOnItsProducer(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s, _ := streamTestArm(t, ft, zeros)

	in := make(chan []arm.TrajectoryPoint) // never fed
	out := make(chan arm.Response)
	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveThroughJointPositionsStreamed(context.Background(), in, out, nil) }()

	time.Sleep(20 * time.Millisecond)
	require.NoError(t, s.Stop(context.Background(), nil))

	select {
	case err := <-errCh:
		assert.True(t, errors.Is(err, context.Canceled), "got %v", err)
	case <-time.After(time.Second):
		t.Fatal("Stop did not end the parked stream")
	}
	assertGoalWrites(t, ft, s, 0)
}
