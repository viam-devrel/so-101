//go:build nlopt

package main

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/spatialmath"

	"so_arm/tools/internal/streamx"
)

// fakeStreamArm records every point the producer sends. Embedding arm.Arm leaves every
// other method nil; only the streamed RPC is implemented. errAfter > 0 makes it return
// rpcErr once it has received that many non-empty batches, to simulate the module aborting
// the RPC (e.g. servo.CheckTrajectoryTime).
type fakeStreamArm struct {
	arm.Arm
	mu        sync.Mutex
	sent      []arm.TrajectoryPoint
	cancelled bool
	errAfter  int
	rpcErr    error
}

func (f *fakeStreamArm) MoveThroughJointPositionsStreamed(ctx context.Context, batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response, _ map[string]interface{},
) error {
	n := 0
	for {
		select {
		case <-ctx.Done():
			f.mu.Lock()
			f.cancelled = true
			f.mu.Unlock()
			return ctx.Err()
		case b, ok := <-batches:
			if !ok {
				return nil
			}
			f.mu.Lock()
			f.sent = append(f.sent, b...)
			f.mu.Unlock()
			if len(b) == 0 {
				continue
			}
			n++
			if f.errAfter > 0 && n >= f.errAfter {
				return f.rpcErr
			}
			select {
			case responses <- arm.Response{}:
			case <-ctx.Done():
				return ctx.Err()
			}
		}
	}
}

// synth is a 100 Hz, 1-joint trajectory of n points; position i is i*0.01.
func synth(n int) []arm.TrajectoryPoint {
	out := make([]arm.TrajectoryPoint, n)
	for i := range out {
		out[i] = arm.TrajectoryPoint{Time: time.Duration(i) * 10 * time.Millisecond, Positions: []float64{float64(i) * 0.01}}
	}
	return out
}

const (
	testRunway = 200 * time.Millisecond
	testSend   = 20 * time.Millisecond
	testMargin = 300 * time.Millisecond
)

// fakeReplan answers every replan with retPts (default synth(50), 0.5s) after delay,
// recording the calls.
type fakeReplan struct {
	delay  time.Duration
	err    error
	retPts []arm.TrajectoryPoint
	mu     sync.Mutex
	calls  [][]streamx.Event
}

func (r *fakeReplan) fn(ctx context.Context, _ []float64, due []streamx.Event) ([][]float64, []arm.TrajectoryPoint, time.Duration, error) {
	r.mu.Lock()
	r.calls = append(r.calls, due)
	r.mu.Unlock()
	select {
	case <-time.After(r.delay):
	case <-ctx.Done():
		return nil, nil, 0, ctx.Err()
	}
	if r.err != nil {
		return nil, nil, 0, r.err
	}
	pts := r.retPts
	if pts == nil {
		pts = synth(50)
	}
	return [][]float64{{0}, {0.49}}, pts, r.delay, nil
}

// assertStreamedPrefix checks the arm saw an in-order, well-formed prefix of cur: after a
// cancel, cur extends past what was sent by up to a leg.
func assertStreamedPrefix(t *testing.T, f *fakeStreamArm, cur []arm.TrajectoryPoint) {
	t.Helper()
	f.mu.Lock()
	defer f.mu.Unlock()
	require.NotEmpty(t, f.sent)
	require.LessOrEqual(t, len(f.sent), len(cur))
	for i := range f.sent {
		assert.Equal(t, cur[i], f.sent[i], "point %d", i)
	}
	assert.Equal(t, time.Duration(0), f.sent[0].Time)
	for i := 1; i < len(f.sent); i++ {
		assert.Less(t, f.sent[i-1].Time, f.sent[i].Time, "strictly increasing at %d", i)
	}
}

// assertStreamed is the prefix check plus "nothing was left unsent".
func assertStreamed(t *testing.T, f *fakeStreamArm, cur []arm.TrajectoryPoint) {
	t.Helper()
	assertStreamedPrefix(t, f, cur)
	f.mu.Lock()
	defer f.mu.Unlock()
	assert.Len(t, f.sent, len(cur), "every point of the final trajectory was sent")
}

func TestProducerAppliesTwoEventsDueOnOneTickAsOneSplice(t *testing.T) {
	f := &fakeStreamArm{}
	r := &fakeReplan{delay: 50 * time.Millisecond}
	events := []streamx.Event{{At: 100 * time.Millisecond, SwitchGoal: true}, {At: 100 * time.Millisecond}}
	p := newProducer(f, synth(200), events, r.fn, testRunway, testSend, testMargin)
	require.NoError(t, p.run(context.Background()))

	cur, splices := p.result()
	require.Len(t, splices, 1)
	assert.Len(t, splices[0].events, 2, "both events ride one replan")
	want := 100*time.Millisecond + testRunway + testMargin
	assert.GreaterOrEqual(t, splices[0].tStitch, want)
	assert.Less(t, splices[0].tStitch, want+100*time.Millisecond, "t_e + runway + margin, to within a few ticks")
	assert.Equal(t, time.Duration(0), splices[0].stitchLate)
	assertStreamed(t, f, cur)
}

func TestProducerHoldsAnEventThatArrivesWhileASpliceIsPending(t *testing.T) {
	f := &fakeStreamArm{}
	r := &fakeReplan{delay: 300 * time.Millisecond} // the 200 ms event lands mid-flight
	obstacle, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 100}), r3.Vector{X: 10, Y: 10, Z: 10}, "held")
	require.NoError(t, err)
	events := []streamx.Event{{At: 100 * time.Millisecond, SwitchGoal: true}, {At: 200 * time.Millisecond, Obstacle: obstacle}}
	p := newProducer(f, synth(200), events, r.fn, testRunway, testSend, testMargin)
	require.NoError(t, p.run(context.Background()))

	cur, splices := p.result()
	require.Len(t, splices, 2, "held, then applied as its own splice")
	assert.True(t, splices[0].events[0].SwitchGoal)
	assert.NotNil(t, splices[1].events[0].Obstacle, "pins which event was held")
	assert.False(t, splices[1].events[0].SwitchGoal)
	assert.Greater(t, splices[1].tStitch, splices[0].tStitch)
	assertStreamed(t, f, cur)
}

func TestProducerReportsALateStitchAndDropsNothing(t *testing.T) {
	f := &fakeStreamArm{}
	// The stitch is runway+margin (500 ms) after the event; returning 1100 ms later is 600 ms late.
	r := &fakeReplan{delay: 1100 * time.Millisecond}
	events := []streamx.Event{{At: 100 * time.Millisecond, SwitchGoal: true}}
	p := newProducer(f, synth(200), events, r.fn, testRunway, testSend, testMargin)
	require.NoError(t, p.run(context.Background()))

	cur, splices := p.result()
	require.Len(t, splices, 1)
	assert.InDelta(t, float64(600*time.Millisecond), float64(splices[0].stitchLate), float64(50*time.Millisecond))
	assertStreamed(t, f, cur)
}

func TestProducerReplanErrorCancelsTheStream(t *testing.T) {
	f := &fakeStreamArm{}
	boom := errors.New("boom")
	r := &fakeReplan{delay: 10 * time.Millisecond, err: boom}
	events := []streamx.Event{{At: 100 * time.Millisecond, SwitchGoal: true}}
	p := newProducer(f, synth(200), events, r.fn, testRunway, testSend, testMargin)
	err := p.run(context.Background())
	require.ErrorIs(t, err, boom)
	f.mu.Lock()
	defer f.mu.Unlock()
	assert.True(t, f.cancelled, "the RPC was released by ctx, not by a closed channel")
}

// TestProducerFiresAnEventPastTheTrajectoryEnd pins hasUnfired's claim: the arm rests at
// the end of a short trajectory while batches stays open, and a later event still lands
// its splice past the end.
func TestProducerFiresAnEventPastTheTrajectoryEnd(t *testing.T) {
	f := &fakeStreamArm{}
	r := &fakeReplan{delay: 50 * time.Millisecond, retPts: synth(30)}
	events := []streamx.Event{{At: 400 * time.Millisecond}}
	p := newProducer(f, synth(20), events, r.fn, testRunway, testSend, testMargin) // synth(20) ends at 190ms
	require.NoError(t, p.run(context.Background()))

	cur, splices := p.result()
	require.Len(t, splices, 1)
	assert.Greater(t, splices[0].tStitch, 200*time.Millisecond)
	assertStreamed(t, f, cur)
}

// TestProducerSurfacesAnRPCError pins the servo.CheckTrajectoryTime-violation path: the
// module aborts MoveThroughJointPositionsStreamed mid-stream and run surfaces that error,
// wrapped with "stream: " per the precedence branch in producer.go's run.
func TestProducerSurfacesAnRPCError(t *testing.T) {
	rpcErr := errors.New("module rejected trajectory")
	f := &fakeStreamArm{errAfter: 3, rpcErr: rpcErr}
	r := &fakeReplan{delay: 10 * time.Millisecond}
	p := newProducer(f, synth(200), nil, r.fn, testRunway, testSend, testMargin)

	done := make(chan error, 1)
	go func() { done <- p.run(context.Background()) }()
	select {
	case err := <-done:
		require.Error(t, err)
		assert.ErrorIs(t, err, rpcErr)
		assert.ErrorContains(t, err, "module rejected trajectory")
	case <-time.After(5 * time.Second):
		t.Fatal("run did not return within 5s")
	}
}

// TestProducerLoopsWithTheNextHook pins the loop contract: the hook keeps enqueuing, so
// done is never true and run returns only on cancel, and each stitch lands at the end of
// the leg it was scheduled from.
func TestProducerLoopsWithTheNextHook(t *testing.T) {
	const (
		loopRunway = 50 * time.Millisecond
		loopMargin = 50 * time.Millisecond
		legEnd     = 190 * time.Millisecond // synth(20)'s last Time
	)
	f := &fakeStreamArm{}
	r := &fakeReplan{delay: 10 * time.Millisecond, retPts: synth(20)}
	p := newProducer(f, synth(20), nil, r.fn, loopRunway, testSend, loopMargin)
	p.next = func(cur []arm.TrajectoryPoint) (streamx.Event, bool) {
		return streamx.Event{At: max(0, cur[len(cur)-1].Time-loopRunway-loopMargin)}, true
	}

	// Cancel, not a deadline: run must surface context.Canceled through its "stream: " wrap.
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	time.AfterFunc(800*time.Millisecond, cancel)
	// result() is safe to call while run is live; -race is the assertion.
	poll := make(chan struct{})
	go func() {
		defer close(poll)
		for ctx.Err() == nil {
			p.result()
			time.Sleep(5 * time.Millisecond)
		}
	}()
	err := p.run(ctx)
	<-poll
	require.ErrorIs(t, err, context.Canceled, "cancel, wrapped as stream: ...")

	cur, splices := p.result()
	require.GreaterOrEqual(t, len(splices), 3, "at least three legs")
	end := legEnd
	for k, s := range splices {
		assert.GreaterOrEqual(t, s.tStitch, end, "splice %d stitches at or after leg %d's end", k, k)
		assert.Less(t, s.tStitch, end+4*testSend, "splice %d stitches within a few ticks of it", k)
		end = s.tStitch + legEnd
	}
	assertStreamedPrefix(t, f, cur)
}
