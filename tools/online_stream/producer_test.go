//go:build nlopt

package main

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"

	"so_arm/tools/internal/streamx"
)

// fakeStreamArm records every point the producer sends. Embedding arm.Arm leaves every
// other method nil; only the streamed RPC is implemented.
type fakeStreamArm struct {
	arm.Arm
	mu        sync.Mutex
	sent      []arm.TrajectoryPoint
	cancelled bool
}

func (f *fakeStreamArm) MoveThroughJointPositionsStreamed(ctx context.Context, batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response, _ map[string]interface{},
) error {
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
			if len(b) > 0 {
				select {
				case responses <- arm.Response{}:
				case <-ctx.Done():
					return ctx.Err()
				}
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

// fakeReplan answers every replan with 50 points (0.5 s) after delay, recording the calls.
type fakeReplan struct {
	delay time.Duration
	err   error
	mu    sync.Mutex
	calls [][]streamx.Event
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
	return [][]float64{{0}, {0.49}}, synth(50), r.delay, nil
}

// assertStreamed checks the arm saw exactly the final trajectory, in order, well-formed.
func assertStreamed(t *testing.T, f *fakeStreamArm, cur []arm.TrajectoryPoint) {
	t.Helper()
	f.mu.Lock()
	defer f.mu.Unlock()
	require.Equal(t, len(cur), len(f.sent), "every point of the final trajectory was sent")
	for i := range cur {
		assert.Equal(t, cur[i], f.sent[i], "point %d", i)
	}
	require.NotEmpty(t, f.sent)
	assert.Equal(t, time.Duration(0), f.sent[0].Time)
	for i := 1; i < len(f.sent); i++ {
		assert.Less(t, f.sent[i-1].Time, f.sent[i].Time, "strictly increasing at %d", i)
	}
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
	events := []streamx.Event{{At: 100 * time.Millisecond, SwitchGoal: true}, {At: 200 * time.Millisecond}}
	p := newProducer(f, synth(200), events, r.fn, testRunway, testSend, testMargin)
	require.NoError(t, p.run(context.Background()))

	cur, splices := p.result()
	require.Len(t, splices, 2, "held, then applied as its own splice")
	assert.True(t, splices[0].events[0].SwitchGoal)
	assert.Nil(t, splices[1].events[0].Obstacle)
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
