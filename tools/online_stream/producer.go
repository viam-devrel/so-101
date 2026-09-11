//go:build nlopt

package main

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"

	"so_arm/tools/internal/streamx"
)

// replanFunc plans from `from` (radians) once the listed events have been applied to the
// caller's goal/obstacle state, returning the waypoints, their trajex points (Time from 0),
// and planning+trajex latency. Called from one goroutine at a time.
type replanFunc func(ctx context.Context, from []float64, due []streamx.Event) ([][]float64, []arm.TrajectoryPoint, time.Duration, error)

// splice records one applied replan for the report.
type splice struct {
	events      []streamx.Event
	tStitch     time.Duration
	qs          []float64 // where the arm was predicted to be at tStitch (radians)
	planLatency time.Duration
	stitchLate  time.Duration // max(0, plan return - tStitch)
	waypoints   [][]float64
	newPts      []arm.TrajectoryPoint
}

// producer feeds a live trajectory to MoveThroughJointPositionsStreamed a runway ahead of
// the clock and splices replans into it without stopping. Everything under mu is shared
// with the replan goroutine.
type producer struct {
	arm                           arm.Arm
	events                        []streamx.Event
	replan                        replanFunc
	runway, sendEvery, planMargin time.Duration

	mu      sync.Mutex
	t0      time.Time // first send; zero until then
	cur     []arm.TrajectoryPoint
	sent    int
	fired   []bool
	pending bool          // a replan is in flight
	tStitch time.Duration // valid while pending
	splices []splice
	planErr error
}

func newProducer(a arm.Arm, initial []arm.TrajectoryPoint, events []streamx.Event, replan replanFunc,
	runway, sendEvery, planMargin time.Duration,
) *producer {
	return &producer{
		arm: a, events: events, replan: replan, runway: runway, sendEvery: sendEvery, planMargin: planMargin,
		cur: initial, fired: make([]bool, len(events)),
	}
}

// run owns both channels (the rdk contract): it closes batches to end the stream and
// closes responses only after the RPC has returned. On a feed error it cancels the ctx
// instead, because the module parks on <-batches holding its move lock and only the ctx
// releases it.
func (p *producer) run(ctx context.Context) error {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	batches := make(chan []arm.TrajectoryPoint)
	responses := make(chan arm.Response)
	rpcDone := make(chan error, 1)
	go func() {
		err := p.arm.MoveThroughJointPositionsStreamed(ctx, batches, responses, nil)
		cancel() // a feed blocked on a send nobody reads any more must return
		rpcDone <- err
	}()
	drained := make(chan struct{})
	go func() {
		defer close(drained)
		for range responses {
		}
	}()

	feedErr := p.feed(ctx, batches)
	if feedErr == nil {
		close(batches)
	} else {
		cancel()
	}
	rpcErr := <-rpcDone
	close(responses)
	<-drained
	if rpcErr != nil && (feedErr == nil || errors.Is(feedErr, context.Canceled)) {
		return fmt.Errorf("stream: %w", rpcErr)
	}
	return feedErr
}

// elapsed is the stream clock: time since the first send. mu must be held.
func (p *producer) elapsed() time.Duration {
	if p.t0.IsZero() {
		return 0
	}
	return time.Since(p.t0)
}

// feed runs the send tick. It returns nil once every point of the final trajectory is out
// and nothing is pending or left to fire.
func (p *producer) feed(ctx context.Context, batches chan<- []arm.TrajectoryPoint) error {
	tick := time.NewTicker(p.sendEvery)
	defer tick.Stop()
	for {
		p.mu.Lock()
		if p.planErr != nil {
			err := p.planErr
			p.mu.Unlock()
			return err
		}
		elapsed := p.elapsed()
		// At most one splice in flight: an event that comes due while one is pending is
		// held and lands on the first tick after, with its t_s computed against the new cur.
		if !p.pending {
			if due := streamx.Due(p.events, p.fired, elapsed); len(due) > 0 {
				p.startReplan(ctx, due, elapsed)
			}
		}
		// upTo is non-decreasing: at the event tick elapsed+runway = t_s-margin < t_s, so the
		// clamp binds only as elapsed grows, caps at t_s, and lifts when the splice lands.
		// Strict-< in Window means no point at or past t_s is sent and then dropped by Splice.
		upTo := elapsed + p.runway
		if p.pending {
			upTo = min(upTo, p.tStitch)
		}
		w := streamx.Window(p.cur, p.sent, upTo)
		p.sent += len(w)
		first := p.t0.IsZero() && len(w) > 0
		if first {
			p.t0 = time.Now()
		}
		done := p.sent == len(p.cur) && !p.pending && !p.hasUnfired()
		p.mu.Unlock()

		if len(w) > 0 {
			select {
			case batches <- w:
			case <-ctx.Done():
				return ctx.Err()
			}
		}
		if done {
			return nil
		}
		select {
		case <-tick.C:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// hasUnfired reports whether an event is still to come. mu must be held. An event whose
// t_e lies past the end of the (spliced) trajectory still fires once elapsed reaches it:
// the arm rests at the end, the splice lands past it, and At clamps to the last sample.
func (p *producer) hasUnfired() bool {
	for _, f := range p.fired {
		if !f {
			return true
		}
	}
	return false
}

// startReplan fixes the stitch time and starts planning from where the arm WILL be then.
// mu must be held; the goroutine re-takes it to apply the splice.
func (p *producer) startReplan(ctx context.Context, due []int, elapsed time.Duration) {
	tStitch := elapsed + p.runway + p.planMargin
	qs := streamx.At(p.cur, tStitch)
	events := make([]streamx.Event, len(due))
	for i, idx := range due {
		events[i] = p.events[idx]
	}
	p.pending, p.tStitch = true, tStitch
	go func() {
		wps, pts, latency, err := p.replan(ctx, qs, events)
		p.mu.Lock()
		defer p.mu.Unlock()
		p.pending = false
		if err != nil {
			p.planErr = err
			return
		}
		// Sent points are never changed: every one has Time < tStitch (the window clamp),
		// so p.sent still indexes into the kept prefix.
		cur, err := streamx.Splice(p.cur, pts, tStitch)
		if err != nil {
			p.planErr = err
			return
		}
		p.cur = cur
		p.splices = append(p.splices, splice{
			events: events, tStitch: tStitch, qs: qs, planLatency: latency,
			stitchLate: max(0, p.elapsed()-tStitch), waypoints: wps, newPts: pts,
		})
	}()
}

// result is the final trajectory and the splices applied, for the report. Call after run.
func (p *producer) result() ([]arm.TrajectoryPoint, []splice) {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.cur, p.splices
}
