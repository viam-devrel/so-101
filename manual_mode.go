package so_arm

import "math"

// manualParams are the loop tuning constants (already resolved from config+defaults).
type manualParams struct {
	deadband  float64 // load excess (above bias) required before a joint follows the hand
	gain      float64 // radians per (load-unit * second)
	biasAlpha float64 // 0..1 low-pass factor for at-rest bias tracking
	dt        float64 // seconds per tick (1/loopHz)
}

// jointState is the per-joint state carried between ticks.
type jointState struct {
	bias float64 // filtered steady holding load at rest
	goal float64 // commanded position, radians
}

// stepResult is the outcome of one control step for one joint.
type stepResult struct {
	bias  float64
	goal  float64
	moved bool // true if goal changed this tick (caller should write it)
}

// manualStep runs one admittance step for a single joint.
// load is the present load (signed). limit is [min,max] radians.
func manualStep(st jointState, load float64, limit [2]float64, p manualParams) stepResult {
	excess := load - st.bias
	if math.Abs(excess) < p.deadband {
		// At rest: hold goal, let bias track toward the current load.
		newBias := st.bias + p.biasAlpha*(load-st.bias)
		return stepResult{bias: newBias, goal: st.goal, moved: false}
	}
	// Pushing: move goal past the deadband edge, freeze bias.
	drive := excess - math.Copysign(p.deadband, excess)
	newGoal := st.goal + p.gain*drive*p.dt
	newGoal = math.Max(limit[0], math.Min(limit[1], newGoal))
	moved := newGoal != st.goal
	return stepResult{bias: st.bias, goal: newGoal, moved: moved}
}
