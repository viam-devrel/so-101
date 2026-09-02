package servo

// Gains is one servo's position-loop PID register triple. Each value is a single byte.
type Gains struct {
	P int
	D int
	I int
}

// DefaultArmGains is the tuned per-joint gain table for arm servos 1-5, against a stock
// 32/32/0. A tuning value, not a hardware constant -- measured on ONE arm at 12.2-12.3 V.
// See docs/arm.md, "Servo gains", for the measurements, the stability ceiling, and the
// peak-current cost.
//
// Servo 6 (gripper) is deliberately absent: never characterized, and its lower stock P=16 is
// likely a deliberate soft-grasp choice.
var DefaultArmGains = map[int]Gains{
	1: {P: 32, D: 48, I: 0}, // shoulder pan  -- gravity-free; D 32->48 nearly halves overshoot
	2: {P: 48, D: 64, I: 2}, // shoulder lift -- gravity-loaded; I halves the residual droop
	3: {P: 48, D: 64, I: 4}, // elbow         -- tolerates I=4 where the shoulder does not
	4: {P: 48, D: 64, I: 2}, // wrist flex
	5: {P: 48, D: 48, I: 0}, // wrist roll    -- gravity-free, needs no integral term
}
