package servo

import (
	"math"
	"testing"
)

func TestDegPerSecToStepsPerSec(t *testing.T) {
	tests := []struct {
		name string
		degs float64
		want int
	}{
		{"50 deg/s", 50, 569}, // round(50 * 4096/360)
		{"3 deg/s min config", 3, 34},
		{"180 deg/s max config", 180, 2048},
		{"clamps to floor", 0, 1},
		{"clamps to ceiling", 100000, 3000},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if got := DegPerSecToStepsPerSec(tt.degs); got != tt.want {
				t.Errorf("DegPerSecToStepsPerSec(%v) = %d, want %d", tt.degs, got, tt.want)
			}
		})
	}
}

func TestResolveSpeedDegsPerSec(t *testing.T) {
	const def = 50.0
	tests := []struct {
		name       string
		maxVelRads float64
		want       float64
	}{
		{"zero falls back to default", 0, def},
		{"negative falls back to default", -1, def},
		{"converts rad/s to deg/s", math.Pi / 2, 90}, // 1.5708 rad/s -> 90 deg/s
		{"clamps below min to 3", 0.001, 3},
		{"clamps above max to 180", 100, 180},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got := ResolveSpeedDegsPerSec(tt.maxVelRads, def)
			if math.Abs(got-tt.want) > 1e-6 {
				t.Errorf("ResolveSpeedDegsPerSec(%v, %v) = %v, want %v", tt.maxVelRads, def, got, tt.want)
			}
		})
	}
}

func TestMoveTimeoutMs(t *testing.T) {
	tests := []struct {
		name   string
		travel float64
		speed  float64
		accel  float64
		want   int
	}{
		// Trapezoidal: (d/v + v/a) * 2.0 factor.
		{"normal move", 90, 45, 500, 4180},
		// Triangular, and the case that motivated modelling the ramp at all: with the old
		// d/v formula this computed 1200ms against a true duration of 1549ms, so
		// WaitForServosToStop returned while the arm was still moving.
		{"triangular regime aborts early under d/v", 30, 50, 50, 3098},
		{"clamps to floor", 1, 180, 500, 1000},
		{"clamps to ceiling", 1000, 1, 500, 15000},
		{"zero speed -> ceiling", 90, 0, 500, 15000},
		{"zero acceleration -> ceiling", 90, 45, 0, 15000},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if got := MoveTimeoutMs(tt.travel, tt.speed, tt.accel); got != tt.want {
				t.Errorf("MoveTimeoutMs(%v, %v, %v) = %d, want %d",
					tt.travel, tt.speed, tt.accel, got, tt.want)
			}
		})
	}
}
