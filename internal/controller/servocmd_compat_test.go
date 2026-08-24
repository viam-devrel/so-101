package controller

import (
	"so_arm/internal/servocmd"
)

// The real controller must satisfy servocmd.ServoOps, or the arm cannot serve the protocol that
// every test in internal/servocmd exercises against fakes. This assertion is the seam between the
// tested dispatch logic and the hardware path, which needs a serial port and so cannot be unit
// tested directly.
var _ servocmd.ServoOps = (*ControllerHandle)(nil)
