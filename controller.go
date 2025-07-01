package arm

import (
	"errors"
	"fmt"
	"math"
	"sync"
	"time"

	"go.bug.st/serial"
	"go.viam.com/rdk/logging"
)

// SO101JointCalibration represents calibration data for a single joint
type SO101JointCalibration struct {
	ID           int `json:"id"`
	DriveMode    int `json:"drive_mode"`
	HomingOffset int `json:"homing_offset"`
	RangeMin     int `json:"range_min"`
	RangeMax     int `json:"range_max"`
}

// SO101Calibration holds calibration data for all joints
type SO101Calibration struct {
	ShoulderPan  SO101JointCalibration `json:"shoulder_pan"`
	ShoulderLift SO101JointCalibration `json:"shoulder_lift"`
	ElbowFlex    SO101JointCalibration `json:"elbow_flex"`
	WristFlex    SO101JointCalibration `json:"wrist_flex"`
	WristRoll    SO101JointCalibration `json:"wrist_roll"`
	Gripper      SO101JointCalibration `json:"gripper"`
}

// Your improved calibration data from LeRobot
var DefaultSO101Calibration = SO101Calibration{
	ShoulderPan: SO101JointCalibration{
		ID: 1, DriveMode: 0, HomingOffset: 631,
		RangeMin: 1156, RangeMax: 2976,
	},
	ShoulderLift: SO101JointCalibration{
		ID: 2, DriveMode: 0, HomingOffset: -268,
		RangeMin: 848, RangeMax: 3206,
	},
	ElbowFlex: SO101JointCalibration{
		ID: 3, DriveMode: 0, HomingOffset: -713,
		RangeMin: 976, RangeMax: 3205,
	},
	WristFlex: SO101JointCalibration{
		ID: 4, DriveMode: 0, HomingOffset: -515,
		RangeMin: 711, RangeMax: 2688,
	},
	WristRoll: SO101JointCalibration{
		ID: 5, DriveMode: 0, HomingOffset: -230,
		RangeMin: 369, RangeMax: 3646,
	},
	Gripper: SO101JointCalibration{
		ID: 6, DriveMode: 0, HomingOffset: -1479,
		RangeMin: 2046, RangeMax: 3403,
	},
}

// SoArmController handles communication with SO-ARM servo motors using Feetech protocol
type SoArmController struct {
	port        serial.Port
	servoIDs    []int
	logger      logging.Logger
	mu          sync.RWMutex
	timeout     time.Duration
	calibration SO101Calibration
}

// Feetech servo command constants
const (
	// Packet structure
	PKT_HEADER1     = 0xFF
	PKT_HEADER2     = 0xFF
	PKT_ID          = 2
	PKT_LENGTH      = 3
	PKT_INSTRUCTION = 4
	PKT_ERROR       = 4
	PKT_PARAMETER0  = 5

	// Instructions
	INST_PING       = 0x01
	INST_READ       = 0x02
	INST_WRITE      = 0x03
	INST_REG_WRITE  = 0x04
	INST_ACTION     = 0x05
	INST_RESET      = 0x06
	INST_SYNC_WRITE = 0x83

	// Control table addresses (from STS_SMS_SERIES_CONTROL_TABLE)
	ADDR_MODEL_NUMBER     = 3
	ADDR_ID               = 5
	ADDR_TORQUE_ENABLE    = 40
	ADDR_ACCELERATION     = 41
	ADDR_GOAL_POSITION    = 42
	ADDR_GOAL_TIME        = 44
	ADDR_GOAL_VELOCITY    = 46
	ADDR_PRESENT_POSITION = 56
	ADDR_PRESENT_VELOCITY = 58
	ADDR_PRESENT_LOAD     = 60
	ADDR_MOVING           = 66

	// Communication results
	COMM_SUCCESS    = 0
	COMM_PORT_BUSY  = -1000
	COMM_TX_FAIL    = -1001
	COMM_RX_FAIL    = -1002
	COMM_TX_ERROR   = -2000
	COMM_RX_WAITING = -3000
	COMM_RX_TIMEOUT = -3001
	COMM_RX_CORRUPT = -3002

	// Special IDs
	BROADCAST_ID = 0xFE

	// Position range for SO-101 (based on your reading of 0x500A = 20490)
	// SO-101 appears to use 16-bit position values
	SERVO_CENTER_POSITION = 32768 // Center position
	SERVO_MIN_POSITION    = 0     // Minimum position
	SERVO_MAX_POSITION    = 65535 // Maximum position
)

// NewSoArmController creates a new controller for SO-ARM servos
func NewSoArmController(portName string, baudrate int, servoIDs []int, logger logging.Logger) (*SoArmController, error) {
	if logger == nil {
		return nil, errors.New("logger cannot be nil")
	}

	// Open serial port
	mode := &serial.Mode{
		BaudRate: baudrate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open serial port %s: %w", portName, err)
	}

	controller := &SoArmController{
		port:        port,
		servoIDs:    servoIDs,
		logger:      logger,
		timeout:     time.Second * 1,
		calibration: DefaultSO101Calibration, // Use the calibration data
	}

	// Skip ping test since we know movement commands work
	// Test communication will happen during actual operations
	logger.Warnf("Skipping ping test - will verify communication during operations")

	logger.Infof("SO-ARM controller initialized on %s at %d baud with servo IDs: %v", portName, baudrate, servoIDs)
	return controller, nil
}

// Ping tests communication with all servos
func (c *SoArmController) Ping() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	for _, id := range c.servoIDs {
		if err := c.sendPing(id); err != nil {
			return fmt.Errorf("ping failed for servo %d: %w", id, err)
		}
	}
	return nil
}

// sendPing sends a ping command to a specific servo
func (c *SoArmController) sendPing(servoID int) error {
	// Build ping packet: [0xFF][0xFF][ID][Length][Instruction][Checksum]
	packet := []byte{PKT_HEADER1, PKT_HEADER2, byte(servoID), 0x02, INST_PING}

	// Calculate checksum (exclude headers, include ID, length, instruction)
	checksum := byte(servoID) + 0x02 + INST_PING
	checksum = ^checksum // Bitwise NOT
	packet = append(packet, checksum)

	if err := c.writePacket(packet); err != nil {
		return err
	}

	// Read response with more lenient timeout
	response, err := c.readResponseLenient()
	if err != nil {
		return err
	}

	if len(response) < 6 || response[PKT_ID] != byte(servoID) {
		return fmt.Errorf("invalid ping response from servo %d: got %X", servoID, response)
	}

	return nil
}

// SetTorqueEnable enables or disables torque for all servos
func (c *SoArmController) SetTorqueEnable(enable bool) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	value := 0
	if enable {
		value = 1
	}

	for _, id := range c.servoIDs {
		if err := c.writeRegister(id, ADDR_TORQUE_ENABLE, 1, []int{value}); err != nil {
			return fmt.Errorf("failed to set torque enable for servo %d: %w", id, err)
		}
	}

	status := "disabled"
	if enable {
		status = "enabled"
	}
	c.logger.Infof("Torque %s for all servos", status)
	return nil
}

// MoveToJointPositions moves all joints to specified angles using calibration
func (c *SoArmController) MoveToJointPositions(jointAngles []float64, speed, acceleration int) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if len(jointAngles) != len(c.servoIDs) {
		return fmt.Errorf("expected %d joint angles, got %d", len(c.servoIDs), len(jointAngles))
	}

	// Convert radians to servo positions using calibration
	positions := make([]int, len(jointAngles))
	for i, angle := range jointAngles {
		positions[i] = c.radiansToServoPositionCalibrated(angle, i)
		c.logger.Debugf("Servo %d: %.3f rad -> position %d", c.servoIDs[i], angle, positions[i])
	}

	// Use sync write to move all servos simultaneously
	return c.syncWritePositions(positions)
}

// GetJointPositions reads current positions of all joints using calibration
func (c *SoArmController) GetJointPositions() ([]float64, error) {
	c.mu.RLock()
	defer c.mu.RUnlock()

	angles := make([]float64, len(c.servoIDs))
	for i, id := range c.servoIDs {
		position, err := c.readCurrentPosition(id)
		if err != nil {
			return nil, fmt.Errorf("failed to read position from servo %d: %w", id, err)
		}
		angles[i] = c.servoPositionToRadiansCalibrated(position, i)
	}

	return angles, nil
}

// Stop stops all servo movement by setting goal velocity to 0
func (c *SoArmController) Stop() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	for _, id := range c.servoIDs {
		if err := c.writeRegister(id, ADDR_GOAL_VELOCITY, 2, []int{0, 0}); err != nil {
			return fmt.Errorf("failed to stop servo %d: %w", id, err)
		}
	}
	return nil
}

// Close closes the serial connection
func (c *SoArmController) Close() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if c.port != nil {
		c.logger.Info("Closing SO-ARM controller")
		return c.port.Close()
	}
	return nil
}

// Helper methods

// radiansToServoPositionCalibrated converts radians to servo position using calibration
func (c *SoArmController) radiansToServoPositionCalibrated(radians float64, jointIndex int) int {
	// Get calibration for this joint
	calibrations := []SO101JointCalibration{
		c.calibration.ShoulderPan,
		c.calibration.ShoulderLift,
		c.calibration.ElbowFlex,
		c.calibration.WristFlex,
		c.calibration.WristRoll,
	}

	if jointIndex >= len(calibrations) {
		// Fall back to uncalibrated conversion
		return c.radiansToServoPosition(radians)
	}

	cal := calibrations[jointIndex]

	// Convert radians to normalized range (-1 to 1)
	// Assuming ±π radians maps to full joint range
	normalizedPos := radians / math.Pi

	// Clamp to reasonable range
	if normalizedPos > 1.0 {
		normalizedPos = 1.0
	} else if normalizedPos < -1.0 {
		normalizedPos = -1.0
	}

	// Map to calibrated servo range
	center := (cal.RangeMin + cal.RangeMax) / 2
	halfRange := (cal.RangeMax - cal.RangeMin) / 2

	position := center + int(normalizedPos*float64(halfRange))

	// Apply homing offset
	position += cal.HomingOffset

	// Clamp to safe range
	if position < cal.RangeMin {
		c.logger.Warnf("Joint %d position %d below min %d, clamping", jointIndex, position, cal.RangeMin)
		position = cal.RangeMin
	} else if position > cal.RangeMax {
		c.logger.Warnf("Joint %d position %d above max %d, clamping", jointIndex, position, cal.RangeMax)
		position = cal.RangeMax
	}

	c.logger.Debugf("Joint %d: %.3f rad -> position %d (range: %d-%d, offset: %d)",
		jointIndex, radians, position, cal.RangeMin, cal.RangeMax, cal.HomingOffset)

	return position
}

// servoPositionToRadiansCalibrated converts servo position to radians using calibration
func (c *SoArmController) servoPositionToRadiansCalibrated(position int, jointIndex int) float64 {
	// Get calibration for this joint
	calibrations := []SO101JointCalibration{
		c.calibration.ShoulderPan,
		c.calibration.ShoulderLift,
		c.calibration.ElbowFlex,
		c.calibration.WristFlex,
		c.calibration.WristRoll,
	}

	if jointIndex >= len(calibrations) {
		// Fall back to uncalibrated conversion
		return c.servoPositionToRadians(position)
	}

	cal := calibrations[jointIndex]

	// Remove homing offset
	adjustedPos := position - cal.HomingOffset

	// Convert from servo range to normalized range (-1 to 1)
	center := (cal.RangeMin + cal.RangeMax) / 2
	halfRange := (cal.RangeMax - cal.RangeMin) / 2

	normalizedPos := float64(adjustedPos-center) / float64(halfRange)

	// Convert to radians (±π range)
	return normalizedPos * math.Pi
}

// radiansToServoPosition converts radians to servo position (0-65535 range)
func (c *SoArmController) radiansToServoPosition(radians float64) int {
	// Convert radians to degrees
	degrees := radians * 180.0 / math.Pi

	// Map -180° to +180° to range 0-65535, center at 32768
	position := int(SERVO_CENTER_POSITION + (degrees * float64(SERVO_MAX_POSITION) / 360.0))

	// Clamp to valid range
	if position < SERVO_MIN_POSITION {
		position = SERVO_MIN_POSITION
	} else if position > SERVO_MAX_POSITION {
		position = SERVO_MAX_POSITION
	}

	return position
}

// servoPositionToRadians converts servo position to radians
func (c *SoArmController) servoPositionToRadians(position int) float64 {
	// Convert from 16-bit range back to degrees, then radians
	degrees := (float64(position) - float64(SERVO_CENTER_POSITION)) * 360.0 / float64(SERVO_MAX_POSITION)
	return degrees * math.Pi / 180.0
}

// syncWritePositions moves multiple servos simultaneously using sync write
func (c *SoArmController) syncWritePositions(positions []int) error {
	if len(positions) != len(c.servoIDs) {
		return fmt.Errorf("position count mismatch: expected %d, got %d", len(c.servoIDs), len(positions))
	}

	// Build sync write packet for Goal_Position (address 42, 2 bytes per servo)
	dataLen := 2                                // 2 bytes per position
	paramLen := len(c.servoIDs) * (1 + dataLen) // ID + data for each servo

	packet := []byte{
		PKT_HEADER1, PKT_HEADER2,
		BROADCAST_ID,       // Broadcast ID for sync write
		byte(4 + paramLen), // Length: instruction + addr + len + params + checksum
		INST_SYNC_WRITE,    // Sync write instruction
		ADDR_GOAL_POSITION, // Starting address
		byte(dataLen),      // Data length per servo
	}

	// Add parameters for each servo
	for i, id := range c.servoIDs {
		position := positions[i]
		packet = append(packet, byte(id))                 // Servo ID
		packet = append(packet, byte(position&0xFF))      // Position low byte
		packet = append(packet, byte((position>>8)&0xFF)) // Position high byte
	}

	// Calculate and append checksum
	checksum := c.calculateChecksum(packet[2:]) // Skip headers
	packet = append(packet, checksum)

	return c.writePacket(packet)
}

// readCurrentPosition reads the current position of a servo
func (c *SoArmController) readCurrentPosition(servoID int) (int, error) {
	// Build read packet: [0xFF][0xFF][ID][Length][Instruction][Address][Data_Length][Checksum]
	packet := []byte{
		PKT_HEADER1, PKT_HEADER2,
		byte(servoID),
		0x04,                  // Length
		INST_READ,             // Read instruction
		ADDR_PRESENT_POSITION, // Address
		0x02,                  // Read 2 bytes
	}

	checksum := c.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	if err := c.writePacket(packet); err != nil {
		return 0, err
	}

	// Read response: [0xFF][0xFF][ID][Length][Error][Data1][Data2][Checksum]
	response, err := c.readResponse(8) // Expected response length
	if err != nil {
		return 0, err
	}

	if len(response) < 8 || response[PKT_ID] != byte(servoID) {
		return 0, fmt.Errorf("invalid position response from servo %d", servoID)
	}

	// Extract position from response (little endian)
	position := int(response[5]) | (int(response[6]) << 8)
	return position, nil
}

// writeRegister writes data to a servo register
func (c *SoArmController) writeRegister(servoID int, address byte, length int, data []int) error {
	// Build write packet
	packet := []byte{
		PKT_HEADER1, PKT_HEADER2,
		byte(servoID),
		byte(3 + length), // Length: instruction + address + data + checksum
		INST_WRITE,       // Write instruction
		address,          // Register address
	}

	// Add data bytes
	for _, value := range data {
		packet = append(packet, byte(value&0xFF))
		if length > 1 {
			packet = append(packet, byte((value>>8)&0xFF))
		}
	}

	checksum := c.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	return c.writePacket(packet)
}

// calculateChecksum calculates Feetech protocol checksum
func (c *SoArmController) calculateChecksum(packet []byte) byte {
	checksum := byte(0)
	for _, b := range packet {
		checksum += b
	}
	return ^checksum // Bitwise NOT
}

// readResponseLenient reads response with more flexible error handling
func (c *SoArmController) readResponseLenient() ([]byte, error) {
	// Set read timeout
	if err := c.port.SetReadTimeout(200 * time.Millisecond); err != nil {
		return nil, fmt.Errorf("failed to set read timeout: %w", err)
	}

	// Read up to 20 bytes to capture any response
	buffer := make([]byte, 20)
	n, err := c.port.Read(buffer)
	if err != nil {
		return nil, fmt.Errorf("failed to read response: %w", err)
	}

	if n == 0 {
		return nil, errors.New("no response received")
	}

	response := buffer[:n]
	c.logger.Debugf("Raw response: %X", response)

	// Just return the response without strict validation for debugging
	return response, nil
}

// writePacket writes a packet to the serial port
func (c *SoArmController) writePacket(packet []byte) error {
	// Clear input buffer
	c.port.ResetInputBuffer()

	n, err := c.port.Write(packet)
	if err != nil {
		return fmt.Errorf("failed to write packet: %w", err)
	}
	if n != len(packet) {
		return fmt.Errorf("incomplete packet write: wrote %d of %d bytes", n, len(packet))
	}

	return nil
}

// readResponse reads a response packet from the serial port
func (c *SoArmController) readResponse(expectedLen int) ([]byte, error) {
	// Set read timeout
	if err := c.port.SetReadTimeout(c.timeout); err != nil {
		return nil, fmt.Errorf("failed to set read timeout: %w", err)
	}

	buffer := make([]byte, expectedLen)
	totalRead := 0

	for totalRead < expectedLen {
		n, err := c.port.Read(buffer[totalRead:])
		if err != nil {
			return nil, fmt.Errorf("failed to read response: %w", err)
		}
		totalRead += n

		// Check for timeout or partial read
		if n == 0 {
			break
		}
	}

	response := buffer[:totalRead]

	// Verify packet structure
	if len(response) < 4 {
		return nil, errors.New("response too short")
	}

	if response[0] != PKT_HEADER1 || response[1] != PKT_HEADER2 {
		return nil, errors.New("invalid packet header")
	}

	// Verify checksum
	if !c.verifyChecksum(response) {
		return nil, errors.New("checksum verification failed")
	}

	return response, nil
}

// verifyChecksum verifies the packet checksum
func (c *SoArmController) verifyChecksum(packet []byte) bool {
	if len(packet) < 4 {
		return false
	}

	// Calculate expected checksum (exclude headers and final checksum byte)
	checksum := c.calculateChecksum(packet[2 : len(packet)-1])

	return checksum == packet[len(packet)-1]
}
