package so_arm

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
}

// Default calibration data for SO-101
var DefaultSO101Calibration = SO101Calibration{
	ShoulderPan: SO101JointCalibration{
		ID: 1, DriveMode: 0, HomingOffset: 2048,
		RangeMin: 0, RangeMax: 4095,
	},
	ShoulderLift: SO101JointCalibration{
		ID: 2, DriveMode: 0, HomingOffset: 2048,
		RangeMin: 0, RangeMax: 4095,
	},
	ElbowFlex: SO101JointCalibration{
		ID: 3, DriveMode: 0, HomingOffset: 2048,
		RangeMin: 0, RangeMax: 4095,
	},
	WristFlex: SO101JointCalibration{
		ID: 4, DriveMode: 0, HomingOffset: 2048,
		RangeMin: 0, RangeMax: 4095,
	},
	WristRoll: SO101JointCalibration{
		ID: 5, DriveMode: 0, HomingOffset: 2048,
		RangeMin: 0, RangeMax: 4095,
	},
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

	// Control table addresses
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

	// Special IDs
	BROADCAST_ID = 0xFE

	// Position range for servos
	SERVO_CENTER_POSITION = 2048
	SERVO_MIN_POSITION    = 0
	SERVO_MAX_POSITION    = 4095
)

// Improved controller with better concurrent access handling and calibration support
type SoArmController struct {
	port        serial.Port
	servoIDs    []int
	logger      logging.Logger
	mu          sync.RWMutex
	timeout     time.Duration
	calibration SO101Calibration

	// Serial communication management
	serialMu        sync.Mutex    // Separate mutex for serial operations
	lastCommandTime time.Time     // Track timing between commands
	minCommandGap   time.Duration // Minimum gap between commands
}

// Enhanced controller creation with calibration support
func NewSoArmController(portName string, baudrate int, servoIDs []int, calibration SO101Calibration, logger logging.Logger) (*SoArmController, error) {
	if logger == nil {
		return nil, errors.New("logger cannot be nil")
	}

	// Open serial port with improved configuration
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
		port:            port,
		servoIDs:        servoIDs,
		logger:          logger,
		timeout:         time.Second * 1,
		calibration:     calibration,
		minCommandGap:   time.Millisecond * 5, // Minimum 5ms between commands
		lastCommandTime: time.Now(),
	}

	// Clear any existing data in buffers
	controller.clearSerialBuffers()

	logger.Infof("SO-ARM controller initialized on %s at %d baud with servo IDs: %v", portName, baudrate, servoIDs)
	return controller, nil
}

// SetCalibration updates the controller's calibration and validates it
func (c *SoArmController) SetCalibration(calibration SO101Calibration) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	// Validate the new calibration
	if err := ValidateCalibration(calibration, c.logger); err != nil {
		return fmt.Errorf("invalid calibration: %w", err)
	}

	c.calibration = calibration
	c.logger.Info("Controller calibration updated successfully")
	return nil
}

// GetCalibration returns a copy of the current calibration
func (c *SoArmController) GetCalibration() SO101Calibration {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.calibration
}

// clearSerialBuffers clears both input and output buffers
func (c *SoArmController) clearSerialBuffers() {
	if c.port == nil {
		return
	}

	// Clear input buffer
	c.port.ResetInputBuffer()

	// Read any remaining data with a short timeout
	c.port.SetReadTimeout(10 * time.Millisecond)
	buffer := make([]byte, 256)
	for {
		n, err := c.port.Read(buffer)
		if err != nil || n == 0 {
			break
		}
		c.logger.Debugf("Cleared %d bytes from input buffer", n)
	}

	// Reset timeout
	c.port.SetReadTimeout(c.timeout)
}

// enforceCommandGap ensures minimum time between serial commands
func (c *SoArmController) enforceCommandGap() {
	elapsed := time.Since(c.lastCommandTime)
	if elapsed < c.minCommandGap {
		time.Sleep(c.minCommandGap - elapsed)
	}
	c.lastCommandTime = time.Now()
}

// Improved GetJointPositions with better error handling and recovery
func (c *SoArmController) GetJointPositions() ([]float64, error) {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	angles := make([]float64, len(c.servoIDs))
	maxRetries := 3

	for i, id := range c.servoIDs {
		var position int
		var err error

		// Retry logic for each servo
		for attempt := 0; attempt < maxRetries; attempt++ {
			if attempt > 0 {
				c.logger.Debugf("Retrying read for servo %d, attempt %d", id, attempt+1)
				// Clear buffers before retry
				c.clearSerialBuffers()
				// Wait longer between retries
				time.Sleep(time.Duration(attempt*10) * time.Millisecond)
			}

			c.enforceCommandGap()
			position, err = c.readCurrentPositionRobust(id)
			if err == nil {
				break
			}

			c.logger.Warnf("Failed to read position from servo %d on attempt %d: %v", id, attempt+1, err)
		}

		if err != nil {
			return nil, fmt.Errorf("failed to read position from servo %d after %d attempts: %w", id, maxRetries, err)
		}

		angles[i] = c.servoPositionToRadiansCalibrated(position, i)
	}

	return angles, nil
}

// readCurrentPositionRobust with improved error detection and recovery
func (c *SoArmController) readCurrentPositionRobust(servoID int) (int, error) {
	// Build read packet
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

	// Clear buffers before sending
	c.clearSerialBuffers()

	// Send packet with verification
	if err := c.writePacketVerified(packet); err != nil {
		return 0, fmt.Errorf("failed to send packet: %w", err)
	}

	// Read response with improved validation
	response, err := c.readResponseRobust(8) // Expected response length
	if err != nil {
		return 0, err
	}

	// Validate response structure
	if len(response) < 8 {
		return 0, fmt.Errorf("response too short: got %d bytes, expected 8", len(response))
	}

	// Check packet headers
	if response[0] != PKT_HEADER1 || response[1] != PKT_HEADER2 {
		return 0, fmt.Errorf("invalid packet header: got [0x%02X, 0x%02X], expected [0xFF, 0xFF]", response[0], response[1])
	}

	// Check servo ID
	if response[PKT_ID] != byte(servoID) {
		return 0, fmt.Errorf("invalid position response from servo %d: got ID %d", servoID, response[PKT_ID])
	}

	// Verify checksum
	if !c.verifyChecksum(response) {
		return 0, fmt.Errorf("checksum verification failed")
	}

	// Extract position from response (little endian)
	if len(response) < 7 {
		return 0, fmt.Errorf("response too short for position data")
	}

	position := int(response[5]) | (int(response[6]) << 8)
	return position, nil
}

// writePacketVerified writes packet and verifies it was sent completely
func (c *SoArmController) writePacketVerified(packet []byte) error {
	// Clear input buffer before writing
	c.port.ResetInputBuffer()

	n, err := c.port.Write(packet)
	if err != nil {
		return fmt.Errorf("failed to write packet: %w", err)
	}
	if n != len(packet) {
		return fmt.Errorf("incomplete packet write: wrote %d of %d bytes", n, len(packet))
	}

	// Small delay to allow transmission to complete
	time.Sleep(time.Millisecond * 2)

	return nil
}

// readResponseRobust with improved error handling and timeout management
func (c *SoArmController) readResponseRobust(expectedLen int) ([]byte, error) {
	// Set read timeout
	if err := c.port.SetReadTimeout(c.timeout); err != nil {
		return nil, fmt.Errorf("failed to set read timeout: %w", err)
	}

	buffer := make([]byte, expectedLen*2) // Allow for extra data
	totalRead := 0
	startTime := time.Now()

	// Read with timeout and partial read handling
	for totalRead < expectedLen {
		if time.Since(startTime) > c.timeout {
			return nil, fmt.Errorf("timeout reading response after %v", c.timeout)
		}

		n, err := c.port.Read(buffer[totalRead:])
		if err != nil {
			return nil, fmt.Errorf("failed to read response: %w", err)
		}

		if n == 0 {
			// No data available, check if we've waited long enough
			if time.Since(startTime) > time.Millisecond*100 {
				break
			}
			time.Sleep(time.Millisecond * 5)
			continue
		}

		totalRead += n

		// Check if we have enough data for a valid packet
		if totalRead >= 6 {
			// Look for packet headers
			for i := 0; i <= totalRead-6; i++ {
				if buffer[i] == PKT_HEADER1 && buffer[i+1] == PKT_HEADER2 {
					// Found headers, check if we have complete packet
					if i+3 < totalRead {
						packetLength := int(buffer[i+3]) + 4 // Length + headers + ID + length
						if i+packetLength <= totalRead {
							// We have a complete packet
							response := make([]byte, packetLength)
							copy(response, buffer[i:i+packetLength])
							return response, nil
						}
					}
				}
			}
		}
	}

	if totalRead == 0 {
		return nil, errors.New("no response received")
	}

	// Return whatever we got
	response := make([]byte, totalRead)
	copy(response, buffer[:totalRead])
	return response, nil
}

// Improved MoveToJointPositions with better error handling
func (c *SoArmController) MoveToJointPositions(jointAngles []float64, speed, acceleration int) error {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

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
	return c.syncWritePositionsRobust(positions)
}

// syncWritePositionsRobust with improved error handling
func (c *SoArmController) syncWritePositionsRobust(positions []int) error {
	if len(positions) != len(c.servoIDs) {
		return fmt.Errorf("position count mismatch: expected %d, got %d", len(c.servoIDs), len(positions))
	}

	// Clear buffers before sync write
	c.clearSerialBuffers()
	c.enforceCommandGap()

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

	return c.writePacketVerified(packet)
}

// Rest of the methods remain the same but with improved error handling...
// (keeping the original methods but adding the serialMu.Lock() pattern)

func (c *SoArmController) SetTorqueEnable(enable bool) error {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	value := 0
	if enable {
		value = 1
	}

	for _, id := range c.servoIDs {
		c.enforceCommandGap()
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

func (c *SoArmController) Ping() error {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	for _, id := range c.servoIDs {
		c.enforceCommandGap()
		if err := c.sendPing(id); err != nil {
			return fmt.Errorf("ping failed for servo %d: %w", id, err)
		}
	}
	return nil
}

func (c *SoArmController) Stop() error {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	for _, id := range c.servoIDs {
		c.enforceCommandGap()
		if err := c.writeRegister(id, ADDR_GOAL_VELOCITY, 2, []int{0, 0}); err != nil {
			return fmt.Errorf("failed to stop servo %d: %w", id, err)
		}
	}
	return nil
}

func (c *SoArmController) Close() error {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	if c.port != nil {
		c.logger.Info("Closing SO-ARM controller")
		return c.port.Close()
	}
	return nil
}

// Helper methods (keeping original implementations but adding proper error handling)

func (c *SoArmController) sendPing(servoID int) error {
	packet := []byte{PKT_HEADER1, PKT_HEADER2, byte(servoID), 0x02, INST_PING}
	checksum := byte(servoID) + 0x02 + INST_PING
	checksum = ^checksum
	packet = append(packet, checksum)

	if err := c.writePacketVerified(packet); err != nil {
		return err
	}

	response, err := c.readResponseRobust(6)
	if err != nil {
		return err
	}

	if len(response) < 6 || response[PKT_ID] != byte(servoID) {
		return fmt.Errorf("invalid ping response from servo %d: got %X", servoID, response)
	}

	return nil
}

// Updated calibration methods to use the configurable calibration

func (c *SoArmController) radiansToServoPositionCalibrated(radians float64, jointIndex int) int {
	calibrations := []SO101JointCalibration{
		c.calibration.ShoulderPan,
		c.calibration.ShoulderLift,
		c.calibration.ElbowFlex,
		c.calibration.WristFlex,
		c.calibration.WristRoll,
	}

	if jointIndex >= len(calibrations) {
		return c.radiansToServoPosition(radians)
	}

	cal := calibrations[jointIndex]

	// Apply drive mode (invert direction if needed)
	adjustedRadians := radians
	if cal.DriveMode != 0 {
		adjustedRadians = -radians
	}

	// Convert radians to normalized position (-1 to 1)
	normalizedPos := adjustedRadians / math.Pi
	if normalizedPos > 1.0 {
		normalizedPos = 1.0
	} else if normalizedPos < -1.0 {
		normalizedPos = -1.0
	}

	// Map to servo range
	center := (cal.RangeMin + cal.RangeMax) / 2
	halfRange := (cal.RangeMax - cal.RangeMin) / 2
	position := center + int(normalizedPos*float64(halfRange))

	// Apply homing offset
	position += cal.HomingOffset

	// Clamp to valid range
	if position < cal.RangeMin {
		c.logger.Warnf("Joint %d position %d below min %d, clamping", jointIndex, position, cal.RangeMin)
		position = cal.RangeMin
	} else if position > cal.RangeMax {
		c.logger.Warnf("Joint %d position %d above max %d, clamping", jointIndex, position, cal.RangeMax)
		position = cal.RangeMax
	}

	return position
}

func (c *SoArmController) servoPositionToRadiansCalibrated(position int, jointIndex int) float64 {
	calibrations := []SO101JointCalibration{
		c.calibration.ShoulderPan,
		c.calibration.ShoulderLift,
		c.calibration.ElbowFlex,
		c.calibration.WristFlex,
		c.calibration.WristRoll,
	}

	if jointIndex >= len(calibrations) {
		return c.servoPositionToRadians(position)
	}

	cal := calibrations[jointIndex]

	// Remove homing offset
	adjustedPos := position - cal.HomingOffset

	// Map from servo range to normalized position
	center := (cal.RangeMin + cal.RangeMax) / 2
	halfRange := (cal.RangeMax - cal.RangeMin) / 2
	normalizedPos := float64(adjustedPos-center) / float64(halfRange)

	// Convert to radians
	radians := normalizedPos * math.Pi

	// Apply drive mode (invert direction if needed)
	if cal.DriveMode != 0 {
		radians = -radians
	}

	return radians
}

func (c *SoArmController) radiansToServoPosition(radians float64) int {
	degrees := radians * 180.0 / math.Pi
	position := int(SERVO_CENTER_POSITION + (degrees * float64(SERVO_MAX_POSITION) / 360.0))
	if position < SERVO_MIN_POSITION {
		position = SERVO_MIN_POSITION
	} else if position > SERVO_MAX_POSITION {
		position = SERVO_MAX_POSITION
	}
	return position
}

func (c *SoArmController) servoPositionToRadians(position int) float64 {
	degrees := (float64(position) - float64(SERVO_CENTER_POSITION)) * 360.0 / float64(SERVO_MAX_POSITION)
	return degrees * math.Pi / 180.0
}

func (c *SoArmController) writeRegister(servoID int, address byte, length int, data []int) error {
	packet := []byte{
		PKT_HEADER1, PKT_HEADER2,
		byte(servoID),
		byte(3 + length),
		INST_WRITE,
		address,
	}

	for _, value := range data {
		packet = append(packet, byte(value&0xFF))
		if length > 1 {
			packet = append(packet, byte((value>>8)&0xFF))
		}
	}

	checksum := c.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	return c.writePacketVerified(packet)
}

func (c *SoArmController) calculateChecksum(packet []byte) byte {
	checksum := byte(0)
	for _, b := range packet {
		checksum += b
	}
	return ^checksum
}

func (c *SoArmController) verifyChecksum(packet []byte) bool {
	if len(packet) < 4 {
		return false
	}
	checksum := c.calculateChecksum(packet[2 : len(packet)-1])
	return checksum == packet[len(packet)-1]
}
