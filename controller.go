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

// SO101FullCalibration holds calibration data for all joints
type SO101FullCalibration struct {
	ShoulderPan  SO101JointCalibration `json:"shoulder_pan"`
	ShoulderLift SO101JointCalibration `json:"shoulder_lift"`
	ElbowFlex    SO101JointCalibration `json:"elbow_flex"`
	WristFlex    SO101JointCalibration `json:"wrist_flex"`
	WristRoll    SO101JointCalibration `json:"wrist_roll"`
	Gripper      SO101JointCalibration `json:"gripper"`
}

// Default calibration data for SO-101 (all 6 joints)
// Note: These are placeholder values - actual calibration should be loaded from file
var DefaultSO101FullCalibration = SO101FullCalibration{
	ShoulderPan: SO101JointCalibration{
		ID: 1, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
	},
	ShoulderLift: SO101JointCalibration{
		ID: 2, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
	},
	ElbowFlex: SO101JointCalibration{
		ID: 3, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
	},
	WristFlex: SO101JointCalibration{
		ID: 4, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
	},
	WristRoll: SO101JointCalibration{
		ID: 5, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
	},
	Gripper: SO101JointCalibration{
		ID: 6, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
	},
}

// Feetech servo command constants
const (
	PKT_HEADER1     = 0xFF
	PKT_HEADER2     = 0xFF
	PKT_ID          = 2
	PKT_LENGTH      = 3
	PKT_INSTRUCTION = 4
	PKT_ERROR       = 4
	PKT_PARAMETER0  = 5

	INST_PING       = 0x01
	INST_READ       = 0x02
	INST_WRITE      = 0x03
	INST_REG_WRITE  = 0x04
	INST_ACTION     = 0x05
	INST_RESET      = 0x06
	INST_SYNC_WRITE = 0x83

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

	BROADCAST_ID = 0xFE

	SERVO_CENTER_POSITION = 2048
	SERVO_MIN_POSITION    = 0
	SERVO_MAX_POSITION    = 4095
)

// Enhanced controller that handles all 6 servos
type SoArmController struct {
	port        serial.Port
	servoIDs    []int // All servo IDs this controller manages
	logger      logging.Logger
	mu          sync.RWMutex
	timeout     time.Duration
	calibration SO101FullCalibration

	// Serial communication management
	serialMu        sync.Mutex
	lastCommandTime time.Time
	minCommandGap   time.Duration
}

// NewSoArmController creates a controller that can handle all 6 servos
func NewSoArmController(portName string, baudrate int, servoIDs []int, calibration SO101FullCalibration, logger logging.Logger) (*SoArmController, error) {
	if logger == nil {
		return nil, errors.New("logger cannot be nil")
	}

	// Default to all 6 servos if none specified
	if len(servoIDs) == 0 {
		servoIDs = []int{1, 2, 3, 4, 5, 6}
	}

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
		minCommandGap:   time.Millisecond * 5,
		lastCommandTime: time.Now(),
	}

	controller.clearSerialBuffers()

	logger.Infof("SO-ARM controller initialized on %s at %d baud with servo IDs: %v", portName, baudrate, servoIDs)
	return controller, nil
}

// SetCalibration updates the controller's calibration
func (c *SoArmController) SetCalibration(calibration SO101FullCalibration) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if err := ValidateFullCalibration(calibration, c.logger); err != nil {
		return fmt.Errorf("invalid calibration: %w", err)
	}

	c.calibration = calibration
	c.logger.Info("Controller calibration updated successfully")
	return nil
}

// GetCalibration returns a copy of the current calibration
func (c *SoArmController) GetCalibration() SO101FullCalibration {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.calibration
}

// GetJointPositions returns positions for all configured servos in order
func (c *SoArmController) GetJointPositions() ([]float64, error) {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	angles := make([]float64, len(c.servoIDs))
	maxRetries := 3

	for i, id := range c.servoIDs {
		var position int
		var err error

		for attempt := 0; attempt < maxRetries; attempt++ {
			if attempt > 0 {
				c.logger.Debugf("Retrying read for servo %d, attempt %d", id, attempt+1)
				c.clearSerialBuffers()
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

		angles[i] = c.servoPositionToRadiansCalibrated(position, id)
	}

	return angles, nil
}

// GetJointPositionsForServos returns positions for specific servo IDs
func (c *SoArmController) GetJointPositionsForServos(requestedServoIDs []int) ([]float64, error) {
	allPositions, err := c.GetJointPositions()
	if err != nil {
		return nil, err
	}

	// Map requested servo IDs to positions
	positions := make([]float64, len(requestedServoIDs))
	for i, requestedID := range requestedServoIDs {
		// Find the index of this servo ID in our configured servos
		found := false
		for j, configuredID := range c.servoIDs {
			if configuredID == requestedID {
				positions[i] = allPositions[j]
				found = true
				break
			}
		}
		if !found {
			return nil, fmt.Errorf("servo ID %d not configured in controller", requestedID)
		}
	}

	return positions, nil
}

// MoveToJointPositions moves all configured servos to specified positions
func (c *SoArmController) MoveToJointPositions(jointAngles []float64, speed, acceleration int) error {
	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	if len(jointAngles) != len(c.servoIDs) {
		return fmt.Errorf("expected %d joint angles for configured servos, got %d", len(c.servoIDs), len(jointAngles))
	}

	positions := make([]int, len(jointAngles))
	for i, angle := range jointAngles {
		servoID := c.servoIDs[i]
		positions[i] = c.radiansToServoPositionCalibrated(angle, servoID)
		c.logger.Debugf("Servo %d: %.3f rad -> position %d", servoID, angle, positions[i])
	}

	return c.syncWritePositionsRobust(positions)
}

// MoveServosToPositions moves specific servos to specific positions
func (c *SoArmController) MoveServosToPositions(servoIDs []int, jointAngles []float64, speed, acceleration int) error {
	if len(servoIDs) != len(jointAngles) {
		return fmt.Errorf("servo IDs and joint angles length mismatch: %d vs %d", len(servoIDs), len(jointAngles))
	}

	c.serialMu.Lock()
	defer c.serialMu.Unlock()

	positions := make([]int, len(jointAngles))
	for i, angle := range jointAngles {
		servoID := servoIDs[i]
		positions[i] = c.radiansToServoPositionCalibrated(angle, servoID)
		c.logger.Debugf("Servo %d: %.3f rad -> position %d", servoID, angle, positions[i])
	}

	return c.syncWriteSpecificServos(servoIDs, positions)
}

// Helper method to get calibration for a specific servo ID
func (c *SoArmController) getCalibrationForServo(servoID int) SO101JointCalibration {
	cal := c.calibration
	switch servoID {
	case 1:
		return cal.ShoulderPan
	case 2:
		return cal.ShoulderLift
	case 3:
		return cal.ElbowFlex
	case 4:
		return cal.WristFlex
	case 5:
		return cal.WristRoll
	case 6:
		return cal.Gripper
	default:
		c.logger.Warnf("Unknown servo ID %d, using default calibration", servoID)
		return SO101JointCalibration{
			ID: servoID, DriveMode: 0, HomingOffset: 0,
			RangeMin: 500, RangeMax: 3500,
		}
	}
}

func (c *SoArmController) radiansToServoPositionCalibrated(radians float64, servoID int) int {
	cal := c.getCalibrationForServo(servoID)

	// Convert radians to degrees
	degrees := radians * 180.0 / math.Pi

	// Apply drive mode (invert direction if needed)
	if cal.DriveMode != 0 {
		degrees = -degrees
	}

	// Calculate mid point of calibrated range
	mid := float64(cal.RangeMin+cal.RangeMax) / 2

	// Convert degrees to servo position
	// Full servo range (4096 positions) represents 360 degrees
	position := int((degrees * 4095.0 / 360.0) + mid)

	// Clamp to calibrated range
	if position < cal.RangeMin {
		c.logger.Warnf("Servo %d position %d below min %d, clamping", servoID, position, cal.RangeMin)
		position = cal.RangeMin
	} else if position > cal.RangeMax {
		c.logger.Warnf("Servo %d position %d above max %d, clamping", servoID, position, cal.RangeMax)
		position = cal.RangeMax
	}

	return position
}

func (c *SoArmController) servoPositionToRadiansCalibrated(position int, servoID int) float64 {
	cal := c.getCalibrationForServo(servoID)

	// Calculate mid point of calibrated range
	mid := float64(cal.RangeMin+cal.RangeMax) / 2

	// Convert servo position to degrees
	// Full servo range (4096 positions) represents 360 degrees
	degrees := (float64(position) - mid) * 360.0 / 4095.0

	// Apply drive mode (invert direction if needed)
	if cal.DriveMode != 0 {
		degrees = -degrees
	}

	// Convert degrees to radians
	radians := degrees * math.Pi / 180.0

	return radians
}

// syncWriteSpecificServos writes positions to specific servo IDs
func (c *SoArmController) syncWriteSpecificServos(servoIDs []int, positions []int) error {
	if len(positions) != len(servoIDs) {
		return fmt.Errorf("position count mismatch: expected %d, got %d", len(servoIDs), len(positions))
	}

	c.clearSerialBuffers()
	c.enforceCommandGap()

	dataLen := 2
	paramLen := len(servoIDs) * (1 + dataLen)

	packet := []byte{
		PKT_HEADER1, PKT_HEADER2,
		BROADCAST_ID,
		byte(4 + paramLen),
		INST_SYNC_WRITE,
		ADDR_GOAL_POSITION,
		byte(dataLen),
	}

	for i, id := range servoIDs {
		position := positions[i]
		packet = append(packet, byte(id))
		packet = append(packet, byte(position&0xFF))
		packet = append(packet, byte((position>>8)&0xFF))
	}

	checksum := c.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	return c.writePacketVerified(packet)
}

// Rest of the methods remain the same but use the full calibration structure
func (c *SoArmController) syncWritePositionsRobust(positions []int) error {
	return c.syncWriteSpecificServos(c.servoIDs, positions)
}

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
	c.logger.Infof("Torque %s for servos %v", status, c.servoIDs)
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

// Keep all the existing helper methods unchanged
func (c *SoArmController) clearSerialBuffers() {
	if c.port == nil {
		return
	}
	c.port.ResetInputBuffer()
	c.port.SetReadTimeout(10 * time.Millisecond)
	buffer := make([]byte, 256)
	for {
		n, err := c.port.Read(buffer)
		if err != nil || n == 0 {
			break
		}
		c.logger.Debugf("Cleared %d bytes from input buffer", n)
	}
	c.port.SetReadTimeout(c.timeout)
}

func (c *SoArmController) enforceCommandGap() {
	elapsed := time.Since(c.lastCommandTime)
	if elapsed < c.minCommandGap {
		time.Sleep(c.minCommandGap - elapsed)
	}
	c.lastCommandTime = time.Now()
}

func (c *SoArmController) readCurrentPositionRobust(servoID int) (int, error) {
	packet := []byte{
		PKT_HEADER1, PKT_HEADER2,
		byte(servoID),
		0x04,
		INST_READ,
		ADDR_PRESENT_POSITION,
		0x02,
	}

	checksum := c.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	c.clearSerialBuffers()

	if err := c.writePacketVerified(packet); err != nil {
		return 0, fmt.Errorf("failed to send packet: %w", err)
	}

	response, err := c.readResponseRobust(8)
	if err != nil {
		return 0, err
	}

	if len(response) < 8 {
		return 0, fmt.Errorf("response too short: got %d bytes, expected 8", len(response))
	}

	if response[0] != PKT_HEADER1 || response[1] != PKT_HEADER2 {
		return 0, fmt.Errorf("invalid packet header: got [0x%02X, 0x%02X], expected [0xFF, 0xFF]", response[0], response[1])
	}

	if response[PKT_ID] != byte(servoID) {
		return 0, fmt.Errorf("invalid position response from servo %d: got ID %d", servoID, response[PKT_ID])
	}

	if !c.verifyChecksum(response) {
		return 0, fmt.Errorf("checksum verification failed")
	}

	if len(response) < 7 {
		return 0, fmt.Errorf("response too short for position data")
	}

	position := int(response[5]) | (int(response[6]) << 8)
	return position, nil
}

func (c *SoArmController) writePacketVerified(packet []byte) error {
	c.port.ResetInputBuffer()

	n, err := c.port.Write(packet)
	if err != nil {
		return fmt.Errorf("failed to write packet: %w", err)
	}
	if n != len(packet) {
		return fmt.Errorf("incomplete packet write: wrote %d of %d bytes", n, len(packet))
	}

	time.Sleep(time.Millisecond * 2)
	return nil
}

func (c *SoArmController) readResponseRobust(expectedLen int) ([]byte, error) {
	if err := c.port.SetReadTimeout(c.timeout); err != nil {
		return nil, fmt.Errorf("failed to set read timeout: %w", err)
	}

	buffer := make([]byte, expectedLen*2)
	totalRead := 0
	startTime := time.Now()

	for totalRead < expectedLen {
		if time.Since(startTime) > c.timeout {
			return nil, fmt.Errorf("timeout reading response after %v", c.timeout)
		}

		n, err := c.port.Read(buffer[totalRead:])
		if err != nil {
			return nil, fmt.Errorf("failed to read response: %w", err)
		}

		if n == 0 {
			if time.Since(startTime) > time.Millisecond*100 {
				break
			}
			time.Sleep(time.Millisecond * 5)
			continue
		}

		totalRead += n

		if totalRead >= 6 {
			for i := 0; i <= totalRead-6; i++ {
				if buffer[i] == PKT_HEADER1 && buffer[i+1] == PKT_HEADER2 {
					if i+3 < totalRead {
						packetLength := int(buffer[i+3]) + 4
						if i+packetLength <= totalRead {
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

	response := make([]byte, totalRead)
	copy(response, buffer[:totalRead])
	return response, nil
}

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
