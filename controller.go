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

// SoArmController handles communication with SO-ARM servo motors
type SoArmController struct {
	port     serial.Port
	servoIDs []int
	logger   logging.Logger
	mu       sync.RWMutex
	timeout  time.Duration
}

// Servo command constants for SO-ARM protocol
const (
	// Basic commands
	CMD_PING       = 0x01
	CMD_READ_DATA  = 0x02
	CMD_WRITE_DATA = 0x03
	CMD_REG_WRITE  = 0x04
	CMD_ACTION     = 0x05
	CMD_RESET      = 0x06
	CMD_SYNC_WRITE = 0x83

	// Memory addresses
	ADDR_TORQUE_ENABLE      = 0x18
	ADDR_GOAL_POSITION_L    = 0x1E
	ADDR_GOAL_POSITION_H    = 0x1F
	ADDR_MOVING_SPEED_L     = 0x20
	ADDR_MOVING_SPEED_H     = 0x21
	ADDR_PRESENT_POSITION_L = 0x24
	ADDR_PRESENT_POSITION_H = 0x25

	// Servo limits
	SERVO_MIN_POSITION    = 0
	SERVO_MAX_POSITION    = 4095
	SERVO_CENTER_POSITION = 2048
)

// NewSoArmController creates a new controller for SO-ARM servos
func NewSoArmController(portName string, baudrate int, servoIDs []int, logger logging.Logger) (*SoArmController, error) {
	if logger == nil {
		return nil, errors.New("logger cannot be nil")
	}

	// Open serial port using go.bug.st/serial
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
		port:     port,
		servoIDs: servoIDs,
		logger:   logger,
		timeout:  time.Second * 5,
	}

	// Test communication
	if err := controller.Ping(); err != nil {
		port.Close()
		return nil, fmt.Errorf("failed to communicate with servos: %w", err)
	}

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
	packet := c.buildPacket(servoID, CMD_PING, nil)
	if err := c.writePacket(packet); err != nil {
		return err
	}

	// Read response
	response, err := c.readResponse()
	if err != nil {
		return err
	}

	if len(response) < 6 || response[2] != byte(servoID) {
		return fmt.Errorf("invalid ping response from servo %d", servoID)
	}

	return nil
}

// SetTorqueEnable enables or disables torque for all servos
func (c *SoArmController) SetTorqueEnable(enable bool) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	value := byte(0)
	if enable {
		value = 1
	}

	for _, id := range c.servoIDs {
		if err := c.writeRegister(id, ADDR_TORQUE_ENABLE, []byte{value}); err != nil {
			return fmt.Errorf("failed to set torque enable for servo %d: %w", id, err)
		}
	}

	c.logger.Infof("Torque %s for all servos", map[bool]string{true: "enabled", false: "disabled"}[enable])
	return nil
}

// MoveToJointPositions moves all joints to specified angles
func (c *SoArmController) MoveToJointPositions(jointAngles []float64, speed, acceleration int) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if len(jointAngles) != len(c.servoIDs) {
		return fmt.Errorf("expected %d joint angles, got %d", len(c.servoIDs), len(jointAngles))
	}

	// Convert radians to servo positions
	positions := make([]int, len(jointAngles))
	for i, angle := range jointAngles {
		positions[i] = c.radiansToServoPosition(angle)
	}

	// Set speed for all servos first
	if err := c.setMovingSpeed(speed); err != nil {
		return fmt.Errorf("failed to set moving speed: %w", err)
	}

	// Move all servos simultaneously using sync write
	if err := c.syncWritePositions(positions); err != nil {
		return fmt.Errorf("failed to move servos: %w", err)
	}

	c.logger.Debugf("Moving servos to positions: %v (angles: %v rad)", positions, jointAngles)
	return nil
}

// GetJointPositions reads current positions of all joints
func (c *SoArmController) GetJointPositions() ([]float64, error) {
	c.mu.RLock()
	defer c.mu.RUnlock()

	angles := make([]float64, len(c.servoIDs))
	for i, id := range c.servoIDs {
		position, err := c.readCurrentPosition(id)
		if err != nil {
			return nil, fmt.Errorf("failed to read position from servo %d: %w", id, err)
		}
		angles[i] = c.servoPositionToRadians(position)
	}

	return angles, nil
}

// Stop stops all servo movement
func (c *SoArmController) Stop() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	// Stop by setting speed to 0
	return c.setMovingSpeed(0)
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

// radiansToServoPosition converts radians to servo position (0-4095)
func (c *SoArmController) radiansToServoPosition(radians float64) int {
	// Convert radians to degrees, then to servo position
	// Assuming ±180° maps to 0-4095 range
	degrees := radians * 180.0 / math.Pi
	position := int((degrees + 180.0) * 4095.0 / 360.0)

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
	// Convert servo position to degrees, then to radians
	degrees := (float64(position) * 360.0 / 4095.0) - 180.0
	return degrees * math.Pi / 180.0
}

// setMovingSpeed sets the moving speed for all servos
func (c *SoArmController) setMovingSpeed(speed int) error {
	if speed < 0 || speed > 4094 {
		return fmt.Errorf("speed must be between 0 and 4094, got %d", speed)
	}

	speedBytes := []byte{byte(speed & 0xFF), byte((speed >> 8) & 0xFF)}

	for _, id := range c.servoIDs {
		if err := c.writeRegister(id, ADDR_MOVING_SPEED_L, speedBytes); err != nil {
			return fmt.Errorf("failed to set speed for servo %d: %w", id, err)
		}
	}

	return nil
}

// syncWritePositions moves multiple servos simultaneously
func (c *SoArmController) syncWritePositions(positions []int) error {
	if len(positions) != len(c.servoIDs) {
		return fmt.Errorf("position count mismatch: expected %d, got %d", len(c.servoIDs), len(positions))
	}

	// Build sync write packet
	dataLen := 2 // 2 bytes per position (low byte + high byte)
	params := make([]byte, 0, 2+len(c.servoIDs)*(1+dataLen))
	params = append(params, ADDR_GOAL_POSITION_L, byte(dataLen))

	for i, id := range c.servoIDs {
		position := positions[i]
		params = append(params, byte(id))
		params = append(params, byte(position&0xFF))      // Low byte
		params = append(params, byte((position>>8)&0xFF)) // High byte
	}

	packet := c.buildPacket(0xFE, CMD_SYNC_WRITE, params) // 0xFE = broadcast ID
	return c.writePacket(packet)
}

// readCurrentPosition reads the current position of a servo
func (c *SoArmController) readCurrentPosition(servoID int) (int, error) {
	params := []byte{ADDR_PRESENT_POSITION_L, 2} // Read 2 bytes starting from position low byte
	packet := c.buildPacket(servoID, CMD_READ_DATA, params)

	if err := c.writePacket(packet); err != nil {
		return 0, err
	}

	response, err := c.readResponse()
	if err != nil {
		return 0, err
	}

	if len(response) < 8 || response[2] != byte(servoID) {
		return 0, fmt.Errorf("invalid position response from servo %d", servoID)
	}

	// Extract position from response (little endian)
	position := int(response[5]) | (int(response[6]) << 8)
	return position, nil
}

// writeRegister writes data to a servo register
func (c *SoArmController) writeRegister(servoID int, address byte, data []byte) error {
	params := make([]byte, 0, 1+len(data))
	params = append(params, address)
	params = append(params, data...)

	packet := c.buildPacket(servoID, CMD_WRITE_DATA, params)
	return c.writePacket(packet)
}

// buildPacket builds a SO-ARM protocol packet
func (c *SoArmController) buildPacket(id int, instruction byte, params []byte) []byte {
	length := len(params) + 2 // instruction + checksum
	packet := make([]byte, 0, 6+len(params))

	packet = append(packet, 0xFF, 0xFF)   // Header
	packet = append(packet, byte(id))     // Servo ID
	packet = append(packet, byte(length)) // Length
	packet = append(packet, instruction)  // Instruction
	packet = append(packet, params...)    // Parameters

	// Calculate checksum
	checksum := byte(id) + byte(length) + instruction
	for _, param := range params {
		checksum += param
	}
	checksum = ^checksum // Bitwise NOT

	packet = append(packet, checksum)
	return packet
}

// writePacket writes a packet to the serial port
func (c *SoArmController) writePacket(packet []byte) error {
	// Reset input buffer
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
func (c *SoArmController) readResponse() ([]byte, error) {
	// Read header first
	header := make([]byte, 4)
	if err := c.readWithTimeout(header); err != nil {
		return nil, fmt.Errorf("failed to read header: %w", err)
	}

	if header[0] != 0xFF || header[1] != 0xFF {
		return nil, errors.New("invalid packet header")
	}

	// Read the rest based on length
	length := int(header[3])
	if length < 2 || length > 255 {
		return nil, fmt.Errorf("invalid packet length: %d", length)
	}

	remaining := make([]byte, length)
	if err := c.readWithTimeout(remaining); err != nil {
		return nil, fmt.Errorf("failed to read packet body: %w", err)
	}

	// Combine header and body
	response := append(header, remaining...)

	// Verify checksum
	if !c.verifyChecksum(response) {
		return nil, errors.New("checksum verification failed")
	}

	return response, nil
}

// readWithTimeout reads data with a timeout
func (c *SoArmController) readWithTimeout(buffer []byte) error {
	// Set read timeout
	if err := c.port.SetReadTimeout(c.timeout); err != nil {
		return fmt.Errorf("failed to set read timeout: %w", err)
	}

	totalRead := 0
	for totalRead < len(buffer) {
		n, err := c.port.Read(buffer[totalRead:])
		if err != nil {
			return fmt.Errorf("failed to read data: %w", err)
		}
		totalRead += n
	}

	return nil
}

// verifyChecksum verifies the packet checksum
func (c *SoArmController) verifyChecksum(packet []byte) bool {
	if len(packet) < 6 {
		return false
	}

	// Calculate expected checksum
	checksum := byte(0)
	for i := 2; i < len(packet)-1; i++ {
		checksum += packet[i]
	}
	checksum = ^checksum // Bitwise NOT

	return checksum == packet[len(packet)-1]
}
