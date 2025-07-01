package arm

import (
	"context"
	"encoding/binary"
	"sync"
	"time"

	"github.com/jacobsa/go-serial/serial"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
	"go.viam.com/utils/serial"
)

// SO101Config represents the configuration for the SO-101 arm
type SO101Config struct {
	Port     string `json:"port"`
	Baudrate int    `json:"baudrate,omitempty"`
	Debug    bool   `json:"debug,omitempty"`
	ArmType  string `json:"arm_type,omitempty"` // "leader" or "follower"
	ServoIDs []int  `json:"servo_ids,omitempty"`
}

// SO101Controller implements the arm.Arm interface for the SO-101 robotic arm
type SO101Controller struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	mu         sync.RWMutex
	port       string
	baudrate   int
	serialPort serial.OpenOptions // Fixed: Use serial.OpenOptions instead of *serial.Options
	conn       serial.Port        // Fixed: Use serial.Port instead of *serial.Port
	logger     logging.Logger
	debug      bool
	armType    string // "leader" or "follower"

	// Servo configuration
	servoCount   int
	servoIDs     []int
	currentPose  spatialmath.Pose
	homePosition []float64
	model        referenceframe.Model
}

// Feetech protocol constants - using different names to avoid conflicts
const (
	// Protocol constants
	CONTROLLER_FRAME_HEADER = 0xFF
	CONTROLLER_BROADCAST_ID = 0xFE

	// Instruction constants (different names from arm.go)
	CONTROLLER_INST_PING       = 0x01
	CONTROLLER_INST_READ       = 0x02
	CONTROLLER_INST_WRITE      = 0x03
	CONTROLLER_INST_REG_WRITE  = 0x04
	CONTROLLER_INST_ACTION     = 0x05
	CONTROLLER_INST_RESET      = 0x06
	CONTROLLER_INST_SYNC_WRITE = 0x83

	// Control table addresses (different names from arm.go)
	CONTROLLER_ADDR_MODEL_NUMBER     = 0x00
	CONTROLLER_ADDR_FIRMWARE_VERSION = 0x02
	CONTROLLER_ADDR_ID               = 0x03
	CONTROLLER_ADDR_BAUD_RATE        = 0x04
	CONTROLLER_ADDR_GOAL_POSITION    = 0x2A
	CONTROLLER_ADDR_PRESENT_POSITION = 0x38
	CONTROLLER_ADDR_TORQUE_ENABLE    = 0x28
	CONTROLLER_ADDR_MOVING_SPEED     = 0x2E
	CONTROLLER_ADDR_GOAL_TIME        = 0x2C

	// Default values
	CONTROLLER_DEFAULT_BAUDRATE    = 1000000
	CONTROLLER_DEFAULT_SERVO_COUNT = 6
	CONTROLLER_PROTOCOL_TIMEOUT    = 100 * time.Millisecond
)

// Validate validates the SO101Config
func (cfg *SO101Config) Validate(path string) ([]string, []string, error) {
	var deps []string
	if cfg.Port == "" {
		return nil, nil, errors.New("serial port is required")
	}
	if cfg.Baudrate <= 0 {
		cfg.Baudrate = CONTROLLER_DEFAULT_BAUDRATE
	}
	if cfg.ArmType == "" {
		cfg.ArmType = "follower" // Default to follower
	}
	if cfg.ArmType != "leader" && cfg.ArmType != "follower" {
		return nil, nil, errors.New("arm_type must be either 'leader' or 'follower'")
	}
	if len(cfg.ServoIDs) == 0 {
		cfg.ServoIDs = []int{1, 2, 3, 4, 5, 6} // Default servo IDs
	}
	return deps, nil, nil
}

// NewSO101Controller creates a new SO-101 arm controller
func NewSO101Controller(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (arm.Arm, error) {
	newConf, err := resource.NativeConfig[*SO101Config](conf)
	if err != nil {
		return nil, err
	}

	controller := &SO101Controller{
		Named:        conf.ResourceName().AsNamed(),
		port:         newConf.Port,
		baudrate:     newConf.Baudrate,
		logger:       logger,
		debug:        newConf.Debug,
		armType:      newConf.ArmType,
		servoCount:   len(newConf.ServoIDs),
		servoIDs:     newConf.ServoIDs,
		homePosition: make([]float64, len(newConf.ServoIDs)),
	}

	// Set default home positions (center position for all servos)
	for i := range controller.homePosition {
		controller.homePosition[i] = 512
	}

	if err := controller.connect(); err != nil {
		return nil, errors.Wrap(err, "failed to connect to SO-101 arm")
	}

	// Initialize servos
	if err := controller.initServos(); err != nil {
		return nil, errors.Wrap(err, "failed to initialize servos")
	}

	return controller, nil
}

// connect establishes a serial connection to the arm
func (c *SO101Controller) connect() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	options := serial.OpenOptions{
		PortName:        c.port,
		BaudRate:        uint(c.baudrate),
		DataBits:        8,
		StopBits:        1,
		MinimumReadSize: 1,
	}

	port, err := serial.Open(options)
	if err != nil {
		return errors.Wrapf(err, "failed to open serial port %s", c.port)
	}

	c.conn = port // Fixed: Assign directly, not as pointer
	c.logger.Infof("Connected to SO-101 %s arm on port %s at %d baud", c.armType, c.port, c.baudrate)
	return nil
}

// initServos initializes all servos in the arm
func (c *SO101Controller) initServos() error {
	c.logger.Infof("Initializing SO-101 %s arm servos...", c.armType)

	// Ping all servos to verify connectivity
	for _, id := range c.servoIDs {
		if err := c.pingServo(id); err != nil {
			c.logger.Warnf("Failed to ping servo %d: %v", id, err)
		} else {
			c.logger.Debugf("Servo %d responded to ping", id)
		}
	}

	// Enable torque for all servos (except for leader arm in read-only mode)
	torqueValue := byte(1)
	if c.armType == "leader" {
		// For leader arm, you might want to disable torque to allow manual manipulation
		// Uncomment the next line if you want the leader arm to be manually movable
		// torqueValue = 0
	}

	for _, id := range c.servoIDs {
		if err := c.writeRegister(id, CONTROLLER_ADDR_TORQUE_ENABLE, []byte{torqueValue}); err != nil {
			c.logger.Warnf("Failed to set torque for servo %d: %v", id, err)
		}
	}

	return nil
}

// sendPacket sends a packet to the servo and returns the response
func (c *SO101Controller) sendPacket(id int, instruction byte, params []byte) ([]byte, error) {
	length := len(params) + 2 // instruction + checksum
	packet := make([]byte, 0, 6+len(params))

	// Build packet: [0xFF, 0xFF, ID, LENGTH, INSTRUCTION, ...PARAMS, CHECKSUM]
	packet = append(packet, CONTROLLER_FRAME_HEADER, CONTROLLER_FRAME_HEADER)
	packet = append(packet, byte(id), byte(length), instruction)
	packet = append(packet, params...)

	// Calculate checksum
	checksum := byte(0)
	for i := 2; i < len(packet); i++ {
		checksum += packet[i]
	}
	checksum = ^checksum
	packet = append(packet, checksum)

	if c.debug {
		c.logger.Debugf("Sending packet to servo %d: %x", id, packet)
	}

	// Send packet
	_, err := c.conn.Write(packet) // Fixed: Remove pointer dereference
	if err != nil {
		return nil, errors.Wrap(err, "failed to write to serial port")
	}

	// Read response for non-broadcast commands
	if id != CONTROLLER_BROADCAST_ID {
		response := make([]byte, 256)
		c.conn.SetReadTimeout(CONTROLLER_PROTOCOL_TIMEOUT) // Fixed: Remove pointer dereference
		n, err := c.conn.Read(response)                    // Fixed: Remove pointer dereference
		if err != nil {
			return nil, errors.Wrap(err, "failed to read response")
		}

		if c.debug {
			c.logger.Debugf("Received response: %x", response[:n])
		}

		return response[:n], nil
	}

	return nil, nil
}

// pingServo pings a specific servo
func (c *SO101Controller) pingServo(id int) error {
	_, err := c.sendPacket(id, CONTROLLER_INST_PING, []byte{})
	return err
}

// writeRegister writes data to a servo register
func (c *SO101Controller) writeRegister(id int, address int, data []byte) error {
	params := make([]byte, 0, 1+len(data))
	params = append(params, byte(address))
	params = append(params, data...)
	_, err := c.sendPacket(id, CONTROLLER_INST_WRITE, params)
	return err
}

// readRegister reads data from a servo register
func (c *SO101Controller) readRegister(id int, address int, length int) ([]byte, error) {
	params := []byte{byte(address), byte(length)}
	response, err := c.sendPacket(id, CONTROLLER_INST_READ, params)
	if err != nil {
		return nil, err
	}

	if len(response) < 6 {
		return nil, errors.New("invalid response length")
	}

	// Extract data from response (skip header, id, length, error, checksum)
	dataLength := int(response[3]) - 2
	if dataLength <= 0 || len(response) < 5+dataLength {
		return nil, errors.New("invalid data length in response")
	}

	return response[5 : 5+dataLength], nil
}

// setPosition sets the target position for a servo
func (c *SO101Controller) setPosition(id int, position int, speed int) error {
	posBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(posBytes, uint16(position))

	speedBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(speedBytes, uint16(speed))

	// Write position
	if err := c.writeRegister(id, CONTROLLER_ADDR_GOAL_POSITION, posBytes); err != nil {
		return err
	}

	// Write speed
	return c.writeRegister(id, CONTROLLER_ADDR_MOVING_SPEED, speedBytes)
}

// getPosition gets the current position of a servo
func (c *SO101Controller) getPosition(id int) (int, error) {
	data, err := c.readRegister(id, CONTROLLER_ADDR_PRESENT_POSITION, 2)
	if err != nil {
		return 0, err
	}

	if len(data) < 2 {
		return 0, errors.New("insufficient position data")
	}

	position := int(binary.LittleEndian.Uint16(data))
	return position, nil
}

// Arm interface implementations

// EndPosition returns the current end effector position
func (c *SO101Controller) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	c.mu.RLock()
	defer c.mu.RUnlock()

	// Read current positions from all servos
	positions := make([]int, len(c.servoIDs))
	for i, id := range c.servoIDs {
		pos, err := c.getPosition(id)
		if err != nil {
			c.logger.Warnf("Failed to read position from servo %d: %v", id, err)
			positions[i] = 512 // Default to center position
		} else {
			positions[i] = pos
		}
	}

	// Convert servo positions to end effector pose
	// This is a simplified implementation - you'll need to implement
	// proper forward kinematics based on your arm's geometry
	pose := spatialmath.Pose{}

	c.currentPose = pose
	return pose, nil
}

// MoveToPosition moves the arm to a target pose
func (c *SO101Controller) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.logger.Infof("Moving %s arm to position: %v", c.armType, pose.Point())

	// Convert pose to joint angles using inverse kinematics
	// This is a simplified implementation - you'll need to implement
	// proper inverse kinematics based on your arm's geometry
	point := pose.Point()

	// Simple mapping from Cartesian coordinates to servo positions
	positions := make([]int, len(c.servoIDs))
	if len(positions) >= 6 {
		positions[0] = int(point.X*1000) + 512 // Base rotation
		positions[1] = int(point.Y*1000) + 512 // Shoulder
		positions[2] = int(point.Z*1000) + 300 // Elbow (account for base height)
		positions[3] = 512                     // Wrist rotation
		positions[4] = 512                     // Wrist tilt
		positions[5] = 512                     // Gripper
	}

	// Fill remaining servos with center position if more than 6
	for i := 6; i < len(positions); i++ {
		positions[i] = 512
	}

	// Clamp positions to valid range (0-1023 for most Feetech servos)
	for i := range positions {
		if positions[i] < 0 {
			positions[i] = 0
		} else if positions[i] > 1023 {
			positions[i] = 1023
		}
	}

	// Move all servos to target positions
	for i, id := range c.servoIDs {
		if err := c.setPosition(id, positions[i], 100); err != nil {
			return errors.Wrapf(err, "failed to move servo %d on %s arm", id, c.armType)
		}
	}

	// Wait for movement to complete
	time.Sleep(time.Second)

	c.currentPose = pose
	return nil
}

// MoveToJointPositions moves the arm to specific joint positions
func (c *SO101Controller) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.logger.Infof("Moving %s arm to joint positions", c.armType)

	if len(positions) != len(c.servoIDs) {
		return errors.Errorf("expected %d joint positions, got %d", len(c.servoIDs), len(positions))
	}

	// Convert radians to servo positions (assuming -π to π maps to 0-1023)
	for i, id := range c.servoIDs {
		servoPos := int((positions[i].Value/(2*3.14159) + 0.5) * 1023)
		if servoPos < 0 {
			servoPos = 0
		} else if servoPos > 1023 {
			servoPos = 1023
		}

		if err := c.setPosition(id, servoPos, 100); err != nil {
			return errors.Wrapf(err, "failed to move servo %d on %s arm", id, c.armType)
		}
	}

	// Wait for movement to complete
	time.Sleep(time.Second)

	return nil
}

// JointPositions returns the current joint positions
func (c *SO101Controller) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	c.mu.RLock()
	defer c.mu.RUnlock()

	positions := make([]referenceframe.Input, len(c.servoIDs))
	for i, id := range c.servoIDs {
		pos, err := c.getPosition(id)
		if err != nil {
			c.logger.Warnf("Failed to read position from servo %d: %v", id, err)
			positions[i] = referenceframe.Input{Value: 0}
		} else {
			// Convert servo position (0-1023) to radians (-π to π)
			radians := (float64(pos)/1023.0 - 0.5) * 2 * 3.14159
			positions[i] = referenceframe.Input{Value: radians}
		}
	}

	return positions, nil
}

// Stop stops the arm
func (c *SO101Controller) Stop(ctx context.Context, extra map[string]interface{}) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.logger.Infof("Stopping %s arm", c.armType)

	// Disable torque for all servos
	for _, id := range c.servoIDs {
		if err := c.writeRegister(id, CONTROLLER_ADDR_TORQUE_ENABLE, []byte{0}); err != nil {
			c.logger.Warnf("Failed to disable torque for servo %d: %v", id, err)
		}
	}

	return nil
}

// IsMoving returns whether the arm is currently moving
func (c *SO101Controller) IsMoving(ctx context.Context) (bool, error) {
	// For simplicity, always return false
	// In a real implementation, you would check if any servo is currently moving
	return false, nil
}

// CurrentInputs returns the current state of the arm
func (c *SO101Controller) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return c.JointPositions(ctx, nil)
}

// GoToInputs moves the arm to the specified input positions
func (c *SO101Controller) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	for _, inputs := range inputSteps {
		if err := c.MoveToJointPositions(ctx, inputs, nil); err != nil {
			return err
		}
	}
	return nil
}

// MoveThroughJointPositions moves through a series of joint positions
func (c *SO101Controller) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	for _, jointPositions := range positions {
		if err := c.MoveToJointPositions(ctx, jointPositions, extra); err != nil {
			return err
		}

		if ctx.Err() != nil {
			return ctx.Err()
		}

		time.Sleep(100 * time.Millisecond)
	}
	return nil
}

// Geometries returns the geometries of the arm
func (c *SO101Controller) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return []spatialmath.Geometry{}, nil
}

// Kinematics returns the kinematics model
func (c *SO101Controller) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, errors.New("kinematics model not implemented")
}

// NewClientFromConn creates a new client from connection
func (c *SO101Controller) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (arm.Arm, error) {
	return nil, errors.New("remote client not implemented")
}

// Close closes the connection to the arm
func (c *SO101Controller) Close(ctx context.Context) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	// Disable torque for all servos before closing
	for _, id := range c.servoIDs {
		c.writeRegister(id, CONTROLLER_ADDR_TORQUE_ENABLE, []byte{0})
	}

	if c.conn != nil {
		err := c.conn.Close() // Fixed: Remove pointer dereference
		return err
	}
	return nil
}

// DoCommand executes custom commands
func (c *SO101Controller) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "home":
		return c.doHome(ctx)
	case "set_servo_position":
		return c.doSetServoPosition(ctx, cmd)
	case "get_servo_position":
		return c.doGetServoPosition(ctx, cmd)
	case "mirror_positions":
		return c.doMirrorPositions(ctx, cmd)
	case "set_torque_enable":
		return c.doSetTorqueEnable(ctx, cmd)
	case "get_arm_info":
		return c.doGetArmInfo(ctx)
	default:
		return nil, errors.Errorf("unknown command: %v", cmd["command"])
	}
}

// doHome moves the arm to home position
func (c *SO101Controller) doHome(ctx context.Context) (map[string]interface{}, error) {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.logger.Infof("Moving %s arm to home position", c.armType)

	for i, id := range c.servoIDs {
		if err := c.setPosition(id, int(c.homePosition[i]), 50); err != nil {
			return nil, errors.Wrapf(err, "failed to home servo %d on %s arm", id, c.armType)
		}
	}

	time.Sleep(2 * time.Second)
	return map[string]interface{}{
		"status":   "homed",
		"arm_type": c.armType,
	}, nil
}

// doSetServoPosition sets a specific servo position
func (c *SO101Controller) doSetServoPosition(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	id, ok := cmd["servo_id"].(float64)
	if !ok {
		return nil, errors.New("servo_id must be a number")
	}

	position, ok := cmd["position"].(float64)
	if !ok {
		return nil, errors.New("position must be a number")
	}

	speed := 100.0
	if s, ok := cmd["speed"].(float64); ok {
		speed = s
	}

	err := c.setPosition(int(id), int(position), int(speed))
	if err != nil {
		return nil, err
	}

	return map[string]interface{}{"status": "moved"}, nil
}

// doGetServoPosition gets a specific servo position
func (c *SO101Controller) doGetServoPosition(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	id, ok := cmd["servo_id"].(float64)
	if !ok {
		return nil, errors.New("servo_id must be a number")
	}

	position, err := c.getPosition(int(id))
	if err != nil {
		return nil, err
	}

	return map[string]interface{}{
		"position": position,
		"servo_id": int(id),
		"arm_type": c.armType,
	}, nil
}

// doMirrorPositions copies positions from leader to follower (used externally)
func (c *SO101Controller) doMirrorPositions(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	positions, ok := cmd["positions"].([]interface{})
	if !ok {
		return nil, errors.New("positions must be an array")
	}

	if len(positions) != len(c.servoIDs) {
		return nil, errors.Errorf("expected %d positions, got %d", len(c.servoIDs), len(positions))
	}

	c.logger.Debugf("Mirroring positions to %s arm", c.armType)

	for i, id := range c.servoIDs {
		pos, ok := positions[i].(float64)
		if !ok {
			return nil, errors.Errorf("position %d must be a number", i)
		}

		if err := c.setPosition(id, int(pos), 100); err != nil {
			return nil, errors.Wrapf(err, "failed to set position for servo %d", id)
		}
	}

	return map[string]interface{}{
		"status":   "mirrored",
		"arm_type": c.armType,
	}, nil
}

// doSetTorqueEnable enables or disables torque for all servos
func (c *SO101Controller) doSetTorqueEnable(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	enable, ok := cmd["enable"].(bool)
	if !ok {
		return nil, errors.New("enable must be a boolean")
	}

	torqueValue := byte(0)
	if enable {
		torqueValue = 1
	}

	c.logger.Infof("Setting torque enable to %v for %s arm", enable, c.armType)

	for _, id := range c.servoIDs {
		if err := c.writeRegister(id, CONTROLLER_ADDR_TORQUE_ENABLE, []byte{torqueValue}); err != nil {
			c.logger.Warnf("Failed to set torque for servo %d: %v", id, err)
		}
	}

	return map[string]interface{}{
		"status":         "torque_set",
		"arm_type":       c.armType,
		"torque_enabled": enable,
	}, nil
}

// doGetArmInfo returns information about the arm
func (c *SO101Controller) doGetArmInfo(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{
		"arm_type":    c.armType,
		"port":        c.port,
		"baudrate":    c.baudrate,
		"servo_count": len(c.servoIDs),
		"servo_ids":   c.servoIDs,
	}, nil
}

// ModelFrame returns the model frame of the arm
func (c *SO101Controller) ModelFrame() referenceframe.Model {
	if c.model != nil {
		return c.model
	}

	// Create a simple 6-DOF arm model
	// You should replace this with the actual kinematics of your SO-101 arm
	return nil
}

// init registers the SO-101 arm component
func init() {
	resource.RegisterComponent(
		arm.API,
		resource.DefaultModelFamily.WithModel("so101"),
		resource.Registration[arm.Arm, *SO101Config]{
			Constructor: NewSO101Controller,
		},
	)
}
