package arm

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
	"go.bug.st/serial"
)

var (
	So101Leader   = resource.NewModel("devrel", "arm", "so-101-leader")
	So101Follower = resource.NewModel("devrel", "arm", "so-101-follower")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterComponent(arm.API, So101Leader,
		resource.Registration[arm.Arm, *Config]{
			Constructor: newArmSo101Leader,
		},
	)
	resource.RegisterComponent(arm.API, So101Follower,
		resource.Registration[arm.Arm, *Config]{
			Constructor: newArmSo101Follower,
		},
	)
}

type Config struct {
	// Serial communication settings
	Port     string        `json:"port"`                       // Required: Serial port path (e.g., "/dev/ttyUSB0")
	Baudrate int           `json:"baudrate,omitempty"`         // Baudrate for Feetech servos (default: 1000000)
	Timeout  time.Duration `json:"timeout,omitempty"`          // Communication timeout (default: 5s)
	
	// Motion parameters
	DefaultSpeed       int     `json:"default_speed,omitempty"`        // Default servo speed (1-4094)
	DefaultAcceleration int    `json:"default_acceleration,omitempty"` // Default servo acceleration (0-254)
	
	// Servo configuration
	ServoIDs []int `json:"servo_ids,omitempty"` // Servo IDs for the 5 arm joints (default: [1,2,3,4,5])
	
	// Leader-Follower configuration
	Mode           string `json:"mode,omitempty"`            // "leader" or "follower"
	FollowerArm    string `json:"follower_arm,omitempty"`    // Name of follower arm (for leader mode)
	LeaderArm      string `json:"leader_arm,omitempty"`      // Name of leader arm (for follower mode)
	MirrorMode     bool   `json:"mirror_mode,omitempty"`     // Mirror movements horizontally
	ScaleFactor    float64 `json:"scale_factor,omitempty"`   // Scale factor for movements (default: 1.0)
	SyncRate       int    `json:"sync_rate,omitempty"`       // Sync rate in Hz (default: 20)
}

// Validate ensures all parts of the config are valid
func (cfg *Config) Validate(path string) ([]string, error) {
	if cfg.Port == "" {
		return nil, fmt.Errorf("serial port must be specified")
	}
	
	// Set defaults
	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000 // Standard Feetech baudrate
	}
	if cfg.Timeout == 0 {
		cfg.Timeout = 5 * time.Second
	}
	if cfg.DefaultSpeed == 0 {
		cfg.DefaultSpeed = 1000 // Mid-range speed
	}
	if cfg.DefaultAcceleration == 0 {
		cfg.DefaultAcceleration = 50 // Mid-range acceleration
	}
	if len(cfg.ServoIDs) == 0 {
		cfg.ServoIDs = []int{1, 2, 3, 4, 5} // Default servo IDs
	}
	if len(cfg.ServoIDs) != 5 {
		return nil, fmt.Errorf("expected 5 servo IDs for arm joints, got %d", len(cfg.ServoIDs))
	}
	if cfg.ScaleFactor == 0 {
		cfg.ScaleFactor = 1.0
	}
	if cfg.SyncRate == 0 {
		cfg.SyncRate = 20 // 20 Hz default
	}
	
	// Validate ranges
	if cfg.DefaultSpeed < 1 || cfg.DefaultSpeed > 4094 {
		return nil, fmt.Errorf("default_speed must be between 1 and 4094, got %d", cfg.DefaultSpeed)
	}
	if cfg.DefaultAcceleration < 0 || cfg.DefaultAcceleration > 254 {
		return nil, fmt.Errorf("default_acceleration must be between 0 and 254, got %d", cfg.DefaultAcceleration)
	}
	
	// Validate mode
	if cfg.Mode != "" && cfg.Mode != "leader" && cfg.Mode != "follower" {
		return nil, fmt.Errorf("mode must be 'leader' or 'follower', got '%s'", cfg.Mode)
	}
	
	return nil, nil
}

// Feetech protocol constants
const (
	// Instruction types
	INST_PING       = 0x01
	INST_READ       = 0x02
	INST_WRITE      = 0x03
	INST_SYNC_WRITE = 0x83
	
	// Register addresses (SCS series)
	ADDR_TORQUE_ENABLE    = 40
	ADDR_GOAL_POSITION    = 42
	ADDR_GOAL_SPEED       = 46
	ADDR_GOAL_ACCELERATION = 41
	ADDR_PRESENT_POSITION = 56
	ADDR_PRESENT_SPEED    = 58
	ADDR_PRESENT_LOAD     = 60
	
	// Protocol constants
	PKT_HEADER1 = 0xFF
	PKT_HEADER2 = 0xFF
	PKT_ID      = 0xFD
	PKT_RESERVED = 0x00
	BROADCAST_ID = 0xFE
)

// Joint limits for SO-101 arm (5 joints) in radians
var so101JointLimits = [][2]float64{
	{-math.Pi, math.Pi},       // Base rotation: ±180°
	{-math.Pi/2, math.Pi/2},   // Shoulder: ±90°
	{-math.Pi/2, math.Pi/2},   // Elbow: ±90°
	{-math.Pi/2, math.Pi/2},   // Wrist pitch: ±90°
	{-math.Pi, math.Pi},       // Wrist roll: ±180°
}

// FeetechController handles low-level communication with Feetech servos
type FeetechController struct {
	port     serial.Port
	servoIDs []int
	logger   logging.Logger
	mu       sync.Mutex
}

func NewFeetechController(portName string, baudrate int, servoIDs []int, logger logging.Logger) (*FeetechController, error) {
	mode := &serial.Mode{
		BaudRate: baudrate,
		Parity:   serial.NoParity,
		DataBits: 8,
		StopBits: serial.OneStopBit,
	}
	
	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open serial port %s: %w", portName, err)
	}
	
	controller := &FeetechController{
		port:     port,
		servoIDs: servoIDs,
		logger:   logger,
	}
	
	// Test communication with ping
	if err := controller.PingAll(); err != nil {
		port.Close()
		return nil, fmt.Errorf("failed to communicate with servos: %w", err)
	}
	
	return controller, nil
}

func (fc *FeetechController) Close() error {
	if fc.port != nil {
		return fc.port.Close()
	}
	return nil
}

// Calculate checksum for Feetech protocol
func (fc *FeetechController) calculateChecksum(data []byte) byte {
	sum := 0
	for _, b := range data[2:] { // Skip the two 0xFF headers
		sum += int(b)
	}
	return byte(^sum)
}

// Send packet and receive response
func (fc *FeetechController) sendPacket(id byte, instruction byte, params []byte) ([]byte, error) {
	fc.mu.Lock()
	defer fc.mu.Unlock()
	
	// Build packet
	length := byte(len(params) + 2) // instruction + checksum
	packet := []byte{PKT_HEADER1, PKT_HEADER2, id, length, instruction}
	packet = append(packet, params...)
	
	// Add checksum
	checksum := fc.calculateChecksum(packet)
	packet = append(packet, checksum)
	
	// Send packet
	_, err := fc.port.Write(packet)
	if err != nil {
		return nil, fmt.Errorf("failed to write packet: %w", err)
	}
	
	// Read response if not broadcast
	if id != BROADCAST_ID {
		response := make([]byte, 64) // Max expected response size
		n, err := fc.port.Read(response)
		if err != nil {
			return nil, fmt.Errorf("failed to read response: %w", err)
		}
		return response[:n], nil
	}
	
	return nil, nil
}

// Ping a servo
func (fc *FeetechController) Ping(id int) error {
	_, err := fc.sendPacket(byte(id), INST_PING, []byte{})
	return err
}

// Ping all servos
func (fc *FeetechController) PingAll() error {
	for _, id := range fc.servoIDs {
		if err := fc.Ping(id); err != nil {
			return fmt.Errorf("servo %d ping failed: %w", id, err)
		}
	}
	return nil
}

// Convert radians to servo position (0-4095)
func radiansToPosition(radians float64) uint16 {
	// Assuming 4096 positions for full rotation (2π radians)
	position := (radians + math.Pi) / (2 * math.Pi) * 4095
	if position < 0 {
		position = 0
	} else if position > 4095 {
		position = 4095
	}
	return uint16(position)
}

// Convert servo position to radians
func positionToRadians(position uint16) float64 {
	return (float64(position)/4095)*(2*math.Pi) - math.Pi
}

// Set torque enable for all servos
func (fc *FeetechController) SetTorqueEnable(enable bool) error {
	value := byte(0)
	if enable {
		value = 1
	}
	
	for _, id := range fc.servoIDs {
		_, err := fc.sendPacket(byte(id), INST_WRITE, []byte{ADDR_TORQUE_ENABLE, value})
		if err != nil {
			return fmt.Errorf("failed to set torque enable for servo %d: %w", id, err)
		}
	}
	return nil
}

// Move servos to joint positions
func (fc *FeetechController) MoveToJointPositions(angles []float64, speed int, acceleration int) error {
	if len(angles) != len(fc.servoIDs) {
		return fmt.Errorf("expected %d angles, got %d", len(fc.servoIDs), len(angles))
	}
	
	// Use sync write for coordinated movement
	params := []byte{ADDR_GOAL_POSITION, 4} // Address and data length per servo
	
	for i, angle := range angles {
		id := byte(fc.servoIDs[i])
		position := radiansToPosition(angle)
		speedBytes := make([]byte, 2)
		binary.LittleEndian.PutUint16(speedBytes, uint16(speed))
		
		params = append(params, id)
		params = append(params, byte(position&0xFF))        // Position low byte
		params = append(params, byte((position>>8)&0xFF))   // Position high byte
		params = append(params, speedBytes[0])              // Speed low byte
		params = append(params, speedBytes[1])              // Speed high byte
	}
	
	_, err := fc.sendPacket(BROADCAST_ID, INST_SYNC_WRITE, params)
	return err
}

// Get current joint positions
func (fc *FeetechController) GetJointPositions() ([]float64, error) {
	angles := make([]float64, len(fc.servoIDs))
	
	for i, id := range fc.servoIDs {
		response, err := fc.sendPacket(byte(id), INST_READ, []byte{ADDR_PRESENT_POSITION, 2})
		if err != nil {
			return nil, fmt.Errorf("failed to read position from servo %d: %w", id, err)
		}
		
		if len(response) < 7 {
			return nil, fmt.Errorf("invalid response length from servo %d", id)
		}
		
		// Extract position from response (bytes 5-6)
		position := binary.LittleEndian.Uint16(response[5:7])
		angles[i] = positionToRadians(position)
	}
	
	return angles, nil
}

// Stop all servos
func (fc *FeetechController) Stop() error {
	// Set speed to 0 for all servos
	for _, id := range fc.servoIDs {
		_, err := fc.sendPacket(byte(id), INST_WRITE, []byte{ADDR_GOAL_SPEED, 0x00, 0x00})
		if err != nil {
			return fmt.Errorf("failed to stop servo %d: %w", id, err)
		}
	}
	return nil
}

// Main arm structure
type armSo101 struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	// Hardware controller
	controller *FeetechController
	
	// Motion control
	mu          sync.RWMutex
	moveLock    sync.Mutex
	isMoving    atomic.Bool
	jointLimits [][2]float64
	
	// Motion parameters
	defaultSpeed int
	defaultAcc   int

	// Leader-Follower mode
	isLeader     bool
	isFollower   bool
	followerArm  arm.Arm
	leaderArm    arm.Arm
	syncTicker   *time.Ticker
	syncStop     chan struct{}

	cancelCtx  context.Context
	cancelFunc func()
}

func newArmSo101Leader(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	conf.Mode = "leader"
	return NewSo101(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func newArmSo101Follower(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	conf.Mode = "follower"
	return NewSo101(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewSo101(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (arm.Arm, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	// Initialize Feetech controller
	controller, err := NewFeetechController(conf.Port, conf.Baudrate, conf.ServoIDs, logger)
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("failed to initialize Feetech controller: %w", err)
	}

	s := &armSo101{
		name:         name,
		logger:       logger,
		cfg:          conf,
		controller:   controller,
		jointLimits:  so101JointLimits,
		defaultSpeed: conf.DefaultSpeed,
		defaultAcc:   conf.DefaultAcceleration,
		isLeader:     conf.Mode == "leader",
		isFollower:   conf.Mode == "follower",
		syncStop:     make(chan struct{}),
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
	}

	// Enable torque by default
	if err := controller.SetTorqueEnable(true); err != nil {
		logger.Warnf("Failed to enable torque: %v", err)
	}

	// Setup leader-follower relationship
	if s.isLeader && conf.FollowerArm != "" {
		go s.startLeaderSync(deps, conf.FollowerArm)
	} else if s.isFollower && conf.LeaderArm != "" {
		go s.startFollowerSync(deps, conf.LeaderArm)
	}

	logger.Infof("SO-101 arm (%s mode) initialized on port %s with servo IDs: %v", 
		conf.Mode, conf.Port, conf.ServoIDs)
	return s, nil
}

// Start synchronization for leader mode
func (s *armSo101) startLeaderSync(deps resource.Dependencies, followerName string) {
	ticker := time.NewTicker(time.Duration(1000/s.cfg.SyncRate) * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			// Get follower arm
			if s.followerArm == nil {
				followerResource, err := deps.Lookup(resource.NewName(arm.API, followerName))
				if err != nil {
					s.logger.Debugf("Follower arm not yet available: %v", err)
					continue
				}
				if followerArm, ok := followerResource.(arm.Arm); ok {
					s.followerArm = followerArm
					s.logger.Info("Connected to follower arm")
				}
			}

			// Sync positions
			if s.followerArm != nil {
				s.syncToFollower()
			}

		case <-s.syncStop:
			return
		case <-s.cancelCtx.Done():
			return
		}
	}
}

// Start synchronization for follower mode
func (s *armSo101) startFollowerSync(deps resource.Dependencies, leaderName string) {
	ticker := time.NewTicker(time.Duration(1000/s.cfg.SyncRate) * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			// Get leader arm
			if s.leaderArm == nil {
				leaderResource, err := deps.Lookup(resource.NewName(arm.API, leaderName))
				if err != nil {
					s.logger.Debugf("Leader arm not yet available: %v", err)
					continue
				}
				if leaderArm, ok := leaderResource.(arm.Arm); ok {
					s.leaderArm = leaderArm
					s.logger.Info("Connected to leader arm")
				}
			}

			// Sync from leader
			if s.leaderArm != nil {
				s.syncFromLeader()
			}

		case <-s.syncStop:
			return
		case <-s.cancelCtx.Done():
			return
		}
	}
}

// Sync current position to follower
func (s *armSo101) syncToFollower() {
	if s.followerArm == nil {
		return
	}

	ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
	defer cancel()

	// Get current position
	positions, err := s.JointPositions(ctx, nil)
	if err != nil {
		s.logger.Debugf("Failed to get leader positions: %v", err)
		return
	}

	// Apply mirroring and scaling if configured
	if s.cfg.MirrorMode || s.cfg.ScaleFactor != 1.0 {
		positions = s.transformPositions(positions)
	}

	// Send to follower
	err = s.followerArm.MoveToJointPositions(ctx, positions, map[string]interface{}{
		"speed": s.defaultSpeed,
	})
	if err != nil {
		s.logger.Debugf("Failed to sync to follower: %v", err)
	}
}

// Sync position from leader
func (s *armSo101) syncFromLeader() {
	if s.leaderArm == nil {
		return
	}

	ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
	defer cancel()

	// Get leader position
	positions, err := s.leaderArm.JointPositions(ctx, nil)
	if err != nil {
		s.logger.Debugf("Failed to get leader positions: %v", err)
		return
	}

	// Apply mirroring and scaling if configured
	if s.cfg.MirrorMode || s.cfg.ScaleFactor != 1.0 {
		positions = s.transformPositions(positions)
	}

	// Move to match leader
	err = s.MoveToJointPositions(ctx, positions, map[string]interface{}{
		"speed": s.defaultSpeed,
	})
	if err != nil {
		s.logger.Debugf("Failed to sync from leader: %v", err)
	}
}

// Transform positions for mirroring and scaling
func (s *armSo101) transformPositions(positions []referenceframe.Input) []referenceframe.Input {
	transformed := make([]referenceframe.Input, len(positions))
	
	for i, pos := range positions {
		value := pos.Value
		
		// Apply scaling
		if s.cfg.ScaleFactor != 1.0 {
			value *= s.cfg.ScaleFactor
		}
		
		// Apply mirroring (typically mirror base and wrist roll)
		if s.cfg.MirrorMode && (i == 0 || i == 4) {
			value = -value
		}
		
		transformed[i] = referenceframe.Input{Value: value}
	}
	
	return transformed
}

// Standard arm interface methods
func (s *armSo101) Name() resource.Name {
	return s.name
}

func (s *armSo101) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (arm.Arm, error) {
	return nil, errors.New("remote client not implemented")
}

func (s *armSo101) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	// Simple end position calculation - in practice you'd use forward kinematics
	joints, err := s.JointPositions(ctx, extra)
	if err != nil {
		return nil, err
	}
	
	// Simplified calculation
	x := 0.3 // Default reach
	y := 0.0
	z := 0.2
	
	pose := spatialmath.NewPose(
		spatialmath.R3{X: x, Y: y, Z: z},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: 0, Theta: joints[0].Value * 180 / math.Pi},
	)
	
	return pose, nil
}

func (s *armSo101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	return fmt.Errorf("MoveToPosition not implemented - use MoveToJointPositions instead")
}

func (s *armSo101) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	if len(positions) != 5 {
		return fmt.Errorf("expected 5 joint positions for SO-101, got %d", len(positions))
	}

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	// Extract and validate joint angles
	jointAngles := make([]float64, len(positions))
	for i, input := range positions {
		angle := input.Value
		min, max := s.jointLimits[i][0], s.jointLimits[i][1]
		
		// Clamp to joint limits
		if angle < min {
			s.logger.Warnf("Joint %d angle %.3f rad below limit %.3f rad, clamping", i+1, angle, min)
			angle = min
		} else if angle > max {
			s.logger.Warnf("Joint %d angle %.3f rad above limit %.3f rad, clamping", i+1, angle, max)
			angle = max
		}
		
		jointAngles[i] = angle
	}

	// Get motion parameters
	speed := s.defaultSpeed
	acc := s.defaultAcc
	
	if extra != nil {
		if speedVal, ok := extra["speed"].(int); ok && speedVal > 0 && speedVal <= 4094 {
			speed = speedVal
		}
		if accVal, ok := extra["acceleration"].(int); ok && accVal >= 0 && accVal <= 254 {
			acc = accVal
		}
	}

	// Send movement command to controller
	if err := s.controller.MoveToJointPositions(jointAngles, speed, acc); err != nil {
		return fmt.Errorf("failed to move to joint positions: %w", err)
	}

	return nil
}

func (s *armSo101) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	for _, jointPositions := range positions {
		if err := s.MoveToJointPositions(ctx, jointPositions, extra); err != nil {
			return err
		}
		
		if ctx.Err() != nil {
			return ctx.Err()
		}
		
		time.Sleep(100 * time.Millisecond)
	}
	return nil
}

func (s *armSo101) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	angles, err := s.controller.GetJointPositions()
	if err != nil {
		return nil, fmt.Errorf("failed to read joint positions: %w", err)
	}

	if len(angles) != 5 {
		return nil, fmt.Errorf("expected 5 joint angles, got %d", len(angles))
	}

	positions := make([]referenceframe.Input, 5)
	for i, angle := range angles {
		positions[i] = referenceframe.Input{Value: angle}
	}

	return positions, nil
}

func (s *armSo101) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.isMoving.Store(false)
	return s.controller.Stop()
}

func (s *armSo101) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, fmt.Errorf("kinematics model not implemented")
}

func (s *armSo101) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *armSo101) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *armSo101) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "set_torque":
		enable, ok := cmd["enable"].(bool)
		if !ok {
			return nil, fmt.Errorf("set_torque command requires 'enable' boolean parameter")
		}
		err := s.controller.SetTorqueEnable(enable)
		return map[string]interface{}{"success": err == nil}, err

	case "ping_servos":
		err := s.controller.PingAll()
		return map[string]interface{}{"success": err == nil}, err

	case "set_motion_params":
		result := make(map[string]interface{})
		
		if speedVal, ok := cmd["speed"].(float64); ok {
			speed := int(speedVal)
			if speed < 1 || speed > 4094 {
				return nil, fmt.Errorf("speed must be between 1 and 4094, got %d", speed)
			}
			s.mu.Lock()
			s.defaultSpeed = speed
			s.mu.Unlock()
			result["speed_set"] = speed
		}
		
		if accVal, ok := cmd["acceleration"].(float64); ok {
			acc := int(accVal)
			if acc < 0 || acc > 254 {
				return nil, fmt.Errorf("acceleration must be between 0 and 254, got %d", acc)
			}
			s.mu.Lock()
			s.defaultAcc = acc
			s.mu.Unlock()
			result["acceleration_set"] = acc
		}
		
		return result, nil

	case "start_sync":
		if s.isLeader || s.isFollower {
			return map[string]interface{}{"message": "synchronization already active"}, nil
		}
		return map[string]interface{}{"error": "not configured for leader-follower mode"}, nil

	case "stop_sync":
		select {
		case s.syncStop <- struct{}{}:
			return map[string]interface{}{"message": "synchronization stopped"}, nil
		default:
			return map[string]interface{}{"message": "synchronization already stopped"}, nil
		}

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (s *armSo101) IsMoving(ctx context.Context) (bool, error) {
	return s.isMoving.Load(), nil
}

func (s *armSo101) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return []spatialmath.Geometry{}, nil
}

func (s *armSo101) Close(context.Context) error {
	s.logger.Info("Closing SO-101 arm")
	
	// Stop synchronization
	select {
	case s.syncStop <- struct{}{}:
	default:
	}
	
	s.cancelFunc()
	
	if s.controller != nil {
		// Disable torque before closing
		s.controller.SetTorqueEnable(false)
		return s.controller.Close()
	}
	return nil
}
