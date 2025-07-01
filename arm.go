package arm

import (
	"context"
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

//go:embed soarm_101.json
var soarmModelJson []byte

var (
	So101Leader      = resource.NewModel("devrel", "arm", "so-101-leader")
	So101Follower    = resource.NewModel("devrel", "arm", "so-101-follower")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterComponent(arm.API, So101Leader,
		resource.Registration[arm.Arm, *SoArm101Config]{
			Constructor: newArmSo101Leader,
		},
	)
	resource.RegisterComponent(arm.API, So101Follower,
		resource.Registration[arm.Arm, *SoArm101Config]{
			Constructor: newArmSo101Follower,
		},
	)
}

type SoArm101Config struct {
	// Serial communication settings
	Port     string        `json:"port"`               // Required: Serial port path (e.g., "/dev/ttyUSB0")
	Baudrate int           `json:"baudrate,omitempty"` // Baudrate for SO-ARM servos (default: 1000000)
	Timeout  time.Duration `json:"timeout,omitempty"`  // Communication timeout (default: 5s)

	// Motion parameters
	DefaultSpeed        int `json:"default_speed,omitempty"`        // Default servo speed (1-4094)
	DefaultAcceleration int `json:"default_acceleration,omitempty"` // Default servo acceleration (0-254)

	// Servo configuration
	ServoIDs []int `json:"servo_ids,omitempty"` // Servo IDs for the 5 arm joints (default: [1,2,3,4,5])

	// Leader-Follower configuration
	Mode        string  `json:"mode,omitempty"`         // "leader" or "follower"
	FollowerArm string  `json:"follower_arm,omitempty"` // Name of follower arm (for leader mode)
	LeaderArm   string  `json:"leader_arm,omitempty"`   // Name of leader arm (for follower mode)
	MirrorMode  bool    `json:"mirror_mode,omitempty"`  // Mirror movements horizontally
	ScaleFactor float64 `json:"scale_factor,omitempty"` // Scale factor for movements (default: 1.0)
	SyncRate    int     `json:"sync_rate,omitempty"`    // Sync rate in Hz (default: 20)

	// Internal logger (not from JSON)
	Logger logging.Logger `json:"-"`
}

// Validate ensures all parts of the config are valid
func (cfg *SoArm101Config) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("serial port must be specified")
	}

	// Set defaults
	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000 // Standard SO-ARM baudrate
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
		return nil, nil, fmt.Errorf("expected 5 servo IDs for arm joints, got %d", len(cfg.ServoIDs))
	}
	if cfg.ScaleFactor == 0 {
		cfg.ScaleFactor = 1.0
	}
	if cfg.SyncRate == 0 {
		cfg.SyncRate = 20 // 20 Hz default
	}

	// Validate ranges
	if cfg.DefaultSpeed < 1 || cfg.DefaultSpeed > 4094 {
		return nil, nil, fmt.Errorf("default_speed must be between 1 and 4094, got %d", cfg.DefaultSpeed)
	}
	if cfg.DefaultAcceleration < 0 || cfg.DefaultAcceleration > 254 {
		return nil, nil, fmt.Errorf("default_acceleration must be between 0 and 254, got %d", cfg.DefaultAcceleration)
	}

	// Validate mode
	if cfg.Mode != "" && cfg.Mode != "leader" && cfg.Mode != "follower" {
		return nil, nil, fmt.Errorf("mode must be 'leader' or 'follower', got '%s'", cfg.Mode)
	}

	return nil, nil, nil
}

// Joint limits for SO-101 arm (5 joints) in radians
var so101JointLimits = [][2]float64{
	{-math.Pi, math.Pi},         // Base rotation: ±180°
	{-math.Pi / 2, math.Pi / 2}, // Shoulder: ±90°
	{-math.Pi / 2, math.Pi / 2}, // Elbow: ±90°
	{-math.Pi / 2, math.Pi / 2}, // Wrist pitch: ±90°
	{-math.Pi, math.Pi},         // Wrist roll: ±180°
}

// Create a SO-101 kinematic model
func createSO101Model() (referenceframe.Model, error) {
	// Try to load the embedded SoArm kinematics model (same pattern as RoArm)
	if len(soarmModelJson) > 0 {
		m := &referenceframe.ModelConfigJSON{
			OriginalFile: &referenceframe.ModelFile{
				Bytes:     soarmModelJson,
				Extension: "json",
			},
		}
		err := json.Unmarshal(soarmModelJson, m)
		if err != nil {
			return nil, errors.Wrap(err, "failed to unmarshal json file")
		}

		return m.ParseConfig("soarm_101")
	}

	// If no embedded model, return error since we need a proper kinematic model
	return nil, fmt.Errorf("no embedded soarm_m3.json kinematic model found")
}

func (s *armSo101) Close(context.Context) error {
	s.logger.Info("Closing SO-101 arm")

	// Stop synchronization
	select {
	case s.syncStop <- struct{}{}:
	default:
	}

	s.cancelFunc()

	// Release the shared controller
	ReleaseSharedController()

	return nil
}

// Main arm structure
type armSo101 struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *SoArm101Config
	opMgr      *operation.SingleOperationManager
	controller *SafeSoArmController

	// Motion control
	mu          sync.RWMutex
	moveLock    sync.Mutex
	isMoving    atomic.Bool
	model       referenceframe.Model
	jointLimits [][2]float64

	// Motion parameters
	defaultSpeed int
	defaultAcc   int

	// Leader-Follower mode
	isLeader    bool
	isFollower  bool
	followerArm arm.Arm
	leaderArm   arm.Arm
	syncTicker  *time.Ticker
	syncStop    chan struct{}

	cancelCtx  context.Context
	cancelFunc func()
}

func newArmSo101Leader(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*SoArm101Config](rawConf)
	if err != nil {
		return nil, err
	}
	conf.Mode = "leader"
	conf.Logger = logger
	return NewSo101(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func newArmSo101Follower(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*SoArm101Config](rawConf)
	if err != nil {
		return nil, err
	}
	conf.Mode = "follower"
	conf.Logger = logger
	return NewSo101(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewSo101(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *SoArm101Config, logger logging.Logger) (arm.Arm, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	// Set logger in config if not set
	if conf.Logger == nil {
		conf.Logger = logger
	}

	// Initialize SO-ARM controller using the shared controller manager
	controller, err := GetSharedController(conf)
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("failed to initialize SO-ARM controller: %w", err)
	}

	// Create kinematic model
	model, err := createSO101Model()
	if err != nil {
		ReleaseSharedController() // Clean up on error
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	s := &armSo101{
		name:         name,
		logger:       logger,
		model:        model,
		cfg:          conf,
		opMgr:        operation.NewSingleOperationManager(),
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
	s.mu.RLock()
	defer s.mu.RUnlock()

	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}

	pose, err := referenceframe.ComputeOOBPosition(s.model, inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

func (s *armSo101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if err := motion.MoveArm(ctx, s.logger, s, pose); err != nil {
		return err
	}
	return nil
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
	return s.model, nil
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
		err := s.controller.Ping()
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
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := s.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}
