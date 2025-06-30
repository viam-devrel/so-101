package arm

import (
	"context"
	"errors"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

var (
	So101            = resource.NewModel("devrel", "arm", "so-101")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterComponent(arm.API, So101,
		resource.Registration[arm.Arm, *Config]{
			Constructor: newArmSo101,
		},
	)
}

type Config struct {
	/*
		Put config attributes here. There should be public/exported fields
		with a `json` parameter at the end of each attribute.

		Example config struct:
			type Config struct {
				Pin   string `json:"pin"`
				Board string `json:"board"`
				MinDeg *float64 `json:"min_angle_deg,omitempty"`
			}

		If your model does not need a config, replace *Config in the init
		function with resource.NoNativeConfig
	*/
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, error) {
	// Add config validation code here
	return nil, nil
}

type armSo101 struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()
}

func newArmSo101(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewSo101(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewSo101(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (arm.Arm, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &armSo101{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *armSo101) Name() resource.Name {
	return s.name
}

func (s *armSo101) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (arm.Arm, error) {
	panic("not implemented")
}

func (s *armSo101) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	panic("not implemented")
}

func (s *armSo101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	panic("not implemented")
}

func (s *armSo101) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	panic("not implemented")
}

func (s *armSo101) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	panic("not implemented")
}

func (s *armSo101) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	panic("not implemented")
}

func (s *armSo101) Stop(ctx context.Context, extra map[string]interface{}) error {
	panic("not implemented")
}

func (s *armSo101) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	panic("not implemented")
}

func (s *armSo101) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	panic("not implemented")
}

func (s *armSo101) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	panic("not implemented")
}

func (s *armSo101) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	panic("not implemented")
}

func (s *armSo101) IsMoving(ctx context.Context) (bool, error) {
	panic("not implemented")
}

func (s *armSo101) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	panic("not implemented")
}

func (s *armSo101) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
