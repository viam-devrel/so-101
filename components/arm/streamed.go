package arm

import (
	"context"
	"errors"

	"go.viam.com/rdk/components/arm"
)

// MoveThroughJointPositionsStreamed is not implemented yet.
func (s *so101) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) error {
	return errors.ErrUnsupported
}
