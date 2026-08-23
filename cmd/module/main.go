package main

import (
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	rdkdiscovery "go.viam.com/rdk/services/discovery"
	"go.viam.com/rdk/services/generic"

	so101arm "so_arm/components/arm"
	so101calibration "so_arm/components/calibration"
	so101gripper "so_arm/components/gripper"
	so101simulated "so_arm/components/simulated"
	so101discovery "so_arm/services/discovery"
	so101teleop "so_arm/services/teleop"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: so101arm.SO101Model},
		resource.APIModel{API: arm.API, Model: so101simulated.SO101SimulatedModel},
		resource.APIModel{API: gripper.API, Model: so101gripper.SO101GripperModel},
		resource.APIModel{API: gripper.API, Model: so101simulated.SO101SimulatedGripperModel},
		resource.APIModel{API: sensor.API, Model: so101calibration.SO101CalibrationSensorModel},
		resource.APIModel{API: rdkdiscovery.API, Model: so101discovery.SO101DiscoveryModel},
		resource.APIModel{API: generic.API, Model: so101teleop.SO101TeleopModel},
	)
}
