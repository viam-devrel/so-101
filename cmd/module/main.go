package main

import (
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/discovery"
	"go.viam.com/rdk/services/generic"
	soArm "so_arm"
	so101arm "so_arm/components/arm"
	so101gripper "so_arm/components/gripper"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: so101arm.SO101Model},
		resource.APIModel{API: arm.API, Model: soArm.SO101SimulatedModel},
		resource.APIModel{API: gripper.API, Model: so101gripper.SO101GripperModel},
		resource.APIModel{API: gripper.API, Model: soArm.SO101SimulatedGripperModel},
		resource.APIModel{API: sensor.API, Model: soArm.SO101CalibrationSensorModel},
		resource.APIModel{API: discovery.API, Model: soArm.SO101DiscoveryModel},
		resource.APIModel{API: generic.API, Model: soArm.SO101TeleopModel},
	)
}
