package main

import (
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/discovery"
	soArm "so_arm"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: soArm.SO101Model},
		resource.APIModel{API: gripper.API, Model: soArm.SO101GripperModel},
		resource.APIModel{API: sensor.API, Model: soArm.SO101CalibrationSensorModel},
		resource.APIModel{API: discovery.API, Model: soArm.SO101DiscoveryModel},
	)
}
