package main

import (
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	soArm "so_arm"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	// module.ModularMain(resource.APIModel{arm.API, soArm.So101Leader}, resource.APIModel{arm.API, soArm.So101Follower})
	module.ModularMain(resource.APIModel{arm.API, soArm.SO101Model}, resource.APIModel{gripper.API, soArm.SO101GripperModel})
}
