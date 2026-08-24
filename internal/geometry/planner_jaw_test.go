package geometry

import (
	"context"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// jawGoalConfig is the arm configuration whose forward kinematics defines the planning goal.
// Deriving the goal from FK guarantees it is reachable; hand-picked poses are not.
var jawGoalConfig = []float64{0.8, -1.0, 1.0, 0.6, 0.3}

// TestPlannerJawTravel is the experiment this whole change exists to run.
//
// The jaw is a DoF that changes collision geometry but has NO effect on the TCP, so nothing in
// the goal metric constrains it. Reading armplanning predicts the planner hands it FULL range:
// computeJointSensitivities (linearized_frame_system.go:78) computes
//
//	thisRatio := startDistance / math.Abs(myDistance-startDistance)
//
// which is INVERSE sensitivity -- a joint that does not move the TCP divides by zero, yields
// +Inf, and clampSensitivities floors it at 1.0, the joint's entire range. rdk does this on
// purpose ("a small change has a small effect -> take bigger steps").
//
// What that means for the EMITTED trajectory, after RRT and smoothing, is not determinable by
// reading the code. That is what this measures. It asserts only structural facts and t.Logf's
// the magnitudes; once characterised, tighten the reported numbers into a real assertion.
func TestPlannerJawTravel(t *testing.T) {
	if testing.Short() {
		t.Skip("motion planning is slow")
	}
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	armModel, err := ArmModelJSON("arm")
	require.NoError(t, err)
	gripperModel, err := BuildGripperModel(FollowerGripper, LowDetail, "gripper", true)
	require.NoError(t, err)

	armLink, err := (&referenceframe.LinkConfig{ID: "arm", Parent: referenceframe.World}).ParseConfig()
	require.NoError(t, err)
	gripperLink, err := (&referenceframe.LinkConfig{ID: "gripper", Parent: "arm"}).ParseConfig()
	require.NoError(t, err)

	fs, err := referenceframe.NewFrameSystem("test", []*referenceframe.FrameSystemPart{
		{FrameConfig: armLink, ModelFrame: armModel},
		{FrameConfig: gripperLink, ModelFrame: gripperModel},
	}, nil)
	require.NoError(t, err)

	// Goal by forward kinematics, so it is reachable by construction.
	fkInputs := referenceframe.NewZeroInputs(fs)
	fkInputs["arm"] = jawGoalConfig
	tf, err := fs.Transform(fkInputs.ToLinearInputs(),
		referenceframe.NewPoseInFrame("gripper", spatialmath.NewZeroPose()), referenceframe.World)
	require.NoError(t, err)
	goal := referenceframe.NewPoseInFrame(referenceframe.World, tf.(*referenceframe.PoseInFrame).Pose())
	t.Logf("FK-derived goal: %v", goal.Pose().Point())

	// A slab across the direct path, forcing the RRT to actually search.
	wall, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 290, Y: -78, Z: 169}),
		r3.Vector{X: 90, Y: 90, Z: 260}, "wall")
	require.NoError(t, err)

	fullRange := GripperJointMax - GripperJointMin

	for _, sc := range []struct {
		name      string
		obstacles *referenceframe.GeometriesInFrame
	}{
		{"direct", nil},
		{"obstructed", referenceframe.NewGeometriesInFrame(
			referenceframe.World, []spatialmath.Geometry{wall})},
	} {
		t.Run(sc.name, func(t *testing.T) {
			planned := 0
			for seed := 0; seed < 10; seed++ {
				opts := armplanning.NewBasicPlannerOptions()
				opts.RandomSeed = seed

				plan, _, err := armplanning.PlanMotion(ctx, logger, &armplanning.PlanRequest{
					FrameSystem:           fs,
					StartState:            armplanning.NewPlanState(nil, referenceframe.NewZeroInputs(fs)),
					Goals:                 []*armplanning.PlanState{armplanning.NewPlanState(referenceframe.FrameSystemPoses{"gripper": goal}, nil)},
					ObstaclesInWorldFrame: sc.obstacles,
					PlannerOptions:        opts,
				})
				if err != nil {
					t.Logf("seed %d: planning failed: %v", seed, err)
					continue
				}
				planned++

				traj := plan.Trajectory()
				require.NotEmpty(t, traj)

				var travel, prev, minT, maxT float64
				for i, step := range traj {
					jaw, ok := step["gripper"]
					require.Truef(t, ok, "seed %d step %d: the trajectory has no gripper column", seed, i)
					require.Len(t, jaw, 1)

					// Structural assertion: the planner must respect the joint limits.
					assert.GreaterOrEqualf(t, jaw[0], GripperJointMin, "seed %d step %d below the jaw limit", seed, i)
					assert.LessOrEqualf(t, jaw[0], GripperJointMax, "seed %d step %d above the jaw limit", seed, i)

					if i == 0 {
						prev, minT, maxT = jaw[0], jaw[0], jaw[0]
						continue
					}
					travel += math.Abs(jaw[0] - prev)
					prev = jaw[0]
					minT, maxT = math.Min(minT, jaw[0]), math.Max(maxT, jaw[0])
				}

				t.Logf("seed %2d: %3d steps | jaw travel %.4f rad (%.1f%% of range) | span [%.4f, %.4f] | net %.4f",
					seed, len(traj), travel, 100*travel/fullRange, minT, maxT, traj[len(traj)-1]["gripper"][0]-traj[0]["gripper"][0])
			}

			// Without this the whole experiment can pass green having measured nothing.
			require.Positivef(t, planned, "no seed produced a trajectory in the %q scenario", sc.name)
		})
	}
}
