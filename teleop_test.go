package so_arm

import (
	"testing"

	"go.viam.com/test"
)

func TestTeleopConfigValidate(t *testing.T) {
	t.Run("requires leader_arm and follower_arm", func(t *testing.T) {
		_, _, err := (&SO101TeleopConfig{FollowerArm: "f"}).Validate("p")
		test.That(t, err, test.ShouldNotBeNil)
		_, _, err = (&SO101TeleopConfig{LeaderArm: "l"}).Validate("p")
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("rejects negative rate", func(t *testing.T) {
		cfg := &SO101TeleopConfig{LeaderArm: "l", FollowerArm: "f", RateHz: -1}
		_, _, err := cfg.Validate("p")
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("deps are arms plus configured grippers only", func(t *testing.T) {
		cfg := &SO101TeleopConfig{LeaderArm: "l", FollowerArm: "f"}
		req, _, err := cfg.Validate("p")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req, test.ShouldResemble, []string{"l", "f"})

		cfg2 := &SO101TeleopConfig{
			LeaderArm: "l", FollowerArm: "f",
			LeaderGripper: "lg", FollowerGripper: "fg",
		}
		req2, _, err := cfg2.Validate("p")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req2, test.ShouldResemble, []string{"l", "f", "lg", "fg"})
	})
}
