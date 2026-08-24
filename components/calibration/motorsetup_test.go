package calibration

import (
	"context"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// expectedMotorNames are motorSetupVerify's six SO-101 servo names, independent of test
// naming above so a typo in one doesn't silently agree with the other.
var expectedMotorNames = []string{
	"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper",
}

// motorsAsMap is a small helper: motorSetupVerify returns "motors" as map[string]any, but
// each entry is itself a map[string]any built by the function under test, not a typed struct.
func motorsAsMap(t *testing.T, resp map[string]any) map[string]any {
	t.Helper()
	motors, ok := resp["motors"].(map[string]any)
	require.True(t, ok, "motors must be a map[string]any, got %T", resp["motors"])
	return motors
}

// TestMotorSetupVerifySuccessAllHealthy covers the happy path: every servo pings and its
// reported model number resolves, so all_ok should agree with success.
func TestMotorSetupVerifySuccessAllHealthy(t *testing.T) {
	cs := testRecordingSensor(t, testfake.NewFakeTransport())

	resp, err := cs.motorSetupVerify(context.Background())
	require.NoError(t, err)

	assert.Equal(t, true, resp["success"], "success reports the call completed")
	assert.Equal(t, true, resp["all_ok"], "all_ok reports the aggregate motor verdict")

	motors := motorsAsMap(t, resp)
	require.Len(t, motors, len(expectedMotorNames), "all six expected motors must be reported")
	for _, name := range expectedMotorNames {
		entry, ok := motors[name].(map[string]any)
		require.True(t, ok, "motor %q missing from response", name)
		assert.Equal(t, "ok", entry["status"], "motor %q", name)
	}
}

// TestMotorSetupVerifySuccessTrueDespiteMotorFailure is the regression test for the bug
// described in motorsetup.go: motor_setup_verify used to fold the per-motor verdict into
// `success`, so the app's sendCommand (which throws on success:false) made a partial motor
// failure unreachable as data. success must stay true here even though the bus is fully
// unplugged and every motor fails to respond; all_ok is where the failure belongs.
//
// It also pins the "not_responding vs model_detection_failed" classification: verify used to
// call DetectServoModel after PingServo had already succeeded, and DetectModel re-pings
// (feetech.Servo.DetectModel calls bus.Ping again), so a servo that stopped answering between
// the two pings was reported as a model problem rather than a comm failure. Classifying from
// PingServo's own returned model number removes the second round trip, so a failed ping is
// always not_responding and only an unregistered model number is model_detection_failed
// (below).
func TestMotorSetupVerifySuccessTrueDespiteMotorFailure(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.Unplug()
	cs := testRecordingSensor(t, ft)

	resp, err := cs.motorSetupVerify(context.Background())
	require.NoError(t, err)

	assert.Equal(t, true, resp["success"], "success must report call completion, not motor health")
	assert.Equal(t, false, resp["all_ok"], "all_ok must reflect the unhealthy bus")

	motors := motorsAsMap(t, resp)
	require.Len(t, motors, len(expectedMotorNames), "all six expected motors must still be reported")
	for _, name := range expectedMotorNames {
		entry, ok := motors[name].(map[string]any)
		require.True(t, ok, "motor %q missing from response", name)
		assert.Equal(t, "not_responding", entry["status"], "motor %q", name)
	}
}

// TestMotorSetupVerifyUnknownModelWarnsWithoutFailingAggregate exercises the
// model_detection_failed path: the servo answers its ping but reports a model number the
// driver's registry doesn't know. Per the deliberate split documented at both call sites,
// motor_setup_verify treats this as a non-blocking warning (unlike discoverOneMotor, which
// rejects it outright), so all_ok must stay true and the status string must surface the count.
func TestMotorSetupVerifyUnknownModelWarnsWithoutFailingAggregate(t *testing.T) {
	ft := testfake.NewFakeTransport()
	const unknownModelNumber = 65535
	ft.SetRegister(1, feetech.RegModelNumber.Address, testfake.EncodeWordLE(unknownModelNumber))
	cs := testRecordingSensor(t, ft)

	resp, err := cs.motorSetupVerify(context.Background())
	require.NoError(t, err)

	assert.Equal(t, true, resp["success"])
	assert.Equal(t, true, resp["all_ok"], "an unrecognized model must not fail the aggregate")

	motors := motorsAsMap(t, resp)
	require.Len(t, motors, len(expectedMotorNames))

	shoulderPan, ok := motors["shoulder_pan"].(map[string]any)
	require.True(t, ok)
	assert.Equal(t, "model_detection_failed", shoulderPan["status"])
	assert.NotEmpty(t, shoulderPan["error"])

	for _, name := range expectedMotorNames {
		if name == "shoulder_pan" {
			continue
		}
		entry, ok := motors[name].(map[string]any)
		require.True(t, ok, "motor %q missing from response", name)
		assert.Equal(t, "ok", entry["status"], "motor %q", name)
	}

	status, ok := resp["status"].(string)
	require.True(t, ok)
	assert.Contains(t, status, "1", "status should surface the warning count")
}
