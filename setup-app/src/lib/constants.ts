import type { CalibrationJoint, MotorSetupConfig } from '$lib/types';

// Single source of truth for motor setup order (reverse assembly order, to avoid ID
// conflicts) -- StepMotorSetup renders this list, wizardProgress derives the name set from it.
export const MOTOR_SETUP_ORDER: MotorSetupConfig[] = [
	{ name: 'gripper', targetId: 6, description: 'Gripper (end effector)' },
	{ name: 'wrist_roll', targetId: 5, description: 'Wrist Roll Joint' },
	{ name: 'wrist_flex', targetId: 4, description: 'Wrist Flex Joint' },
	{ name: 'elbow_flex', targetId: 3, description: 'Elbow Flex Joint' },
	{ name: 'shoulder_lift', targetId: 2, description: 'Shoulder Lift Joint' },
	{ name: 'shoulder_pan', targetId: 1, description: 'Shoulder Pan Joint' }
];

export const MOTOR_SETUP_NAMES = MOTOR_SETUP_ORDER.map((motor) => motor.name);

// UI display heuristic only, not a module contract: components/calibration/workflow.go:264
// rejects a recorded range only when RecordedMin >= RecordedMax (span >= 1 tick is valid).
// A span below this just renders as "narrow" even though the module already accepted it --
// fails safe, never blocks a save the module would allow.
export const ADVISORY_RANGE_TICKS = 1000;

// "No data" is min > max, not a nullish value: the module seeds recorded_min/recorded_max
// with math.MaxInt32/math.MinInt32 and always emits them, so a null check never fires and
// the raw sentinels would reach the screen.
export function hasRecordedRange(joint: CalibrationJoint): boolean {
	return (
		joint.recorded_min != null &&
		joint.recorded_max != null &&
		joint.recorded_min <= joint.recorded_max
	);
}
