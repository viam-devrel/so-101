import type { CalibrationJoint, CalibrationReadings, MotorSetupConfig } from '$lib/types';

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

// Full-scale denominator for the live range-recording progress bar. The arm joints' default
// travel is 500..3500 ticks (internal/controller/config.go), so a joint swung from centre to
// one end covers about half of this. ADVISORY_RANGE_TICKS is a "is this range too narrow"
// threshold, not a full span -- using it here filled the bar at half travel.
export const FULL_TRAVEL_TICKS = 3000;

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

// Descending by servo ID (deliberate, per commit bcbf8f1) -- just the ordering, since the three
// call sites render this list differently (a card grid, a progress grid, a summary table).
export function sortedJoints(
	readings: CalibrationReadings | undefined
): [string, CalibrationJoint][] {
	return Object.entries(readings?.joints || {}).sort(([, a], [, b]) => b.id - a.id);
}
