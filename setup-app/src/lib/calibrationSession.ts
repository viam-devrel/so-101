import type { TorqueOutcome, WorkflowStep } from '$lib/types';

// The four steps a calibration run touches, in order. Declared once here -- was duplicated
// identically in all four calibration step components.
export const CALIBRATION_STEPS = [
	'calibration_start',
	'calibration_homing',
	'calibration_recording',
	'calibration_save'
] as const satisfies readonly WorkflowStep[];

// abort/reset invalidate a prior run's save: torque is off again and the save step must
// re-earn its gate. Called from every abort/reset handler across the four calibration steps
// (not StepCalibrationSave's save-failure catch, which un-completes only calibration_save).
export function resetCalibrationProgress({
	markStepIncomplete,
	setTorqueOutcome
}: {
	markStepIncomplete: (step: WorkflowStep) => void;
	setTorqueOutcome: (outcome: TorqueOutcome | null) => void;
}): void {
	setTorqueOutcome(null);
	for (const step of CALIBRATION_STEPS) markStepIncomplete(step);
}
