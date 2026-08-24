import { MOTOR_SETUP_NAMES } from './constants';
import type { CalibrationReadings, MotorSetupResult, WorkflowStep, WorkflowType } from './types';

// Per-workflow step list + display titles, one place for what used to be duplicated
// (byte-for-byte, apart from these two tables) across CalibrationWizard/MotorSetupWizard/
// FullSetupWizard. Lives here rather than in a new file because this module is already the
// per-step source of truth (canAdvanceFromStep below) -- a step name that isn't in a
// workflow's list can't be reached, and stepTitles is a straight lookup keyed by the same
// WorkflowStep names CAN_ADVANCE is keyed by.
export interface WorkflowDefinition {
	steps: WorkflowStep[];
	stepTitles: Record<string, string>;
}

export const WORKFLOW_DEFINITIONS: Record<WorkflowType, WorkflowDefinition> = {
	'motor-setup': {
		steps: ['overview', 'motor_setup', 'motor_verify', 'complete'],
		stepTitles: {
			overview: 'Overview & Safety',
			motor_setup: 'Motor Setup',
			motor_verify: 'Motor Verification',
			complete: 'Setup Complete'
		}
	},
	calibration: {
		steps: [
			'overview',
			'calibration_start',
			'calibration_homing',
			'calibration_recording',
			'calibration_save',
			'complete'
		],
		stepTitles: {
			overview: 'Overview & Safety',
			calibration_start: 'Start Calibration',
			calibration_homing: 'Set Homing Position',
			calibration_recording: 'Record Ranges',
			calibration_save: 'Save Calibration',
			complete: 'Calibration Complete'
		}
	},
	'full-setup': {
		steps: [
			'overview',
			'motor_setup',
			'motor_verify',
			'calibration_start',
			'calibration_homing',
			'calibration_recording',
			'calibration_save',
			'complete'
		],
		stepTitles: {
			overview: 'Overview & Safety',
			motor_setup: 'Motor Setup',
			motor_verify: 'Motor Verification',
			calibration_start: 'Start Calibration',
			calibration_homing: 'Set Homing Position',
			calibration_recording: 'Record Ranges',
			calibration_save: 'Save Calibration',
			complete: 'Setup Complete'
		}
	}
};

// The module's linear calibration progression (workflow.go). 'error' is deliberately absent:
// it is off the line, so every calibration gate closes there and the step's own Reset button
// is the way out.
const CALIBRATION_PROGRESS: CalibrationReadings['calibration_state'][] = [
	'idle',
	'started',
	'homing_position',
	'range_recording',
	'completed'
];

// Gates on "the module is at or past this point", not "exactly at it". Equality would trap a
// user who walks backwards: Previous from Save to Record Ranges leaves the module at
// 'completed', which an equality check on 'homing_position' would read as unfinished, and the
// step body only offers Refresh Status -- the recorded ranges would be unreachable without
// redoing the whole calibration. Forward gating is unchanged, since a fresh run only ever
// arrives at a state from below.
function atLeast(
	readings: CalibrationReadings | undefined,
	required: CalibrationReadings['calibration_state']
): boolean {
	const at = CALIBRATION_PROGRESS.indexOf(readings?.calibration_state as never);
	return at >= 0 && at >= CALIBRATION_PROGRESS.indexOf(required);
}

interface CanAdvanceArgs {
	readings: CalibrationReadings | undefined;
	motorSetupResults: Record<string, MotorSetupResult>;
}

// Whether the footer "Next" button may leave a given step, from readings/motorSetupResults
// alone. A step with no such signal returns false here and instead relies entirely on
// canAdvanceFromStep's stepCompletion check below -- an unrecognized step name is the one
// case that stays permissive, since trapping the user there is worse than an early click
// through.
const CAN_ADVANCE: Record<WorkflowStep, (args: CanAdvanceArgs) => boolean> = {
	// Sends no commands; nothing to gate.
	overview: () => true,

	// Mirrors StepMotorSetup's `allMotorsProcessed`: every motor either configured or
	// deliberately skipped. `step` now survives the skip itself (see MotorSetupResult), so
	// this can observe a partial run the same way the step's own escape hatch does.
	motor_setup: ({ motorSetupResults }) =>
		MOTOR_SETUP_NAMES.every((name) => {
			const step = motorSetupResults[name]?.step;
			return step === 'configured' || step === 'skipped';
		}),

	// No readings-derived signal exists (readings.motor_setup carries only a free-text
	// `status` and a step index). Completion is observable only via the wizard's
	// stepCompletion record, ORed in by canAdvanceFromStep below.
	motor_verify: () => false,

	// StepCalibrationStart's `alreadyStarted`, widened to anything past it.
	calibration_start: ({ readings }) => atLeast(readings, 'started'),

	// StepCalibrationHoming's `alreadyAtHomingPosition`, widened to anything past it.
	calibration_homing: ({ readings }) => atLeast(readings, 'homing_position'),

	// StepCalibrationRecording's `isCompleted` ('completed' is the last state on the line).
	calibration_recording: ({ readings }) => atLeast(readings, 'completed'),

	// No readings-derived signal distinguishes "saved" from "started over" (see
	// StepCalibrationSave: abort/reset/save all leave calibration_state at 'idle' or
	// 'completed' without a durable success marker). Completion is observable only via
	// stepCompletion, ORed in by canAdvanceFromStep below.
	calibration_save: () => false,

	// Terminal step; BaseWizard already disables Next by index here regardless of this value.
	complete: () => true
};

export function canAdvanceFromStep(
	step: WorkflowStep,
	readings: CalibrationReadings | undefined,
	motorSetupResults: Record<string, MotorSetupResult>,
	stepCompletion: Record<WorkflowStep, boolean>
): boolean {
	// An explicitly-completed step is always advanceable, regardless of what readings say --
	// this is what lets motor_verify/calibration_save (no readings signal) gate at all while
	// still covering a fresh run of any step that does have one.
	if (stepCompletion[step]) return true;
	// Permissive on an unrecognized step name: the map is exhaustive over WorkflowStep, so
	// this only fires if a caller passes something off-type, and blocking Next is the worse
	// failure.
	const predicate = CAN_ADVANCE[step];
	return predicate === undefined || predicate({ readings, motorSetupResults });
}
