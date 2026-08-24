import { describe, it, expect } from 'vitest';
import { canAdvanceFromStep } from './wizardProgress';
import { MOTOR_SETUP_NAMES } from './constants';
import { FakeCalibrationSensor } from './testing/fakeCalibrationSensor';
import type { CalibrationReadings, MotorSetupResult, WorkflowStep } from './types';

const STEPS: WorkflowStep[] = [
	'overview',
	'motor_setup',
	'motor_verify',
	'calibration_start',
	'calibration_homing',
	'calibration_recording',
	'calibration_save',
	'complete'
];

function noCompletion(): Record<WorkflowStep, boolean> {
	return Object.fromEntries(STEPS.map((s) => [s, false])) as Record<WorkflowStep, boolean>;
}

function completionWith(step: WorkflowStep): Record<WorkflowStep, boolean> {
	return { ...noCompletion(), [step]: true };
}

function readingsAt(state: CalibrationReadings['calibration_state']): CalibrationReadings {
	const fake = new FakeCalibrationSensor();
	fake.forceState(state);
	return fake.getReadings();
}

function motorResults(
	step: 'configured' | 'skipped' | 'discovered' | undefined
): Record<string, MotorSetupResult> {
	if (!step) return {};
	const results: Record<string, MotorSetupResult> = {};
	for (const name of MOTOR_SETUP_NAMES) {
		results[name] =
			step === 'skipped'
				? { motor_name: name, step: 'skipped', success: true }
				: {
						motor_name: name,
						step,
						current_id: 1,
						target_id: 1,
						model: 'sts3215',
						found_baudrate: 1000000,
						success: true
					};
	}
	return results;
}

describe('canAdvanceFromStep: overview', () => {
	it('is always advanceable -- sends no commands', () => {
		expect(canAdvanceFromStep('overview', undefined, {}, noCompletion())).toBe(true);
		expect(canAdvanceFromStep('overview', readingsAt('error'), {}, noCompletion())).toBe(true);
	});
});

describe('canAdvanceFromStep: motor_setup', () => {
	it('is false until every motor is configured or skipped', () => {
		expect(canAdvanceFromStep('motor_setup', undefined, {}, noCompletion())).toBe(false);
		expect(
			canAdvanceFromStep('motor_setup', undefined, motorResults('discovered'), noCompletion())
		).toBe(false);
	});

	it('is true once every motor is configured', () => {
		expect(
			canAdvanceFromStep('motor_setup', undefined, motorResults('configured'), noCompletion())
		).toBe(true);
	});

	it('is true once every motor is skipped', () => {
		expect(
			canAdvanceFromStep('motor_setup', undefined, motorResults('skipped'), noCompletion())
		).toBe(true);
	});

	it('is true for a mix of configured and skipped motors', () => {
		const results = motorResults('configured');
		results[MOTOR_SETUP_NAMES[0]] = {
			motor_name: MOTOR_SETUP_NAMES[0],
			step: 'skipped',
			success: true
		};
		expect(canAdvanceFromStep('motor_setup', undefined, results, noCompletion())).toBe(true);
	});

	it('is false if even one motor is only discovered, not configured/skipped', () => {
		const results = motorResults('configured');
		results[MOTOR_SETUP_NAMES[0]] = {
			motor_name: MOTOR_SETUP_NAMES[0],
			step: 'discovered',
			current_id: 1,
			target_id: 1,
			model: 'sts3215',
			found_baudrate: 1000000,
			success: true
		};
		expect(canAdvanceFromStep('motor_setup', undefined, results, noCompletion())).toBe(false);
	});
});

describe('canAdvanceFromStep: motor_verify', () => {
	it('has no readings-derived signal -- always false without stepCompletion', () => {
		for (const state of ['idle', 'started', 'completed', 'error'] as const) {
			expect(canAdvanceFromStep('motor_verify', readingsAt(state), {}, noCompletion())).toBe(false);
		}
		expect(canAdvanceFromStep('motor_verify', undefined, {}, noCompletion())).toBe(false);
	});

	it('is true once the wizard records the step complete', () => {
		expect(canAdvanceFromStep('motor_verify', undefined, {}, completionWith('motor_verify'))).toBe(
			true
		);
	});
});

describe('canAdvanceFromStep: calibration_start (atLeast started)', () => {
	it('is false before started', () => {
		expect(canAdvanceFromStep('calibration_start', readingsAt('idle'), {}, noCompletion())).toBe(
			false
		);
		expect(canAdvanceFromStep('calibration_start', undefined, {}, noCompletion())).toBe(false);
	});

	it('is true at or past started', () => {
		for (const state of ['started', 'homing_position', 'range_recording', 'completed'] as const) {
			expect(canAdvanceFromStep('calibration_start', readingsAt(state), {}, noCompletion())).toBe(
				true
			);
		}
	});

	it('is false in error -- error is deliberately off the progress line', () => {
		expect(canAdvanceFromStep('calibration_start', readingsAt('error'), {}, noCompletion())).toBe(
			false
		);
	});
});

describe('canAdvanceFromStep: calibration_homing (atLeast homing_position)', () => {
	it('is false before homing_position', () => {
		for (const state of ['idle', 'started'] as const) {
			expect(canAdvanceFromStep('calibration_homing', readingsAt(state), {}, noCompletion())).toBe(
				false
			);
		}
	});

	it('is true at or past homing_position', () => {
		for (const state of ['homing_position', 'range_recording', 'completed'] as const) {
			expect(canAdvanceFromStep('calibration_homing', readingsAt(state), {}, noCompletion())).toBe(
				true
			);
		}
	});

	// This is the ordering rule the module is being refactored around: a module that has
	// progressed to 'completed' must still let a user standing on an earlier step (having
	// walked Previous) advance forward again. An equality check on 'homing_position' would
	// wrongly read 'completed' as unfinished and dead-end backward navigation.
	it('a completed module still allows Next from calibration_homing (not exact-equality)', () => {
		expect(
			canAdvanceFromStep('calibration_homing', readingsAt('completed'), {}, noCompletion())
		).toBe(true);
	});
});

describe('canAdvanceFromStep: calibration_recording (atLeast completed)', () => {
	it('is false before completed', () => {
		for (const state of ['idle', 'started', 'homing_position', 'range_recording'] as const) {
			expect(
				canAdvanceFromStep('calibration_recording', readingsAt(state), {}, noCompletion())
			).toBe(false);
		}
	});

	it('is true only once completed', () => {
		expect(
			canAdvanceFromStep('calibration_recording', readingsAt('completed'), {}, noCompletion())
		).toBe(true);
	});
});

describe('canAdvanceFromStep: calibration_save', () => {
	it('has no readings-derived signal -- always false without stepCompletion, even completed', () => {
		expect(
			canAdvanceFromStep('calibration_save', readingsAt('completed'), {}, noCompletion())
		).toBe(false);
	});

	it('is true once the wizard records the step complete', () => {
		expect(
			canAdvanceFromStep(
				'calibration_save',
				readingsAt('idle'),
				{},
				completionWith('calibration_save')
			)
		).toBe(true);
	});
});

describe('canAdvanceFromStep: complete', () => {
	it('is always advanceable (BaseWizard disables Next by index here regardless)', () => {
		expect(canAdvanceFromStep('complete', readingsAt('idle'), {}, noCompletion())).toBe(true);
	});
});

describe('canAdvanceFromStep: stepCompletion OR', () => {
	it('an explicitly-completed step advances regardless of contradicting readings', () => {
		expect(
			canAdvanceFromStep(
				'calibration_start',
				readingsAt('idle'),
				{},
				completionWith('calibration_start')
			)
		).toBe(true);
	});
});

describe('canAdvanceFromStep: permissive default for an unrecognized step', () => {
	it('is true for a step name outside the CAN_ADVANCE map', () => {
		const bogusStep = 'bogus_step' as WorkflowStep;
		expect(canAdvanceFromStep(bogusStep, readingsAt('idle'), {}, noCompletion())).toBe(true);
	});
});
