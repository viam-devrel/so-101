// @vitest-environment jsdom
import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/svelte';
import StepCalibrationSave from './StepCalibrationSave.svelte';
import { FakeCalibrationSensor } from '$lib/testing/fakeCalibrationSensor';
import type { CalibrationReadings, StepProps, WorkflowStep } from '$lib/types';

const emptyCompletion: Record<WorkflowStep, boolean> = {
	overview: false,
	motor_setup: false,
	motor_verify: false,
	calibration_start: false,
	calibration_homing: false,
	calibration_recording: false,
	calibration_save: false,
	complete: false
};

function readingsResult(
	data: CalibrationReadings | undefined,
	opts: { isPending?: boolean; error?: Error | null } = {}
) {
	return {
		current: {
			data,
			isPending: opts.isPending ?? false,
			error: opts.error ?? null,
			refetch: vi.fn()
		}
	};
}

// Only the fields StepCalibrationSave actually reads need to be real; the rest of StepProps
// (sensor client/mutation generics) is irrelevant here, so it's stubbed and cast. `overrides`
// is deliberately untyped against StepProps -- readingsResult()'s fake QueryObserverResult
// doesn't structurally match the real (huge) TanStack type, and the whole object is cast at
// the return below anyway.
function makeProps(overrides: Record<string, unknown> = {}): StepProps {
	const fake = new FakeCalibrationSensor();
	const base = {
		sensorReadings: readingsResult(fake.getReadings()),
		sendCommand: fake.sendCommand.bind(fake),
		error: null,
		setError: vi.fn(),
		clearError: vi.fn(),
		nextStep: vi.fn(),
		goToNamedStep: vi.fn(),
		motorSetupResults: {},
		updateMotorSetupResult: vi.fn(),
		clearMotorSetupResult: vi.fn(),
		markStepComplete: vi.fn(),
		markStepIncomplete: vi.fn(),
		stepCompletion: { ...emptyCompletion },
		torqueOutcome: null,
		setTorqueOutcome: vi.fn()
	};
	return { ...base, ...overrides } as unknown as StepProps;
}

describe('StepCalibrationSave panel matrix', () => {
	it('loading: isPending shows the loading panel, not "unexpected state"', () => {
		render(StepCalibrationSave, {
			props: makeProps({ sensorReadings: readingsResult(undefined, { isPending: true }) })
		});
		expect(screen.getByText(/loading calibration status/i)).toBeTruthy();
	});

	it('connection error: query error is distinct from an unexpected calibration state', () => {
		render(StepCalibrationSave, {
			props: makeProps({
				sensorReadings: readingsResult(undefined, { error: new Error('conn refused') })
			})
		});
		expect(screen.getByText(/unable to reach the calibration sensor: conn refused/i)).toBeTruthy();
	});

	it('ready to save: completed state offers Save, and a successful save reports torque_enabled', async () => {
		const fake = new FakeCalibrationSensor();
		fake.forceState('completed');
		const setTorqueOutcome = vi.fn();
		const markStepComplete = vi.fn();

		render(StepCalibrationSave, {
			props: makeProps({
				sensorReadings: readingsResult(fake.getReadings()),
				sendCommand: fake.sendCommand.bind(fake),
				setTorqueOutcome,
				markStepComplete
			})
		});

		expect(screen.getByText(/ready to save calibration/i)).toBeTruthy();
		await fireEvent.click(screen.getByRole('button', { name: /save calibration data/i }));

		await waitFor(() => expect(markStepComplete).toHaveBeenCalledWith('calibration_save'));
		expect(setTorqueOutcome).toHaveBeenCalledWith({ enabled: true, error: undefined });
		expect(screen.getByText(/calibration saved successfully/i)).toBeTruthy();
	});

	it('save attempt failed: stays retryable, offers Retry Save and Start Over', async () => {
		const fake = new FakeCalibrationSensor();
		fake.forceState('completed');
		fake.queueFailure('save_calibration', 'servo bus timeout');
		const setError = vi.fn();
		const markStepIncomplete = vi.fn();

		render(StepCalibrationSave, {
			props: makeProps({
				sensorReadings: readingsResult(fake.getReadings()),
				sendCommand: fake.sendCommand.bind(fake),
				setError,
				markStepIncomplete
			})
		});

		await fireEvent.click(screen.getByRole('button', { name: /save calibration data/i }));

		await waitFor(() => expect(screen.getByText(/save attempt failed/i)).toBeTruthy());
		expect(setError).toHaveBeenCalled();
		expect(markStepIncomplete).toHaveBeenCalledWith('calibration_save');
		expect(screen.getByRole('button', { name: /retry save/i })).toBeTruthy();
		expect(screen.getByRole('button', { name: /start over instead/i })).toBeTruthy();
		// State never moved to 'completed' -> 'error': saveCalibration's failure keeps it
		// StateCompleted, per workflow.go.
		expect(fake.getState()).toBe('completed');
	});

	it('genuine error state: only Reset is offered, and it returns to calibration_start', async () => {
		const fake = new FakeCalibrationSensor();
		fake.forceState('error', 'bus disconnected');
		const goToNamedStep = vi.fn();

		render(StepCalibrationSave, {
			props: makeProps({
				sensorReadings: readingsResult(fake.getReadings()),
				sendCommand: fake.sendCommand.bind(fake),
				goToNamedStep
			})
		});

		expect(screen.getByText(/save error/i)).toBeTruthy();
		expect(screen.queryByRole('button', { name: /retry save/i })).toBeNull();

		await fireEvent.click(screen.getByRole('button', { name: /reset calibration/i }));
		await waitFor(() => expect(goToNamedStep).toHaveBeenCalledWith('calibration_start'));
	});

	it('post-remount, saved with torque left disabled: the arm-limp panel wins even with no local save just ran', () => {
		const fake = new FakeCalibrationSensor();
		fake.forceState('idle'); // saveCalibration's terminal state
		render(StepCalibrationSave, {
			props: makeProps({
				sensorReadings: readingsResult(fake.getReadings()),
				stepCompletion: { ...emptyCompletion, calibration_save: true },
				torqueOutcome: { enabled: false, error: 'linkage jammed' }
			})
		});

		expect(screen.getByText(/arm is not holding position/i)).toBeTruthy();
		expect(screen.getByText(/linkage jammed/i)).toBeTruthy();
		expect(screen.getByRole('button', { name: /continue anyway/i })).toBeTruthy();
	});

	it('post-remount, saved successfully: the plain success panel renders from stepCompletion alone', () => {
		const fake = new FakeCalibrationSensor();
		fake.forceState('idle');
		render(StepCalibrationSave, {
			props: makeProps({
				sensorReadings: readingsResult(fake.getReadings()),
				stepCompletion: { ...emptyCompletion, calibration_save: true },
				torqueOutcome: { enabled: true }
			})
		});

		expect(screen.getByText(/calibration saved successfully/i)).toBeTruthy();
		expect(screen.getByRole('button', { name: /complete setup/i })).toBeTruthy();
	});

	it('fallback: idle with nothing recorded offers Refresh and Start New, not a dead end', () => {
		const fake = new FakeCalibrationSensor();
		fake.forceState('idle');
		render(StepCalibrationSave, {
			props: makeProps({ sensorReadings: readingsResult(fake.getReadings()) })
		});

		expect(screen.getByText(/unexpected calibration state: idle/i)).toBeTruthy();
		expect(screen.getByRole('button', { name: /refresh status/i })).toBeTruthy();
		expect(screen.getByRole('button', { name: /start new calibration/i })).toBeTruthy();
	});
});
