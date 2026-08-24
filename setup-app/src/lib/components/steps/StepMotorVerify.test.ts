// @vitest-environment jsdom
import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/svelte';
import StepMotorVerify from './StepMotorVerify.svelte';
import { FakeCalibrationSensor } from '$lib/testing/fakeCalibrationSensor';
import type { StepProps } from '$lib/types';

// Only the fields StepMotorVerify actually reads need to be real; the rest of StepProps is
// wide (sensor client/query/mutation generics from the Viam SDK) and irrelevant here, so it's
// stubbed and cast rather than fully constructed.
function makeProps(overrides: Partial<StepProps> = {}): StepProps {
	const base = {
		sensorClient: { current: undefined },
		sensorReadings: { current: undefined },
		doCommand: { current: undefined },
		sendCommand: vi.fn(),
		error: null,
		setError: vi.fn(),
		clearError: vi.fn(),
		nextStep: vi.fn(),
		prevStep: vi.fn(),
		goToStep: vi.fn(),
		goToNamedStep: vi.fn(),
		motorSetupResults: {},
		setMotorSetupResults: vi.fn(),
		updateMotorSetupResult: vi.fn(),
		clearMotorSetupResult: vi.fn(),
		markStepComplete: vi.fn(),
		markStepIncomplete: vi.fn(),
		stepCompletion: {
			overview: false,
			motor_setup: false,
			motor_verify: false,
			calibration_start: false,
			calibration_homing: false,
			calibration_recording: false,
			calibration_save: false,
			complete: false
		},
		torqueOutcome: null,
		setTorqueOutcome: vi.fn()
	};
	return { ...base, ...overrides } as unknown as StepProps;
}

async function clickVerify() {
	await fireEvent.click(screen.getByRole('button', { name: /verify all motors/i }));
}

describe('StepMotorVerify', () => {
	it('call failed: the verify command itself throws -- no grid, error surfaces, bus scan offered', async () => {
		const setError = vi.fn();
		const sendCommand = vi.fn().mockRejectedValue(new Error('Communication failed.'));
		render(StepMotorVerify, {
			props: makeProps({ sendCommand, setError, error: 'Communication failed.' })
		});

		await clickVerify();

		await waitFor(() => expect(setError).toHaveBeenCalled());
		// No results grid: the call never resolved, so nothing to render per-motor.
		expect(screen.queryByText(/verification results/i)).toBeNull();
		// error prop was already truthy when rendered -- with no results, the bus scan escape
		// hatch must be offered.
		expect(screen.getByRole('button', { name: /scan.*bus/i })).toBeTruthy();
	});

	it('succeeded with issues: motor_setup_verify resolves (never throws) even with a failing motor -- this is the fix', async () => {
		const fake = new FakeCalibrationSensor();
		fake.setMotorResult('shoulder_pan', { id: 1, status: 'not_responding', error: 'timeout' });
		const markStepComplete = vi.fn();
		const setError = vi.fn();

		render(StepMotorVerify, {
			props: makeProps({
				sendCommand: fake.sendCommand.bind(fake),
				markStepComplete,
				setError
			})
		});

		await clickVerify();

		// The regression this guards: a partial motor failure must resolve into the grid, not
		// throw and leave verificationResults null forever.
		await waitFor(() => expect(screen.getByText(/verification results/i)).toBeTruthy());
		expect(screen.getByText(/issues found/i)).toBeTruthy();
		expect(setError).not.toHaveBeenCalled();
		expect(markStepComplete).not.toHaveBeenCalledWith('motor_verify');
	});

	it('all OK: every motor responds -- grid shows success and unlocks Next', async () => {
		const fake = new FakeCalibrationSensor();
		for (const name of [
			'shoulder_pan',
			'shoulder_lift',
			'elbow_flex',
			'wrist_flex',
			'wrist_roll',
			'gripper'
		]) {
			fake.setMotorResult(name, { id: 1, status: 'ok', model: 'sts3215' });
		}
		const markStepComplete = vi.fn();

		render(StepMotorVerify, {
			props: makeProps({ sendCommand: fake.sendCommand.bind(fake), markStepComplete })
		});

		await clickVerify();

		await waitFor(() => expect(screen.getByText(/all motors ok/i)).toBeTruthy());
		expect(screen.getByRole('button', { name: /continue/i })).toBeTruthy();
		expect(markStepComplete).toHaveBeenCalledWith('motor_verify');
	});
});
