import { describe, it, expect } from 'vitest';
import { CALIBRATION_COMMANDS, canRun } from './calibrationCommands';
import { FakeCalibrationSensor, type FakeCalibrationState } from './testing/fakeCalibrationSensor';

// The six states sensor.go's Readings switch actually assigns during the workflow.
const REACHABLE_STATES: FakeCalibrationState[] = [
	'idle',
	'started',
	'homing_position',
	'range_recording',
	'completed',
	'error'
];

// 'unknown' is CalibrationState.String()'s default branch, never assigned by the state
// machine -- but it is part of the type, and the switch's no-match case (empty commands) is
// worth pinning too.
const ALL_STATES: FakeCalibrationState[] = [...REACHABLE_STATES, 'unknown'];

describe('canRun', () => {
	it('matches the fake sensor available_commands for every state x command pair', () => {
		for (const state of ALL_STATES) {
			const fake = new FakeCalibrationSensor();
			fake.forceState(state);
			const readings = fake.getReadings();
			for (const command of CALIBRATION_COMMANDS) {
				const expected = readings.available_commands.includes(command);
				expect(canRun(readings, command), `state=${state} command=${command}`).toBe(expected);
			}
		}
	});

	it('never offers a command outside the state its own state permits', () => {
		// Restated the other way round from the matrix above: for every state, the set of
		// commands canRun accepts is exactly the module's advertised set -- no more.
		for (const state of REACHABLE_STATES) {
			const fake = new FakeCalibrationSensor();
			fake.forceState(state);
			const readings = fake.getReadings();
			const permitted = CALIBRATION_COMMANDS.filter((c) => canRun(readings, c));
			expect(permitted.sort()).toEqual([...readings.available_commands].sort());
		}
	});

	it('advertises reset in every reachable state', () => {
		for (const state of REACHABLE_STATES) {
			const fake = new FakeCalibrationSensor();
			fake.forceState(state);
			expect(canRun(fake.getReadings(), 'reset')).toBe(true);
		}
	});

	it('is false for every command with no readings at all', () => {
		for (const command of CALIBRATION_COMMANDS) {
			expect(canRun(undefined, command)).toBe(false);
		}
	});

	it('save_calibration is only offered at completed', () => {
		for (const state of REACHABLE_STATES) {
			const fake = new FakeCalibrationSensor();
			fake.forceState(state);
			expect(canRun(fake.getReadings(), 'save_calibration')).toBe(state === 'completed');
		}
	});

	it('abort is offered only mid-workflow: started, homing_position, range_recording', () => {
		for (const state of REACHABLE_STATES) {
			const fake = new FakeCalibrationSensor();
			fake.forceState(state);
			const expected =
				state === 'started' || state === 'homing_position' || state === 'range_recording';
			expect(canRun(fake.getReadings(), 'abort'), state).toBe(expected);
		}
	});
});
