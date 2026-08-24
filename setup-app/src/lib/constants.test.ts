import { describe, it, expect } from 'vitest';
import {
	ADVISORY_RANGE_TICKS,
	MOTOR_SETUP_NAMES,
	MOTOR_SETUP_ORDER,
	hasRecordedRange
} from './constants';
import { RECORDED_MAX_SENTINEL, RECORDED_MIN_SENTINEL } from './testing/fakeCalibrationSensor';
import type { CalibrationJoint } from './types';

function joint(overrides: Partial<CalibrationJoint> = {}): CalibrationJoint {
	return {
		id: 1,
		name: 'shoulder_pan',
		current_position: 0,
		homing_offset: 0,
		recorded_min: RECORDED_MIN_SENTINEL,
		recorded_max: RECORDED_MAX_SENTINEL,
		is_completed: false,
		...overrides
	};
}

describe('hasRecordedRange', () => {
	it('is false for the module-seeded MaxInt32/MinInt32 sentinels', () => {
		expect(hasRecordedRange(joint())).toBe(false);
	});

	it('is true for a min/max of tick 0 -- falsy zero must not read as missing', () => {
		expect(hasRecordedRange(joint({ recorded_min: 0, recorded_max: 0 }))).toBe(true);
	});

	it('is true for any min <= max, regardless of span width', () => {
		expect(hasRecordedRange(joint({ recorded_min: 0, recorded_max: 1 }))).toBe(true);
	});

	it('is false when min > max', () => {
		expect(hasRecordedRange(joint({ recorded_min: 100, recorded_max: 50 }))).toBe(false);
	});

	it('is false for a null recorded_min even though the type claims number', () => {
		expect(hasRecordedRange(joint({ recorded_min: null as unknown as number }))).toBe(false);
	});

	it('is false for an undefined recorded_max even though the type claims number', () => {
		expect(hasRecordedRange(joint({ recorded_max: undefined as unknown as number }))).toBe(false);
	});
});

describe('ADVISORY_RANGE_TICKS', () => {
	it('is advisory only: a span narrower than it is still a valid recorded range', () => {
		const narrow = joint({ recorded_min: 0, recorded_max: ADVISORY_RANGE_TICKS - 1 });
		expect(hasRecordedRange(narrow)).toBe(true);
	});

	it('a span of exactly the module boundary (min === max - 1) is still valid', () => {
		// workflow.go's stopRangeRecording only rejects RecordedMin >= RecordedMax; a span of 1
		// tick is the smallest the module itself accepts.
		expect(hasRecordedRange(joint({ recorded_min: 10, recorded_max: 11 }))).toBe(true);
	});
});

describe('MOTOR_SETUP_NAMES', () => {
	it('is derived 1:1 from MOTOR_SETUP_ORDER, same order', () => {
		expect(MOTOR_SETUP_NAMES).toEqual(MOTOR_SETUP_ORDER.map((m) => m.name));
	});
});
