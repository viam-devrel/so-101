import type { CalibrationReadings } from '$lib/types';

// Every command name components/calibration/sensor.go's Readings can list in
// available_commands, per its per-state switch. Exhaustive and exported so referencing a
// command that doesn't exist is a compile error, not a silently-always-false gate.
export const CALIBRATION_COMMANDS = [
	'start',
	'set_homing',
	'start_range_recording',
	'stop_range_recording',
	'save_calibration',
	'abort',
	'reset'
] as const;

export type CalibrationCommand = (typeof CALIBRATION_COMMANDS)[number];

// True when the module's current readings list `command` as available right now. The module,
// not calibration_state string equality, decides what the user may do.
export function canRun(
	readings: CalibrationReadings | undefined,
	command: CalibrationCommand
): boolean {
	return !!readings?.available_commands?.includes(command);
}
