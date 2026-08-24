<!--
	Usage: <StatusBadge status={calibrationState} />

	Consolidates the calibration-state pill duplicated in StepCalibrationHoming,
	StepCalibrationRecording, StepCalibrationSave, and StepCalibrationStart -- each of those
	implements the same seven-state `CalibrationReadings['calibration_state']` union with its own
	inline ternary chain, and no two agree (see below). All four share the exact pill markup this
	renders (`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium`).

	Discrepancies this one map replaces (file -> color per state; "-" means that file's fallback
	color, since it doesn't name the state explicitly):
	                   Homing    Recording  Save     Start
	  idle             - (gray)  - (gray)   blue     gray
	  started          yellow    - (gray)   - (gray) yellow
	  homing_position  green     blue       - (gray) - (blue)
	  range_recording  - (gray)  yellow     - (gray) - (blue)
	  completed        - (gray)  green      green    - (blue)
	  error            red       red        red      red        <- the one state all four agreed on
	  unknown          - (gray)  - (gray)   - (gray) - (blue)

	Chosen colors below: gray for the two "nothing happening yet" states (idle, unknown), yellow
	for the two "actively doing something" states (started, range_recording), blue for the
	transitional homing_position, green for completed, red for error.
-->
<script lang="ts">
	import type { CalibrationReadings } from '$lib/types';

	type CalibrationState = CalibrationReadings['calibration_state'];

	interface Props {
		status: CalibrationState;
		className?: string;
	}

	let { status, className = '' }: Props = $props();

	const STATUS_COLORS: Record<CalibrationState, string> = {
		idle: 'bg-gray-100 text-gray-800',
		started: 'bg-yellow-100 text-yellow-800',
		homing_position: 'bg-blue-100 text-blue-800',
		range_recording: 'bg-yellow-100 text-yellow-800',
		completed: 'bg-green-100 text-green-800',
		error: 'bg-red-100 text-red-800',
		unknown: 'bg-gray-100 text-gray-800'
	};

	const colorClass = $derived(STATUS_COLORS[status]);
</script>

<span
	class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium {colorClass} {className}"
>
	{status}
</span>
