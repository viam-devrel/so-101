<!--
	Usage: <StatusBadge status={calibrationState} />

	One calibration-state pill, replacing four inline ternary chains (Homing/Recording/Save/
	Start) that disagreed on every state but `error`. Colours: gray for the states where
	nothing is happening (idle, unknown), yellow while something is actively running (started,
	range_recording), blue for the transitional homing_position, green for completed, red for
	error.
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
