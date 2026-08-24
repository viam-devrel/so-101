<!--
	Usage: <CalibrationStatusPanel {calibrationState} instruction={readings?.instruction} />
	       <CalibrationStatusPanel {calibrationState} instruction={readings?.instruction} label="State:">
	         {#snippet children()}...recording timer/sample count...{/snippet}
	       </CalibrationStatusPanel>

	The "Current Status:" heading + state pill + instruction block, duplicated 4x with four
	DIFFERENT color maps for the same seven states (see StatusBadge.svelte, now the single
	source). `children` renders to the right of the pill for StepCalibrationRecording's live
	recording indicator; other callers omit it.
-->
<script lang="ts">
	import { StatusBadge } from '$lib/components/ui';
	import type { CalibrationReadings } from '$lib/types';

	interface Props {
		calibrationState: CalibrationReadings['calibration_state'];
		instruction?: string;
		label?: string;
		children?: any;
	}

	let { calibrationState, instruction, label = 'Calibration State:', children }: Props = $props();
</script>

<div class="mb-6">
	<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Status:</h4>
	<div class="bg-gray-50 p-4 rounded-lg">
		<div class="flex items-center justify-between mb-2">
			<div class="flex items-center space-x-4">
				<span class="text-sm font-medium text-gray-700">{label}</span>
				<StatusBadge status={calibrationState} />
			</div>
			{@render children?.()}
		</div>
		{#if instruction}
			<div class="text-sm text-gray-700">
				<span class="font-medium">Instructions:</span>
				{instruction}
			</div>
		{/if}
	</div>
</div>
