<!--
	Usage: <AbortControl
	         message="Aborting cancels this calibration session and returns the machine to idle."
	         bind:confirmStateFor
	         {calibrationState}
	         {isLoading}
	         onAbort={abortCalibration}
	       />

	The two-click abort confirmation box, previously ~45 duplicated lines (incl. a shared torque
	sentence) between StepCalibrationHoming and StepCalibrationRecording. `confirmStateFor` is
	bindable and scoped to a calibration_state string by the caller: it is set to the state the
	confirmation was opened from, and the caller's own `showAbortConfirm`-equivalent (here,
	internal to this component) collapses the box the instant the state changes underneath the
	user, so a live "Yes, Abort Calibration" can never sit over a panel already moved past.
-->
<script lang="ts">
	import { Button } from '$lib/components/ui';

	interface Props {
		message: string;
		confirmStateFor: string | null;
		calibrationState: string;
		isLoading: boolean;
		onAbort: () => void;
	}

	let {
		message,
		confirmStateFor = $bindable(),
		calibrationState,
		isLoading,
		onAbort
	}: Props = $props();

	const showConfirm = $derived(confirmStateFor === calibrationState);
</script>

<div class="mt-6 pt-4 border-t border-gray-200">
	{#if showConfirm}
		<div class="bg-red-50 border border-red-200 rounded-lg p-4" role="alert">
			<p class="text-red-800 text-sm mb-2">{message}</p>
			<p class="text-red-700 text-xs mb-3">
				The arm stays limp -- holding torque stays off -- so it is safe to keep holding it. The
				wizard stays on this step and then offers to start a new calibration; the step navigation
				above also still works.
			</p>
			<div class="flex justify-center space-x-3">
				<Button variant="secondary" disabled={isLoading} onclick={() => (confirmStateFor = null)}>
					Cancel
				</Button>
				<Button variant="danger" loading={isLoading} onclick={onAbort}>
					{isLoading ? 'Aborting...' : 'Yes, Abort Calibration'}
				</Button>
			</div>
		</div>
	{:else}
		<button
			onclick={() => (confirmStateFor = calibrationState)}
			class="text-sm text-gray-500 hover:text-red-600 underline focus:outline-none focus:ring-2 focus:ring-red-500 rounded"
		>
			Abort Calibration
		</button>
	{/if}
</div>
