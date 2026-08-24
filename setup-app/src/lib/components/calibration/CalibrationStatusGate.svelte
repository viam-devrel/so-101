<!--
	Usage: <CalibrationStatusGate {sensorReadings}>
	         ...the step's own state-branch panels...
	       </CalibrationStatusGate>

	Wraps a calibration step's controls panel, absorbing the isInitialLoad spinner branch and
	the queryError + Retry branch that were ~30 verbatim lines in each of the four calibration
	steps. Renders `children` only once readings have loaded without error.
-->
<script lang="ts">
	import { LoadingSpinner, Button } from '$lib/components/ui';
	import type { StepProps } from '$lib/types';

	interface Props {
		sensorReadings: StepProps['sensorReadings'];
		children?: any;
	}

	let { sensorReadings, children }: Props = $props();

	// isPending, not isLoading: with refetchInterval both are false on background polls, but
	// the query is disabled until the resource client resolves, and only isPending covers that
	// window.
	const isInitialLoad = $derived(sensorReadings.current.isPending);
	const queryError = $derived(sensorReadings.current.error);
</script>

{#if isInitialLoad}
	<div class="text-center">
		<LoadingSpinner size="lg" className="mb-4" />
		<p class="text-gray-600">Loading calibration status...</p>
	</div>
{:else if queryError}
	<div class="text-center" role="alert">
		<p class="text-red-600 mb-4">
			Unable to reach the calibration sensor: {queryError.message}
		</p>
		<Button variant="secondary" onclick={() => sensorReadings.current.refetch()}>Retry</Button>
	</div>
{:else}
	{@render children?.()}
{/if}
