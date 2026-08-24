<!--
	Usage: <AbortedIdlePanel
	         {justAborted}
	         {justAbortedAfterHoming}
	         notStartedMessage="begin positioning the arm"
	         onStartNew={() => goToNamedStep('calibration_start')}
	       />

	The idle-state panel duplicated between StepCalibrationHoming and StepCalibrationRecording:
	"Calibration Aborted" vs "Calibration Not Started", carrying the justAbortedAfterHoming
	cleared-position-limits warning in both. `notStartedMessage` completes "...to
	{notStartedMessage}." in the not-started copy, the one line that legitimately differs
	between the two callers.
-->
<script lang="ts">
	import { Icon, Button } from '$lib/components/ui';

	interface Props {
		justAborted: boolean;
		justAbortedAfterHoming: boolean;
		notStartedMessage: string;
		onStartNew: () => void;
	}

	let { justAborted, justAbortedAfterHoming, notStartedMessage, onStartNew }: Props = $props();
</script>

<div class="text-center">
	<div class="w-16 h-16 bg-gray-100 rounded-full flex items-center justify-center mx-auto mb-4">
		<Icon name="infoCircle" className="w-8 h-8 text-gray-500" />
	</div>
	<h4 class="text-xl font-semibold text-gray-900 mb-4">
		{justAborted ? 'Calibration Aborted' : 'Calibration Not Started'}
	</h4>
	<p class="text-gray-600 mb-6">
		{#if justAborted}
			Calibration was aborted. The arm is limp (holding torque is off) -- it's safe to keep holding,
			but it will not hold its position on its own if you let go.
			{#if justAbortedAfterHoming}
				Setting the homing position already cleared the position limits stored in the servos, so a
				full calibration is needed before relying on the arm.
			{/if}
		{:else}
			No calibration is currently in progress. Start calibration from the previous step to
			{notStartedMessage}.
		{/if}
	</p>

	<Button variant="primary" size="lg" onclick={onStartNew}>Start New Calibration</Button>
</div>
