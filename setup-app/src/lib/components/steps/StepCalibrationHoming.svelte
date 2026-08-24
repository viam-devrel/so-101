<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';
	import { canRun } from '$lib/calibrationCommands';
	import { resetCalibrationProgress } from '$lib/calibrationSession';
	import { sortedJoints } from '$lib/constants';
	import { Icon, InfoPanel, Button } from '$lib/components/ui';
	import CalibrationStatusGate from '$lib/components/calibration/CalibrationStatusGate.svelte';
	import CalibrationStatusPanel from '$lib/components/calibration/CalibrationStatusPanel.svelte';
	import AbortControl from '$lib/components/calibration/AbortControl.svelte';
	import AbortedIdlePanel from '$lib/components/calibration/AbortedIdlePanel.svelte';

	let {
		sensorReadings,
		sendCommand,
		setError,
		clearError,
		nextStep,
		goToNamedStep,
		markStepIncomplete,
		setTorqueOutcome
	}: StepProps = $props();

	// Component state
	let isLoading = $state(false);
	let homingSet = $state(false);
	// Which calibration_state an open abort confirmation belongs to; see AbortControl.svelte.
	let confirmStateFor = $state<string | null>(null);
	// True only when THIS component drove the module to idle via abort -- distinguishes
	// "you just aborted" from an idle module reached some other way (fresh load, a reset
	// elsewhere, or a completed save, which unlike abort DOES re-enable torque).
	let justAborted = $state(false);
	// Captured from the state we aborted from: registers are known-wiped once setHomingPosition
	// has run, i.e. every abortable state except 'started'. Not just 'homing_position' -- the
	// unexpected-state branch below also offers abort, and walking Previous back into this step
	// from range recording lands there at 'range_recording', which is past homing.
	let justAbortedAfterHoming = $state(false);
	// Deliberately NOT $state, and deliberately never read by the effect that writes the flags
	// above: readings still report the pre-abort state for up to a poll interval after abort
	// returns (sendCommand does not invalidate the query), so the flags have to survive that
	// window and may only be cleared once the module has actually reported the idle they
	// describe and then moved on.
	let sawIdleSinceAbort = false;
	// Deliberately NOT $state: the effect below reads it, so a reactive write would
	// re-run the effect and its cleanup would clear the timer it just armed.
	let hasAdvanced = false;

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	const joints = $derived(readings?.joints || {});

	// Check if we're in the correct state
	const canSetHoming = $derived(canRun(readings, 'set_homing'));
	const alreadyAtHomingPosition = $derived(calibrationState === 'homing_position');

	// Advance once polled readings confirm the state we caused, not on a blind timer.
	$effect(() => {
		if (homingSet && calibrationState === 'homing_position' && !hasAdvanced) {
			hasAdvanced = true;
			const timer = setTimeout(() => nextStep(), 2000);
			return () => clearTimeout(timer);
		}
	});

	// Un-attribute a stale "you just aborted" once the module has reported that idle and then
	// left it. Writes the flags without reading them, so there is no self-dependency.
	$effect(() => {
		if (calibrationState === 'idle') {
			sawIdleSinceAbort = true;
		} else if (sawIdleSinceAbort) {
			sawIdleSinceAbort = false;
			justAborted = false;
			justAbortedAfterHoming = false;
		}
	});

	// Set homing position
	async function setHomingPosition() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'set_homing' });
			homingSet = true;
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to set homing position');
		} finally {
			isLoading = false;
		}
	}

	// Abort calibration. components/calibration/workflow.go's abortCalibration never re-enables
	// torque -- it cancels the workflow and sets the state to idle -- so the arm stays limp
	// and safe to keep holding; only the recorded homing attempt is discarded.
	async function abortCalibration() {
		const abortedFrom = confirmStateFor;
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'abort' });
			homingSet = false;
			resetCalibrationProgress({ markStepIncomplete, setTorqueOutcome });
			hasAdvanced = false;
			confirmStateFor = null;
			justAborted = true;
			justAbortedAfterHoming = abortedFrom !== 'started';
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to abort calibration');
		} finally {
			isLoading = false;
		}
	}

	// Reset after an error
	async function resetCalibration() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'reset' });
			homingSet = false;
			resetCalibrationProgress({ markStepIncomplete, setTorqueOutcome });
			hasAdvanced = false;
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to reset calibration');
		} finally {
			isLoading = false;
		}
	}
</script>

<div class="max-w-4xl mx-auto">
	<!-- Header -->
	<div class="mb-8">
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Set Homing Position</h3>
		<p class="text-lg text-gray-600">
			Position the arm to the center of its movement range and set this as the homing reference
			point.
		</p>
	</div>

	<!-- Instructions -->
	<InfoPanel tone="info" className="mb-6" icon={null}>
		<h4 class="text-lg font-semibold text-blue-900 mb-4">Positioning Guidelines:</h4>

		<div class="grid md:grid-cols-2 gap-6">
			<div>
				<h5 class="font-medium text-blue-800 mb-3">What is "Center Position"?</h5>
				<ul class="text-blue-700 text-sm space-y-2">
					<li>• <strong>Shoulder Pan:</strong> Facing straight forward (not rotated left/right)</li>
					<li>• <strong>Shoulder Lift:</strong> Upper arm straight up, completely vertical</li>
					<li>• <strong>Elbow:</strong> Bent approximately 90 degrees</li>
					<li>• <strong>Wrist Flex:</strong> Neutral, not bent up or down</li>
					<li>• <strong>Wrist Roll:</strong> Neutral rotation</li>
					<li>• <strong>Gripper:</strong> Closed position</li>
				</ul>
			</div>

			<div>
				<h5 class="font-medium text-blue-800 mb-3">Why This Matters:</h5>
				<ul class="text-blue-700 text-sm space-y-2">
					<li>• Establishes the "zero" reference point for all movements</li>
					<li>• Ensures consistent positioning after power cycles</li>
					<li>• Provides optimal starting point for motion planning</li>
					<li>• Must be roughly in the middle of each joint's range</li>
				</ul>
			</div>
		</div>
	</InfoPanel>

	<CalibrationStatusPanel {calibrationState} instruction={readings?.instruction} />

	<!-- Real-time Joint Positions -->
	{#if Object.keys(joints).length > 0}
		<div class="mb-6">
			<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Joint Positions:</h4>
			<div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
				{#each sortedJoints(readings) as [jointName, joint] (joint.id)}
					<div class="bg-white border border-gray-200 rounded-lg p-4">
						<h5 class="font-medium text-gray-900 mb-2 capitalize">
							{jointName.replace('_', ' ')}
						</h5>
						<div class="text-sm text-gray-600 space-y-1">
							<div class="flex justify-between">
								<span>ID:</span>
								<span class="font-mono">{joint.id}</span>
							</div>
							<div class="flex justify-between">
								<span>Position:</span>
								<span class="font-mono">{joint.current_position}</span>
							</div>
							{#if joint.homing_offset !== undefined}
								<div class="flex justify-between">
									<span>Homing Offset:</span>
									<span class="font-mono">{joint.homing_offset}</span>
								</div>
							{/if}
						</div>
					</div>
				{/each}
			</div>
		</div>
	{/if}

	<!-- Positioning Controls -->
	<div class="bg-white border border-gray-200 rounded-lg p-6 mb-6">
		<CalibrationStatusGate {sensorReadings}>
			{#if canSetHoming}
				<!-- Ready to set homing -->
				<div class="text-center">
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Position the Arm</h4>
					<InfoPanel tone="warning" title="Manual Positioning Required" className="mb-6 text-left">
						<p class="text-sm">
							Manually move each joint to approximately the center of its range of motion. The
							motors are disabled, so you can move the arm freely by hand.
						</p>
					</InfoPanel>

					<p class="text-gray-600 mb-6">
						Once you're satisfied with the arm position, click "Set Homing Position" to record this
						as the reference point.
					</p>

					<div class="flex justify-center space-x-4">
						<Button variant="success" size="lg" loading={isLoading} onclick={setHomingPosition}>
							{#if isLoading}
								Setting Homing Position...
							{:else}
								<Icon name="targetLocation" className="w-5 h-5 mr-3" />
								Set Homing Position
							{/if}
						</Button>
					</div>

					<AbortControl
						message="Aborting cancels this calibration session and returns the machine to idle. You will need to start calibration again from the beginning."
						bind:confirmStateFor
						{calibrationState}
						{isLoading}
						onAbort={abortCalibration}
					/>
				</div>
			{:else if alreadyAtHomingPosition}
				<!-- Homing position already set -->
				<div class="text-center">
					<div
						class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<Icon name="targetLocation" className="w-8 h-8 text-green-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Homing Position Set</h4>
					<p class="text-gray-600 mb-6">
						The homing reference point has been established. The arm is ready for range recording.
					</p>

					<Button variant="primary" size="lg" onclick={nextStep}>
						Continue to Range Recording
						<Icon name="arrowRight" className="ml-2 -mr-1 w-5 h-5" />
					</Button>

					<AbortControl
						message="Aborting discards the homing position you just set and returns the machine to idle. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm."
						bind:confirmStateFor
						{calibrationState}
						{isLoading}
						onAbort={abortCalibration}
					/>
				</div>
			{:else if calibrationState === 'error'}
				<!-- Error state -->
				<div class="text-center">
					<div
						class="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<Icon name="warningTriangle" className="w-8 h-8 text-red-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Error Occurred</h4>
					<p class="text-red-600 mb-6" role="alert">
						{readings?.error || 'An error occurred during homing position setup.'}
					</p>

					<Button
						variant="danger"
						loading={isLoading}
						disabled={!canRun(readings, 'reset')}
						onclick={resetCalibration}
					>
						{isLoading ? 'Resetting...' : 'Reset Calibration'}
					</Button>
				</div>
			{:else if calibrationState === 'idle'}
				<!-- Idle: reached by our own abort (justAborted) or independently (fresh load, a
				     reset elsewhere). Only claim the abort happened when we caused it this session. -->
				<AbortedIdlePanel
					{justAborted}
					{justAbortedAfterHoming}
					notStartedMessage="begin positioning the arm"
					onStartNew={() => goToNamedStep('calibration_start')}
				/>
			{:else}
				<!-- Reached by walking back into this step after the module has moved on (e.g. range
				     recording or completed), or a truly unrecognized state. The status block above already
				     renders the module's instruction, so point at it rather than repeating it, and
				     offer abort when it's actually available. -->
				<div class="text-center">
					<p class="text-gray-600 mb-4">
						This step is not where the arm currently is — the calibration is at
						<span class="font-mono">{calibrationState}</span>. See the status above for what the
						module is waiting on.
					</p>
					<Button variant="secondary" onclick={() => sensorReadings.current.refetch()}>
						Refresh Status
					</Button>
					<!-- 'abort' is only advertised for started/homing_position/range_recording, and the
					     first two have their own branches above, so whatever we abort from here is past
					     homing -- hence the same cleared-limits warning the homing_position branch carries. -->
					{#if canRun(readings, 'abort')}
						<AbortControl
							message="Aborting cancels the current calibration session and returns the machine to idle. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm."
							bind:confirmStateFor
							{calibrationState}
							{isLoading}
							onAbort={abortCalibration}
						/>
					{/if}
				</div>
			{/if}
		</CalibrationStatusGate>
	</div>

	<!-- Success Message -->
	{#if homingSet}
		<InfoPanel tone="success">
			<span class="text-green-900 font-medium">
				Homing position set successfully! Ready to proceed with range recording.
			</span>
		</InfoPanel>
	{/if}
</div>
