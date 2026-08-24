<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';
	import { canRun } from '$lib/calibrationCommands';

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

	const CALIBRATION_STEPS = [
		'calibration_start',
		'calibration_homing',
		'calibration_recording',
		'calibration_save'
	] as const;

	// Component state
	let isLoading = $state(false);
	let homingSet = $state(false);
	// Which calibration_state an open abort confirmation belongs to. Scoping it to the state
	// collapses the box as soon as the state changes underneath the user, so a live
	// "Yes, Abort Calibration" can never sit over a panel they have since moved past.
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
	// Deliberately NOT $state, and deliberately never read by the effect that writes the
	// flags above: readings still report the pre-abort state for up to a poll interval after
	// abort returns (sendCommand does not invalidate the query), so the flags have to survive
	// that window and may only be cleared once the module has actually reported the idle they
	// describe and then moved on.
	let sawIdleSinceAbort = false;
	// Deliberately NOT $state: the effect below reads it, so a reactive write would
	// re-run the effect and its cleanup would clear the timer it just armed.
	let hasAdvanced = false;

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	const joints = $derived(readings?.joints || {});
	// isPending, not isLoading: with refetchInterval both are false on background polls,
	// but the query is disabled until the resource client resolves, and only isPending
	// covers that window -- isLoading there is false, so the panel fell through to
	// "Unexpected calibration state: unknown".
	const isInitialLoad = $derived(sensorReadings.current.isPending);
	const queryError = $derived(sensorReadings.current.error);

	// Check if we're in the correct state
	const canSetHoming = $derived(canRun(readings, 'set_homing'));
	const alreadyAtHomingPosition = $derived(calibrationState === 'homing_position');
	const showAbortConfirm = $derived(confirmStateFor === calibrationState);

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
			// abort/reset invalidate a prior run's save: torque is off again and the save
			// step must re-earn its gate. See StepCalibrationSave.resetCalibration.
			setTorqueOutcome(null);
			for (const step of CALIBRATION_STEPS) markStepIncomplete(step);
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
			// abort/reset invalidate a prior run's save: torque is off again and the save
			// step must re-earn its gate. See StepCalibrationSave.resetCalibration.
			setTorqueOutcome(null);
			for (const step of CALIBRATION_STEPS) markStepIncomplete(step);
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
	<div class="bg-blue-50 p-6 rounded-lg mb-6">
		<h4 class="text-lg font-semibold text-blue-900 mb-4">Positioning Guidelines:</h4>

		<div class="grid md:grid-cols-2 gap-6">
			<div>
				<h5 class="font-medium text-blue-800 mb-3">What is "Center Position"?</h5>
				<ul class="text-blue-700 text-sm space-y-2">
					<li>• <strong>Shoulder Pan:</strong> Facing straight forward (not rotated left/right)</li>
					<li>• <strong>Shoulder Lift:</strong> Horizontal or slightly elevated</li>
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
	</div>

	<!-- Current Status -->
	<div class="mb-6">
		<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Status:</h4>
		<div class="bg-gray-50 p-4 rounded-lg">
			<div class="flex items-center space-x-4 mb-2">
				<span class="text-sm font-medium text-gray-700">Calibration State:</span>
				<span
					class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium {calibrationState ===
					'started'
						? 'bg-yellow-100 text-yellow-800'
						: calibrationState === 'homing_position'
							? 'bg-green-100 text-green-800'
							: calibrationState === 'error'
								? 'bg-red-100 text-red-800'
								: 'bg-gray-100 text-gray-800'}"
				>
					{calibrationState}
				</span>
			</div>
			{#if readings?.instruction}
				<div class="text-sm text-gray-700">
					<span class="font-medium">Instructions:</span>
					{readings.instruction}
				</div>
			{/if}
		</div>
	</div>

	<!-- Real-time Joint Positions -->
	{#if Object.keys(joints).length > 0}
		<div class="mb-6">
			<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Joint Positions:</h4>
			<div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
				{#each Object.entries(joints).sort(([, a], [, b]) => b.id - a.id) as [jointName, joint] (joint.id)}
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

	<!-- Abort control: shared by every branch the module accepts 'abort' from. Two clicks
	     required (reveal, then confirm) since it discards in-progress work. -->
	{#snippet abortControl(message: string)}
		<div class="mt-6 pt-4 border-t border-gray-200">
			{#if showAbortConfirm}
				<div class="bg-red-50 border border-red-200 rounded-lg p-4">
					<p class="text-red-800 text-sm mb-2">{message}</p>
					<p class="text-red-700 text-xs mb-3">
						The arm stays limp -- holding torque stays off -- so it is safe to keep holding it. The
						wizard stays on this step and then offers to start a new calibration; the step
						navigation above also still works.
					</p>
					<div class="flex justify-center space-x-3">
						<button
							onclick={() => (confirmStateFor = null)}
							disabled={isLoading}
							class="px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-gray-500 disabled:opacity-50"
						>
							Cancel
						</button>
						<button
							onclick={abortCalibration}
							disabled={isLoading}
							class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-red-600 hover:bg-red-700 focus:outline-none focus:ring-2 focus:ring-red-500 disabled:opacity-50"
						>
							{#if isLoading}
								<div class="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
								Aborting...
							{:else}
								Yes, Abort Calibration
							{/if}
						</button>
					</div>
				</div>
			{:else}
				<button
					onclick={() => (confirmStateFor = calibrationState)}
					class="text-sm text-gray-500 hover:text-red-600 underline"
				>
					Abort Calibration
				</button>
			{/if}
		</div>
	{/snippet}

	<!-- Positioning Controls -->
	<div class="bg-white border border-gray-200 rounded-lg p-6 mb-6">
		{#if isInitialLoad}
			<!-- First load: no readings yet -->
			<div class="text-center">
				<div
					class="animate-spin rounded-full h-10 w-10 border-b-2 border-gray-400 mx-auto mb-4"
				></div>
				<p class="text-gray-600">Loading calibration status...</p>
			</div>
		{:else if queryError}
			<!-- Connection error, distinct from an unexpected calibration state -->
			<div class="text-center">
				<p class="text-red-600 mb-4">
					Unable to reach the calibration sensor: {queryError.message}
				</p>
				<button
					onclick={() => sensorReadings.current.refetch()}
					class="px-4 py-2 bg-gray-600 text-white rounded-md hover:bg-gray-700 focus:outline-none focus:ring-2 focus:ring-gray-500"
				>
					Retry
				</button>
			</div>
		{:else if canSetHoming}
			<!-- Ready to set homing -->
			<div class="text-center">
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Position the Arm</h4>
				<div class="bg-yellow-50 p-4 rounded-lg mb-6">
					<div class="flex items-start">
						<div class="flex-shrink-0">
							<svg
								class="h-6 w-6 text-yellow-600"
								fill="none"
								stroke="currentColor"
								viewBox="0 0 24 24"
							>
								<path
									stroke-linecap="round"
									stroke-linejoin="round"
									stroke-width="2"
									d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"
								></path>
							</svg>
						</div>
						<div class="ml-3">
							<h5 class="font-medium text-yellow-900 mb-2">Manual Positioning Required</h5>
							<p class="text-yellow-800 text-sm">
								Manually move each joint to approximately the center of its range of motion. The
								motors are disabled, so you can move the arm freely by hand.
							</p>
						</div>
					</div>
				</div>

				<p class="text-gray-600 mb-6">
					Once you're satisfied with the arm position, click "Set Homing Position" to record this as
					the reference point.
				</p>

				<div class="flex justify-center space-x-4">
					<button
						onclick={setHomingPosition}
						disabled={isLoading}
						class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500 disabled:opacity-50 disabled:cursor-not-allowed"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
							Setting Homing Position...
						{:else}
							<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
								<path
									stroke-linecap="round"
									stroke-linejoin="round"
									stroke-width="2"
									d="M17.657 16.657L13.414 20.9a1.998 1.998 0 01-2.827 0l-4.244-4.243a8 8 0 1111.314 0z"
								></path>
								<path
									stroke-linecap="round"
									stroke-linejoin="round"
									stroke-width="2"
									d="M15 11a3 3 0 11-6 0 3 3 0 016 0z"
								></path>
							</svg>
							Set Homing Position
						{/if}
					</button>
				</div>

				{@render abortControl(
					'Aborting cancels this calibration session and returns the machine to idle. You will need to start calibration again from the beginning.'
				)}
			</div>
		{:else if alreadyAtHomingPosition}
			<!-- Homing position already set -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<svg class="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M17.657 16.657L13.414 20.9a1.998 1.998 0 01-2.827 0l-4.244-4.243a8 8 0 1111.314 0z"
						></path>
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M15 11a3 3 0 11-6 0 3 3 0 016 0z"
						></path>
					</svg>
				</div>
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Homing Position Set</h4>
				<p class="text-gray-600 mb-6">
					The homing reference point has been established. The arm is ready for range recording.
				</p>

				<button
					onclick={nextStep}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
				>
					Continue to Range Recording
					<svg class="ml-2 -mr-1 w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
							clip-rule="evenodd"
						/>
					</svg>
				</button>

				{@render abortControl(
					'Aborting discards the homing position you just set and returns the machine to idle. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm.'
				)}
			</div>
		{:else if calibrationState === 'error'}
			<!-- Error state -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<svg class="w-8 h-8 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"
						></path>
					</svg>
				</div>
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Error Occurred</h4>
				<p class="text-red-600 mb-6">
					{readings?.error || 'An error occurred during homing position setup.'}
				</p>

				<button
					onclick={resetCalibration}
					disabled={isLoading || !canRun(readings, 'reset')}
					class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-red-600 hover:bg-red-700 focus:outline-none focus:ring-2 focus:ring-red-500 disabled:opacity-50"
				>
					{#if isLoading}
						<div class="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
						Resetting...
					{:else}
						Reset Calibration
					{/if}
				</button>
			</div>
		{:else if calibrationState === 'idle'}
			<!-- Idle: reached by our own abort (justAborted) or independently (fresh load, a
			     reset elsewhere). Only claim the abort happened when we caused it this session. -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-gray-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<svg class="w-8 h-8 text-gray-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
						></path>
					</svg>
				</div>
				<h4 class="text-xl font-semibold text-gray-900 mb-4">
					{justAborted ? 'Calibration Aborted' : 'Calibration Not Started'}
				</h4>
				<p class="text-gray-600 mb-6">
					{#if justAborted}
						Calibration was aborted. The arm is limp (holding torque is off) -- it's safe to keep
						holding, but it will not hold its position on its own if you let go.
						{#if justAbortedAfterHoming}
							Setting the homing position already cleared the position limits stored in the servos,
							so a full calibration is needed before relying on the arm.
						{/if}
					{:else}
						No calibration is currently in progress. Start calibration from the previous step to
						begin positioning the arm.
					{/if}
				</p>

				<button
					onclick={() => goToNamedStep('calibration_start')}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
				>
					Start New Calibration
				</button>
			</div>
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
				<button
					onclick={() => sensorReadings.current.refetch()}
					class="px-4 py-2 bg-gray-600 text-white rounded-md hover:bg-gray-700 focus:outline-none focus:ring-2 focus:ring-gray-500"
				>
					Refresh Status
				</button>
				<!-- 'abort' is only advertised for started/homing_position/range_recording, and the
				     first two have their own branches above, so whatever we abort from here is past
				     homing -- hence the same cleared-limits warning the homing_position branch carries. -->
				{#if canRun(readings, 'abort')}
					{@render abortControl(
						'Aborting cancels the current calibration session and returns the machine to idle. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm.'
					)}
				{/if}
			</div>
		{/if}
	</div>

	<!-- Success Message -->
	{#if homingSet}
		<div class="bg-green-50 p-4 rounded-lg">
			<div class="flex items-center">
				<svg class="w-5 h-5 text-green-600 mr-3" fill="currentColor" viewBox="0 0 20 20">
					<path
						fill-rule="evenodd"
						d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
						clip-rule="evenodd"
					/>
				</svg>
				<span class="text-green-900 font-medium">
					Homing position set successfully! Ready to proceed with range recording.
				</span>
			</div>
		</div>
	{/if}
</div>
