<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';
	import { ADVISORY_RANGE_TICKS, hasRecordedRange } from '$lib/constants';
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
	let recordingCompleted = $state(false);
	// Which calibration_state an open abort confirmation belongs to. Scoping it to the state
	// collapses the box as soon as the state changes underneath the user, so a live
	// "Yes, Abort Calibration" can never sit over a panel they have since moved past.
	let confirmStateFor = $state<string | null>(null);
	// True only when THIS component drove the module to idle via abort -- distinguishes
	// "you just aborted" from an idle module reached some other way (fresh load, a reset
	// elsewhere, or a completed save, which unlike abort DOES re-enable torque).
	let justAborted = $state(false);
	// Captured from the state we aborted from: registers are known-wiped once setHomingPosition
	// has run, i.e. every abortable state except 'started'. The two main branches here
	// (homing_position, range_recording) are always past homing, but the unexpected-state branch
	// below also offers abort and is reached at 'started', where homing has not run.
	let justAbortedAfterHoming = $state(false);
	// Deliberately NOT $state, and deliberately never read by the effect that writes the flag
	// above: readings still report the pre-abort state for up to a poll interval after abort
	// returns (sendCommand does not invalidate the query), so the flag has to survive that
	// window and may only be cleared once the module has actually reported the idle it
	// describes and then moved on.
	let sawIdleSinceAbort = false;
	// Deliberately NOT $state: the effect below reads it, so a reactive write would
	// re-run the effect and its cleanup would clear the timer it just armed.
	let hasAdvanced = false;

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	const joints = $derived(readings?.joints || {});
	const recordingTime = $derived(readings?.recording_time_seconds || 0);
	const positionSamples = $derived(readings?.position_samples || 0);
	// isPending, not isLoading: with refetchInterval both are false on background polls,
	// but the query is disabled until the resource client resolves, and only isPending
	// covers that window -- isLoading there is false, so the panel fell through to
	// "Unexpected calibration state: unknown".
	const isInitialLoad = $derived(sensorReadings.current.isPending);
	const queryError = $derived(sensorReadings.current.error);

	// Check states
	const canStartRecording = $derived(canRun(readings, 'start_range_recording'));
	const isRecording = $derived(calibrationState === 'range_recording');
	const isCompleted = $derived(calibrationState === 'completed');
	const isError = $derived(calibrationState === 'error');
	const showAbortConfirm = $derived(confirmStateFor === calibrationState);

	// Advance once polled readings confirm the state we caused, not on a blind timer.
	$effect(() => {
		if (recordingCompleted && calibrationState === 'completed' && !hasAdvanced) {
			hasAdvanced = true;
			const timer = setTimeout(() => nextStep(), 2000);
			return () => clearTimeout(timer);
		}
	});

	// Un-attribute a stale "you just aborted" once the module has reported that idle and then
	// left it. Writes the flag without reading it, so there is no self-dependency.
	$effect(() => {
		if (calibrationState === 'idle') {
			sawIdleSinceAbort = true;
		} else if (sawIdleSinceAbort) {
			sawIdleSinceAbort = false;
			justAborted = false;
			justAbortedAfterHoming = false;
		}
	});

	// Calculate completion statistics
	const totalJoints = $derived(Object.keys(joints).length);
	const completedJoints = $derived(
		Object.values(joints).filter((joint) => joint.is_completed).length
	);
	const allJointsCompleted = $derived(totalJoints > 0 && completedJoints === totalJoints);

	// Format time display
	const formattedTime = $derived(
		`${Math.floor(recordingTime / 60)}:${Math.floor(recordingTime % 60)
			.toString()
			.padStart(2, '0')}`
	);

	// Start range recording
	async function startRecording() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'start_range_recording' });
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to start recording');
		} finally {
			isLoading = false;
		}
	}

	// Stop range recording
	async function stopRecording() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'stop_range_recording' });
			recordingCompleted = true;
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to stop recording');
		} finally {
			isLoading = false;
		}
	}

	// Abort calibration. components/calibration/workflow.go's abortCalibration never re-enables
	// torque -- it cancels the recording goroutine (if running) and sets the state to idle --
	// so the arm stays limp and safe to keep holding; only the homing/recording work is discarded.
	async function abortCalibration() {
		const abortedFrom = confirmStateFor;
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'abort' });
			recordingCompleted = false;
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
			recordingCompleted = false;
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

	// Get joint progress bar width
	function getJointProgressWidth(joint: any): number {
		if (joint.is_completed) return 100;
		if (!hasRecordedRange(joint)) return 0;
		const range = joint.recorded_max - joint.recorded_min;
		return Math.min(100, (range / ADVISORY_RANGE_TICKS) * 100);
	}

	// Get joint status color: completed (module-decided), in progress, or no data yet
	function getJointStatusColor(joint: any): string {
		if (joint.is_completed) return 'bg-green-100 border-green-300 text-green-800';
		if (hasRecordedRange(joint)) return 'bg-blue-100 border-blue-300 text-blue-800';
		return 'bg-gray-100 border-gray-300 text-gray-600';
	}
</script>

<div class="max-w-4xl mx-auto">
	<!-- Header -->
	<div class="mb-8">
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Record Range of Motion</h3>
		<p class="text-lg text-gray-600">
			Move each joint through its full range of motion to record the mechanical limits.
		</p>
	</div>

	<!-- Instructions -->
	<div class="bg-blue-50 p-6 rounded-lg mb-6">
		<h4 class="text-lg font-semibold text-blue-900 mb-4">Recording Process:</h4>

		<div class="grid md:grid-cols-2 gap-6">
			<div>
				<h5 class="font-medium text-blue-800 mb-3">What to Do:</h5>
				<ul class="text-blue-700 text-sm space-y-2">
					<li>• Click "Start Recording" to begin data collection</li>
					<li>• Slowly move EACH joint to its maximum limits</li>
					<li>• Move to the furthest safe position in both directions</li>
					<li>• Watch the progress indicators for each joint</li>
					<li>• Continue until all joints show "completed"</li>
					<li>• Click "Stop Recording" when finished</li>
				</ul>
			</div>

			<div>
				<h5 class="font-medium text-blue-800 mb-3">Important Tips:</h5>
				<ul class="text-blue-700 text-sm space-y-2">
					<li>• Move joints <strong>slowly and smoothly</strong></li>
					<li>• Don't force joints beyond mechanical stops</li>
					<li>• Each joint needs full range coverage to complete</li>
					<li>• Recording typically takes 2-5 minutes</li>
					<li>• The arm will remain limp during recording</li>
				</ul>
			</div>
		</div>
	</div>

	<!-- Current Status -->
	<div class="mb-6">
		<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Status:</h4>
		<div class="bg-gray-50 p-4 rounded-lg">
			<div class="flex items-center justify-between mb-2">
				<div class="flex items-center space-x-4">
					<span class="text-sm font-medium text-gray-700">State:</span>
					<span
						class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium {calibrationState ===
						'homing_position'
							? 'bg-blue-100 text-blue-800'
							: calibrationState === 'range_recording'
								? 'bg-yellow-100 text-yellow-800'
								: calibrationState === 'completed'
									? 'bg-green-100 text-green-800'
									: calibrationState === 'error'
										? 'bg-red-100 text-red-800'
										: 'bg-gray-100 text-gray-800'}"
					>
						{calibrationState}
					</span>
				</div>

				{#if isRecording}
					<div class="flex items-center space-x-6 text-sm">
						<div class="flex items-center space-x-2">
							<div class="w-2 h-2 bg-red-600 rounded-full animate-pulse"></div>
							<span class="font-medium text-gray-700">Recording</span>
						</div>
						<div>
							<span class="text-gray-600">Time:</span>
							<span class="font-mono font-medium text-gray-900">{formattedTime}</span>
						</div>
						<div>
							<span class="text-gray-600">Samples:</span>
							<span class="font-mono font-medium text-gray-900"
								>{positionSamples.toLocaleString()}</span
							>
						</div>
					</div>
				{/if}
			</div>

			{#if readings?.instruction}
				<div class="text-sm text-gray-700">
					<span class="font-medium">Instructions:</span>
					{readings.instruction}
				</div>
			{/if}
		</div>
	</div>

	<!-- Joint Progress Display -->
	{#if Object.keys(joints).length > 0}
		<div class="mb-6">
			<div class="flex items-center justify-between mb-4">
				<h4 class="text-lg font-semibold text-gray-900">Joint Range Progress</h4>
				<div class="text-sm text-gray-600">
					{completedJoints} / {totalJoints} joints completed
				</div>
			</div>

			<div class="grid grid-cols-1 md:grid-cols-2 gap-4">
				{#each Object.entries(joints).sort(([, a], [, b]) => b.id - a.id) as [jointName, joint] (joint.id)}
					{@const range = joint.recorded_max - joint.recorded_min}
					{@const progressWidth = getJointProgressWidth(joint)}

					<div class="border-2 rounded-lg p-4 {getJointStatusColor(joint)}">
						<div class="flex items-center justify-between mb-2">
							<h5 class="font-medium capitalize">
								{jointName.replace('_', ' ')}
							</h5>
							<div class="flex items-center space-x-2">
								{#if joint.is_completed}
									<svg class="w-5 h-5 text-green-600" fill="currentColor" viewBox="0 0 20 20">
										<path
											fill-rule="evenodd"
											d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
											clip-rule="evenodd"
										/>
									</svg>
									<span class="text-xs font-medium">Complete</span>
								{:else if hasRecordedRange(joint)}
									<!-- Same predicate as getJointStatusColor's "in progress" tint, so a
									     single-sample joint (span 0) can't be tinted blue and labelled
									     "Waiting..." at the same time. -->
									<span class="text-xs">Recording...</span>
								{:else}
									<span class="text-xs text-gray-500">Waiting...</span>
								{/if}
							</div>
						</div>

						<div class="text-xs space-y-1">
							<div class="flex justify-between">
								<span>Current:</span>
								<span class="font-mono">{joint.current_position}</span>
							</div>
							{#if hasRecordedRange(joint)}
								<div class="flex justify-between">
									<span>Range:</span>
									<span class="font-mono">{joint.recorded_min} - {joint.recorded_max}</span>
								</div>
								<div class="flex justify-between">
									<span>Span:</span>
									<span class="font-mono">{range}</span>
								</div>

								<!-- Progress Bar -->
								<div class="w-full bg-gray-200 rounded-full h-2 mt-2">
									<div
										class="bg-blue-600 h-2 rounded-full transition-all duration-300"
										style="width: {progressWidth}%"
									></div>
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

	<!-- Recording Controls -->
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
		{:else if canStartRecording}
			<!-- Ready to start recording -->
			<div class="text-center">
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Ready to Record Ranges</h4>
				<p class="text-gray-600 mb-6">
					Click "Start Recording" and then begin moving each joint through its full range of motion.
				</p>

				<button
					onclick={startRecording}
					disabled={isLoading}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-red-600 hover:bg-red-700 focus:outline-none focus:ring-2 focus:ring-red-500 disabled:opacity-50 disabled:cursor-not-allowed"
				>
					{#if isLoading}
						<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
						Starting Recording...
					{:else}
						<svg class="w-5 h-5 mr-3" fill="currentColor" viewBox="0 0 20 20">
							<path
								fill-rule="evenodd"
								d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z"
								clip-rule="evenodd"
							/>
						</svg>
						Start Recording
					{/if}
				</button>

				{@render abortControl(
					'Aborting discards the homing position set for this session and returns the machine to idle -- you will need to redo homing and range recording. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm.'
				)}
			</div>
		{:else if isRecording}
			<!-- Currently recording -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<div class="w-4 h-4 bg-red-600 rounded-full animate-pulse"></div>
				</div>
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Recording in Progress</h4>
				<p class="text-gray-600 mb-6">
					Move each joint through its full range. Watch the progress indicators above and continue
					until all joints are completed.
				</p>

				<div class="bg-yellow-50 p-4 rounded-lg mb-6">
					<p class="text-yellow-800 text-sm">
						<strong>Tip:</strong> Make sure to move each joint to its extreme positions in both directions.
						The system needs to record the full mechanical range for proper calibration.
					</p>
				</div>

				<button
					onclick={stopRecording}
					disabled={isLoading || !canRun(readings, 'stop_range_recording')}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500 disabled:opacity-50 disabled:cursor-not-allowed"
				>
					{#if isLoading}
						<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
						Stopping Recording...
					{:else}
						<svg class="w-5 h-5 mr-3" fill="currentColor" viewBox="0 0 20 20">
							<path
								fill-rule="evenodd"
								d="M10 18a8 8 0 100-16 8 8 0 000 16zM8 7a1 1 0 012 0v6a1 1 0 11-2 0V7zM12 7a1 1 0 012 0v6a1 1 0 11-2 0V7z"
								clip-rule="evenodd"
							/>
						</svg>
						Stop Recording
					{/if}
				</button>

				{#if !allJointsCompleted}
					<p class="mt-3 text-sm text-gray-600">
						Note: Some joints are not fully completed. You can stop recording now or continue moving
						them for better coverage.
					</p>
				{:else}
					<p class="mt-3 text-sm text-green-600 font-medium">
						✅ All joints have sufficient range data! You can stop recording now.
					</p>
				{/if}

				{@render abortControl(
					'Aborting stops recording now, discards every range recorded so far, and returns the machine to idle -- you will need to redo homing and move the arm through its full range again. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm.'
				)}
			</div>
		{:else if isCompleted}
			<!-- Recording completed successfully -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<svg class="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"
						></path>
					</svg>
				</div>
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Recording Complete!</h4>
				<p class="text-gray-600 mb-6">
					Range recording completed successfully. All joint limits have been captured and validated.
				</p>

				<div class="bg-green-50 p-4 rounded-lg mb-6 text-left max-w-md mx-auto">
					<h5 class="font-medium text-green-900 mb-2">Recording Summary:</h5>
					<div class="text-green-800 text-sm space-y-1">
						<div class="flex justify-between">
							<span>Total time:</span>
							<span class="font-mono">{formattedTime}</span>
						</div>
						<div class="flex justify-between">
							<span>Samples collected:</span>
							<span class="font-mono">{positionSamples.toLocaleString()}</span>
						</div>
						<div class="flex justify-between">
							<span>Joints completed:</span>
							<span class="font-mono">{completedJoints} / {totalJoints}</span>
						</div>
					</div>
				</div>

				<button
					onclick={nextStep}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
				>
					Save Calibration Data
					<svg class="ml-2 -mr-1 w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
							clip-rule="evenodd"
						/>
					</svg>
				</button>
			</div>
		{:else if isError}
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
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Recording Error</h4>
				<p class="text-red-600 mb-6">
					{readings?.error ||
						'An error occurred during range recording. This may be due to insufficient joint movement.'}
				</p>

				<div class="space-y-3">
					<button
						onclick={resetCalibration}
						disabled={isLoading || !canRun(readings, 'reset')}
						class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-red-600 hover:bg-red-700 focus:outline-none focus:ring-2 focus:ring-red-500 disabled:opacity-50"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
							Resetting...
						{:else}
							Reset and Start Over
						{/if}
					</button>

					<p class="text-xs text-gray-600">
						Make sure to move each joint through its complete range during recording.
					</p>
				</div>
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
						record joint ranges.
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
			<!-- Reached by walking back into this step while the module is still at 'started'
			     (homing not done yet), or a truly unrecognized state. The status block above already
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
				{#if canRun(readings, 'abort')}
					{@render abortControl(
						'Aborting cancels the current calibration session and returns the machine to idle.'
					)}
				{/if}
			</div>
		{/if}
	</div>

	<!-- Success Message -->
	{#if recordingCompleted && isCompleted}
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
					Range recording completed successfully! Ready to save calibration data.
				</span>
			</div>
		</div>
	{/if}
</div>
