<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';
	import { FULL_TRAVEL_TICKS, hasRecordedRange, sortedJoints } from '$lib/constants';
	import { canRun } from '$lib/calibrationCommands';
	import { resetCalibrationProgress } from '$lib/calibrationSession';
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
	let recordingCompleted = $state(false);
	// Which calibration_state an open abort confirmation belongs to; see AbortControl.svelte.
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

	// Check states
	const canStartRecording = $derived(canRun(readings, 'start_range_recording'));
	const isRecording = $derived(calibrationState === 'range_recording');
	const isCompleted = $derived(calibrationState === 'completed');
	const isError = $derived(calibrationState === 'error');

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
			recordingCompleted = false;
			resetCalibrationProgress({ markStepIncomplete, setTorqueOutcome });
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
		return Math.min(100, (range / FULL_TRAVEL_TICKS) * 100);
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
	<InfoPanel tone="info" className="mb-6" icon={null}>
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
	</InfoPanel>

	<CalibrationStatusPanel {calibrationState} instruction={readings?.instruction} label="State:">
		{#snippet children()}
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
		{/snippet}
	</CalibrationStatusPanel>

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
				{#each sortedJoints(readings) as [jointName, joint] (joint.id)}
					{@const range = joint.recorded_max - joint.recorded_min}
					{@const progressWidth = getJointProgressWidth(joint)}

					<div class="border-2 rounded-lg p-4 {getJointStatusColor(joint)}">
						<div class="flex items-center justify-between mb-2">
							<h5 class="font-medium capitalize">
								{jointName.replace('_', ' ')}
							</h5>
							<div class="flex items-center space-x-2">
								{#if joint.is_completed}
									<Icon name="checkSmall" className="w-5 h-5 text-green-600" />
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

	<!-- Recording Controls -->
	<div class="bg-white border border-gray-200 rounded-lg p-6 mb-6">
		<CalibrationStatusGate {sensorReadings}>
			{#if canStartRecording}
				<!-- Ready to start recording -->
				<div class="text-center">
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Ready to Record Ranges</h4>
					<p class="text-gray-600 mb-6">
						Click "Start Recording" and then begin moving each joint through its full range of
						motion.
					</p>

					<Button variant="danger" size="lg" loading={isLoading} onclick={startRecording}>
						{#if isLoading}
							Starting Recording...
						{:else}
							<Icon name="play" className="w-5 h-5 mr-3" />
							Start Recording
						{/if}
					</Button>

					<AbortControl
						message="Aborting discards the homing position set for this session and returns the machine to idle -- you will need to redo homing and range recording. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm."
						bind:confirmStateFor
						{calibrationState}
						{isLoading}
						onAbort={abortCalibration}
					/>
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

					<InfoPanel tone="warning" className="mb-6 text-left" icon={null}>
						<p class="text-sm">
							<strong>Tip:</strong> Make sure to move each joint to its extreme positions in both directions.
							The system needs to record the full mechanical range for proper calibration.
						</p>
					</InfoPanel>

					<Button
						variant="success"
						size="lg"
						loading={isLoading}
						disabled={!canRun(readings, 'stop_range_recording')}
						onclick={stopRecording}
					>
						{#if isLoading}
							Stopping Recording...
						{:else}
							<Icon name="stop" className="w-5 h-5 mr-3" />
							Stop Recording
						{/if}
					</Button>

					{#if !allJointsCompleted}
						<p class="mt-3 text-sm text-gray-600">
							Note: Some joints are not fully completed. You can stop recording now or continue
							moving them for better coverage.
						</p>
					{:else}
						<p class="mt-3 text-sm text-green-600 font-medium">
							✅ All joints have sufficient range data! You can stop recording now.
						</p>
					{/if}

					<AbortControl
						message="Aborting stops recording now, discards every range recorded so far, and returns the machine to idle -- you will need to redo homing and move the arm through its full range again. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm."
						bind:confirmStateFor
						{calibrationState}
						{isLoading}
						onAbort={abortCalibration}
					/>
				</div>
			{:else if isCompleted}
				<!-- Recording completed successfully -->
				<div class="text-center">
					<div
						class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<Icon name="checkCircle" className="w-8 h-8 text-green-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Recording Complete!</h4>
					<p class="text-gray-600 mb-6">
						Range recording completed successfully. All joint limits have been captured and
						validated.
					</p>

					<InfoPanel
						tone="success"
						title="Recording Summary:"
						icon={null}
						className="mb-6 text-left max-w-md mx-auto"
					>
						<div class="text-sm space-y-1">
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
					</InfoPanel>

					<Button variant="primary" size="lg" onclick={nextStep}>
						Save Calibration Data
						<Icon name="arrowRight" className="ml-2 -mr-1 w-5 h-5" />
					</Button>
				</div>
			{:else if isError}
				<!-- Error state -->
				<div class="text-center">
					<div
						class="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<Icon name="warningTriangle" className="w-8 h-8 text-red-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Recording Error</h4>
					<p class="text-red-600 mb-6" role="alert">
						{readings?.error ||
							'An error occurred during range recording. This may be due to insufficient joint movement.'}
					</p>

					<div class="space-y-3">
						<Button
							variant="danger"
							loading={isLoading}
							disabled={!canRun(readings, 'reset')}
							onclick={resetCalibration}
						>
							{isLoading ? 'Resetting...' : 'Reset and Start Over'}
						</Button>

						<p class="text-xs text-gray-600">
							Make sure to move each joint through its complete range during recording.
						</p>
					</div>
				</div>
			{:else if calibrationState === 'idle'}
				<!-- Idle: reached by our own abort (justAborted) or independently (fresh load, a
				     reset elsewhere). Only claim the abort happened when we caused it this session. -->
				<AbortedIdlePanel
					{justAborted}
					{justAbortedAfterHoming}
					notStartedMessage="record joint ranges"
					onStartNew={() => goToNamedStep('calibration_start')}
				/>
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
					<Button variant="secondary" onclick={() => sensorReadings.current.refetch()}>
						Refresh Status
					</Button>
					{#if canRun(readings, 'abort')}
						<AbortControl
							message="Aborting cancels the current calibration session and returns the machine to idle."
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
	{#if recordingCompleted && isCompleted}
		<InfoPanel tone="success">
			<span class="text-green-900 font-medium">
				Range recording completed successfully! Ready to save calibration data.
			</span>
		</InfoPanel>
	{/if}
</div>
