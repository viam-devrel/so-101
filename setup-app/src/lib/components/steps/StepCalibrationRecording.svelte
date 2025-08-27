<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';

	let { sensorReadings, sendCommand, setError, clearError, nextStep }: StepProps = $props();

	// Component state
	let isLoading = $state(false);
	let recordingCompleted = $state(false);

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	const joints = $derived(readings?.joints || {});
	const recordingTime = $derived(readings?.recording_time_seconds || 0);
	const positionSamples = $derived(readings?.position_samples || 0);

	// Check states
	const canStartRecording = $derived(calibrationState === 'homing_position');
	const isRecording = $derived(calibrationState === 'range_recording');
	const isCompleted = $derived(calibrationState === 'completed');
	const isError = $derived(calibrationState === 'error');

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

			// Auto-advance to next step after a short delay if successful
			setTimeout(() => {
				if (calibrationState === 'completed') {
					nextStep();
				}
			}, 2000);
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to stop recording');
		} finally {
			isLoading = false;
		}
	}

	// Get joint progress bar width
	function getJointProgressWidth(joint: any): number {
		if (!joint.recorded_min || !joint.recorded_max) return 0;
		const range = joint.recorded_max - joint.recorded_min;
		// Consider it good progress if range is > 1000 (rough heuristic)
		return Math.min(100, (range / 2000) * 100);
	}

	// Get joint status color
	function getJointStatusColor(joint: any): string {
		if (joint.is_completed) return 'bg-green-100 border-green-300 text-green-800';
		if (joint.recorded_min !== undefined && joint.recorded_max !== undefined) {
			const range = joint.recorded_max - joint.recorded_min;
			if (range < 1000) return 'bg-yellow-100 border-yellow-300 text-yellow-800';
			if (range > 500) return 'bg-blue-100 border-blue-300 text-blue-800';
		}
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
								{:else if range > 0}
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
							{#if joint.recorded_min !== undefined && joint.recorded_max !== undefined}
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
		{#if canStartRecording}
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
					disabled={isLoading}
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
						onclick={async () => {
							await sendCommand({ command: 'reset' });
						}}
						disabled={isLoading}
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
		{:else}
			<!-- Unexpected state -->
			<div class="text-center">
				<p class="text-gray-600 mb-4">
					Unexpected state: {calibrationState}
				</p>
				<button
					onclick={() => sensorReadings.current.refetch()}
					class="px-4 py-2 bg-gray-600 text-white rounded-md hover:bg-gray-700"
				>
					Refresh Status
				</button>
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
