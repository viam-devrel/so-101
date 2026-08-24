<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';
	import { canRun } from '$lib/calibrationCommands';

	let {
		sensorReadings,
		sendCommand,
		setError,
		clearError,
		nextStep,
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
	let calibrationStarted = $state(false);
	// Deliberately NOT $state: the effect below reads it, so a reactive write would
	// re-run the effect and its cleanup would clear the timer it just armed.
	let hasAdvanced = false;

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	// isPending, not isLoading: with refetchInterval both are false on background polls,
	// but the query is disabled until the resource client resolves, and only isPending
	// covers that window -- isLoading there is false, so the panel fell through to
	// "Unexpected calibration state: unknown".
	const isInitialLoad = $derived(sensorReadings.current.isPending);
	const queryError = $derived(sensorReadings.current.error);

	// Check if calibration has already been started
	const alreadyStarted = $derived(
		calibrationState === 'started' || calibrationState === 'homing_position'
	);

	// Advance once polled readings confirm the state we caused, not on a blind timer.
	$effect(() => {
		if (calibrationStarted && calibrationState === 'started' && !hasAdvanced) {
			hasAdvanced = true;
			const timer = setTimeout(() => nextStep(), 1500);
			return () => clearTimeout(timer);
		}
	});

	// Start calibration workflow
	async function startCalibration() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'start' });
			calibrationStarted = true;
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to start calibration');
		} finally {
			isLoading = false;
		}
	}

	// Abort calibration if needed
	async function abortCalibration() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'abort' });
			calibrationStarted = false;
			// Re-arm the auto-advance for the next start attempt, same block as the flag it
			// guards (cf. StepCalibrationHoming/Recording).
			hasAdvanced = false;
			// abort/reset invalidate a prior run's save: torque is off again and the save
			// step must re-earn its gate. See StepCalibrationSave.resetCalibration.
			setTorqueOutcome(null);
			for (const step of CALIBRATION_STEPS) markStepIncomplete(step);
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
			calibrationStarted = false;
			hasAdvanced = false;
			// abort/reset invalidate a prior run's save: torque is off again and the save
			// step must re-earn its gate. See StepCalibrationSave.resetCalibration.
			setTorqueOutcome(null);
			for (const step of CALIBRATION_STEPS) markStepIncomplete(step);
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
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Start Calibration Process</h3>
		<p class="text-lg text-gray-600">
			Initialize the calibration workflow to set proper joint limits and homing positions for your
			SO-101 arm.
		</p>
	</div>

	<!-- What Calibration Does -->
	<div class="bg-blue-50 p-6 rounded-lg mb-6">
		<h4 class="text-lg font-semibold text-blue-900 mb-4">What Calibration Accomplishes:</h4>
		<div class="grid md:grid-cols-2 gap-4">
			<div>
				<h5 class="font-medium text-blue-800 mb-2">Homing Positions</h5>
				<ul class="text-blue-700 text-sm space-y-1">
					<li>• Sets the "zero" position for each joint</li>
					<li>• Establishes reference point for all movements</li>
					<li>• Ensures consistent positioning across restarts</li>
				</ul>
			</div>
			<div>
				<h5 class="font-medium text-blue-800 mb-2">Range Limits</h5>
				<ul class="text-blue-700 text-sm space-y-1">
					<li>• Records maximum safe movement range</li>
					<li>• Prevents mechanical damage from over-extension</li>
					<li>• Optimizes movement planning and control</li>
				</ul>
			</div>
		</div>
	</div>

	<!-- Current Sensor Status -->
	<div class="mb-6">
		<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Status:</h4>
		<div class="bg-gray-50 p-4 rounded-lg">
			<div class="flex items-center space-x-4">
				<span class="text-sm font-medium text-gray-700">Calibration State:</span>
				<span
					class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium {calibrationState ===
					'idle'
						? 'bg-gray-100 text-gray-800'
						: calibrationState === 'started'
							? 'bg-yellow-100 text-yellow-800'
							: calibrationState === 'error'
								? 'bg-red-100 text-red-800'
								: 'bg-blue-100 text-blue-800'}"
				>
					{calibrationState}
				</span>
			</div>
			{#if readings?.instruction}
				<div class="mt-3 text-sm text-gray-700">
					<span class="font-medium">Instructions:</span>
					{readings.instruction}
				</div>
			{/if}
		</div>
	</div>

	<!-- Safety Warning -->
	<div class="bg-amber-50 border-l-4 border-amber-400 p-6 mb-6">
		<div class="flex items-start">
			<div class="flex-shrink-0">
				<svg class="h-6 w-6 text-amber-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
					<path
						stroke-linecap="round"
						stroke-linejoin="round"
						stroke-width="2"
						d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"
					></path>
				</svg>
			</div>
			<div class="ml-3">
				<h4 class="text-lg font-semibold text-amber-900 mb-2">⚠️ Important Safety Notice</h4>
				<div class="text-amber-800 space-y-2">
					<p class="font-medium">When calibration starts:</p>
					<ul class="list-disc list-inside space-y-1 text-sm">
						<li>All servo motors will be <strong>disabled</strong> (no holding torque)</li>
						<li>The arm will become limp and moveable by hand</li>
						<li>Support the arm to prevent sudden dropping or movement</li>
						<li>Keep your workspace clear and emergency stop accessible</li>
						<li>Move joints slowly and smoothly during the process</li>
					</ul>
				</div>
			</div>
		</div>
	</div>

	<!-- Calibration Controls -->
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
		{:else if calibrationState === 'idle' || calibrationState === 'completed'}
			<!-- Ready to start calibration -->
			<div class="text-center">
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Ready to Begin Calibration</h4>
				<p class="text-gray-600 mb-6">
					Make sure the arm is in a safe position and you're ready to manually guide it through the
					calibration process.
				</p>

				<button
					onclick={startCalibration}
					disabled={isLoading || !canRun(readings, 'start')}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500 disabled:opacity-50 disabled:cursor-not-allowed"
				>
					{#if isLoading}
						<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
						Starting Calibration...
					{:else}
						<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
							<path
								stroke-linecap="round"
								stroke-linejoin="round"
								stroke-width="2"
								d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 100 4m0-4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 100 4m0-4v2m0-6V4"
							></path>
						</svg>
						Start Calibration
					{/if}
				</button>
			</div>
		{:else if alreadyStarted}
			<!-- Calibration already started -->
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
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Calibration Started</h4>
				<p class="text-gray-600 mb-6">
					The calibration process is now active. Motors have been disabled and the arm should be
					moveable by hand.
				</p>

				<div class="flex justify-center space-x-4">
					<button
						onclick={nextStep}
						class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
					>
						Continue to Homing
						<svg class="ml-2 -mr-1 w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
							<path
								fill-rule="evenodd"
								d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
								clip-rule="evenodd"
							/>
						</svg>
					</button>

					<button
						onclick={abortCalibration}
						disabled={isLoading || !canRun(readings, 'abort')}
						class="inline-flex items-center px-4 py-2 border border-gray-300 text-sm font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-4 w-4 border-b-2 border-gray-400 mr-2"></div>
							Aborting...
						{:else}
							Abort Calibration
						{/if}
					</button>
				</div>
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
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Calibration Error</h4>
				<p class="text-red-600 mb-6">
					{readings?.error || 'An error occurred during calibration initialization.'}
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
						Reset and Retry
					{/if}
				</button>
			</div>
		{:else}
			<!-- Reached when the module has moved past this step (e.g. range recording), or a
			     truly unrecognized state. The status block above already renders the module's
			     instruction, so point at it rather than repeating it. Worded to match the
			     equivalent branch in StepCalibrationHoming/Recording. -->
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
			</div>
		{/if}
	</div>

	<!-- Success Message -->
	{#if calibrationStarted}
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
					Calibration started successfully! Motors are now disabled and ready for manual
					positioning.
				</span>
			</div>
		</div>
	{/if}
</div>
