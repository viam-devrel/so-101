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
		markStepComplete,
		markStepIncomplete,
		stepCompletion,
		torqueOutcome,
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
	let saveCompleted = $state(false);
	let calibrationFile = $state<string | null>(null);
	// Immediate local signal for a failed save attempt, set the moment the command rejects --
	// last_save_error (below) is the durable, cross-remount signal but lags up to the 1s
	// readings poll interval.
	let saveFailed = $state(false);

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	const joints = $derived(readings?.joints || {});
	const lastSaveError = $derived(readings?.last_save_error);
	// True once a save attempt has failed while the state stayed 'completed' (see
	// workflow.go's saveCalibration -- a failure there intentionally does not move to
	// StateError so save_calibration remains retryable). Must not be confused with isError
	// below, which is the genuine StateError branch.
	const hasFailedSave = $derived(saveFailed || !!lastSaveError);
	// isPending, not isLoading: with refetchInterval both are false on background polls,
	// but the query is disabled until the resource client resolves, and only isPending
	// covers that window -- isLoading there is false, so the panel fell through to
	// "Unexpected calibration state: unknown".
	const isInitialLoad = $derived(sensorReadings.current.isPending);
	const queryError = $derived(sensorReadings.current.error);

	// Check states
	const canSave = $derived(canRun(readings, 'save_calibration'));
	const isError = $derived(calibrationState === 'error');
	// A successful save leaves the module at 'idle' (workflow.go's saveCalibration), and this
	// step is destroyed by the wizard's {#if} on navigation, so saveCompleted alone renders
	// "Unexpected calibration state: idle" to a user who comes back after saving. The
	// wizard's completion record is the only signal that survives the remount.
	//
	// Deliberately conjoined with !canSave && !isError so a *new* recording session (which
	// leaves the module at 'completed' again) still gets the Save panel rather than a stale
	// success screen.
	const showSaved = $derived(
		saveCompleted || (stepCompletion.calibration_save && !canSave && !isError)
	);

	// Calculate summary statistics
	const totalJoints = $derived(Object.keys(joints).length);
	const completedJoints = $derived(
		Object.values(joints).filter((joint) => joint.is_completed).length
	);

	// Save calibration data
	async function saveCalibration() {
		try {
			isLoading = true;
			clearError();

			const result = await sendCommand({ command: 'save_calibration' });
			saveCompleted = true;
			// torque_enabled/torque_enable_error come from workflow.go's saveCalibration -- set
			// before markStepComplete so the footer never unlocks Next without this being known.
			setTorqueOutcome({
				enabled: result.torque_enabled === true,
				error: result.torque_enable_error
			});
			// Called at the point the flag flips, not via $effect on saveCompleted: this is the
			// only place saveCompleted is set true, so it fires exactly once per successful
			// save and never spuriously on an unrelated re-render.
			markStepComplete('calibration_save');
			saveFailed = false;

			if (result.calibration_file) {
				calibrationFile = result.calibration_file;
			}
		} catch (error) {
			saveFailed = true;
			// Both are stale now: torque is off (calibration disabled it and this attempt never
			// re-enabled it), and an earlier successful save's completion flag would otherwise
			// keep Next unlocked and let StepComplete claim the old torque outcome.
			setTorqueOutcome(null);
			markStepIncomplete('calibration_save');
			setError(error instanceof Error ? error.message : 'Failed to save calibration');
		} finally {
			isLoading = false;
		}
	}

	// Reset after an error or a failed-save abandon. The module drops to 'idle', which this
	// step has no panel for (falls through to "Unexpected calibration state"), so send the
	// user back to where the calibration line starts rather than stranding them there.
	//
	// Un-completes the whole calibration line (start/homing/recording/save), not just this
	// step: the module going back to 'idle' invalidates the progress each of those steps'
	// stepCompletion flag was recording, and canAdvanceFromStep's `stepCompletion[step] ||
	// ...` check would otherwise let a stale flag skip a step's real readings-based gate on
	// the way back through (see wizardProgress.ts). Motor setup/verify and overview are
	// unaffected -- a calibration reset does not touch them.
	async function resetCalibration() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'reset' });
			saveFailed = false;
			setTorqueOutcome(null);
			for (const step of CALIBRATION_STEPS) markStepIncomplete(step);
			goToNamedStep('calibration_start');
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to reset calibration');
		} finally {
			isLoading = false;
		}
	}

	// Format range span for display
	function formatRangeSpan(joint: any): string {
		if (!hasRecordedRange(joint)) return 'No data';
		const span = joint.recorded_max - joint.recorded_min;
		return span.toLocaleString();
	}

	// Get calibration quality indicator: is_completed is the primary signal
	function getCalibrationQuality(joint: any): { label: string; color: string } {
		if (!hasRecordedRange(joint)) {
			return { label: 'No Data', color: 'text-gray-500' };
		}
		if (!joint.is_completed) {
			return { label: 'Incomplete', color: 'text-red-600' };
		}

		const span = joint.recorded_max - joint.recorded_min;
		if (span >= ADVISORY_RANGE_TICKS) {
			return { label: 'Good', color: 'text-green-600' };
		}
		return { label: 'Narrow Range', color: 'text-yellow-600' };
	}
</script>

<div class="max-w-4xl mx-auto">
	<!-- Header -->
	<div class="mb-8">
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Save Calibration Data</h3>
		<p class="text-lg text-gray-600">
			Review and save the calibration data to servo memory and configuration file.
		</p>
	</div>

	<!-- Current Status -->
	<div class="mb-6">
		<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Status:</h4>
		<div class="bg-gray-50 p-4 rounded-lg">
			<div class="flex items-center space-x-4">
				<span class="text-sm font-medium text-gray-700">Calibration State:</span>
				<span
					class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium {calibrationState ===
					'completed'
						? 'bg-green-100 text-green-800'
						: calibrationState === 'idle'
							? 'bg-blue-100 text-blue-800'
							: calibrationState === 'error'
								? 'bg-red-100 text-red-800'
								: 'bg-gray-100 text-gray-800'}"
				>
					{calibrationState}
				</span>
			</div>
			{#if readings?.instruction}
				<div class="mt-2 text-sm text-gray-700">
					<span class="font-medium">Instructions:</span>
					{readings.instruction}
				</div>
			{/if}
		</div>
	</div>

	<!-- Calibration Summary -->
	{#if Object.keys(joints).length > 0}
		<div class="mb-8">
			<div class="flex items-center justify-between mb-4">
				<h4 class="text-lg font-semibold text-gray-900">Calibration Summary</h4>
				<div class="text-sm text-gray-600">
					{completedJoints} / {totalJoints} joints with complete data
				</div>
			</div>

			<div class="bg-white border border-gray-200 rounded-lg overflow-hidden">
				<div class="px-6 py-4 bg-gray-50 border-b border-gray-200">
					<div
						class="grid grid-cols-6 gap-4 text-xs font-medium text-gray-700 uppercase tracking-wide"
					>
						<div class="col-span-2">Joint</div>
						<div>Min Position</div>
						<div>Max Position</div>
						<div>Range Span</div>
						<div>Quality</div>
					</div>
				</div>

				<div class="divide-y divide-gray-200">
					{#each Object.entries(joints).sort(([, a], [, b]) => b.id - a.id) as [jointName, joint] (joint.id)}
						{@const quality = getCalibrationQuality(joint)}

						<div class="px-6 py-4 hover:bg-gray-50">
							<div class="grid grid-cols-6 gap-4 items-center">
								<div class="col-span-2">
									<div class="flex items-center">
										<h5 class="font-medium text-gray-900 capitalize">
											{jointName.replace('_', ' ')}
										</h5>
										{#if joint.is_completed}
											<svg
												class="w-4 h-4 text-green-500 ml-2"
												fill="currentColor"
												viewBox="0 0 20 20"
											>
												<path
													fill-rule="evenodd"
													d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
													clip-rule="evenodd"
												/>
											</svg>
										{/if}
									</div>
									<div class="text-sm text-gray-600">ID: {joint.id}</div>
								</div>

								<div class="font-mono text-sm text-gray-900">
									{hasRecordedRange(joint) ? joint.recorded_min : 'N/A'}
								</div>

								<div class="font-mono text-sm text-gray-900">
									{hasRecordedRange(joint) ? joint.recorded_max : 'N/A'}
								</div>

								<div class="font-mono text-sm text-gray-900">
									{formatRangeSpan(joint)}
								</div>

								<div class="text-sm font-medium {quality.color}">
									{quality.label}
								</div>
							</div>

							{#if joint.homing_offset !== undefined}
								<div class="mt-2 text-xs text-gray-600">
									Homing offset: {joint.homing_offset}
								</div>
							{/if}
						</div>
					{/each}
				</div>
			</div>
		</div>
	{/if}

	<!-- Save Controls -->
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
		{:else if showSaved}
			<!-- Successfully saved: wins over the Save panels while the module state lags, and
			     again on a later remount via the wizard's completion record (see showSaved).
			     torqueOutcome, not this panel's own success, decides whether the arm is safe to
			     let go of -- see workflow.go's saveCalibration: a torque re-enable failure still
			     lands here (the save itself succeeded) but leaves the arm limp. That must be the
			     most prominent thing on screen, not a footnote under a green checkmark. -->
			{#if torqueOutcome?.enabled === false}
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
					<h4 class="text-xl font-semibold text-gray-900 mb-4">
						Calibration Saved -- Arm Is Not Holding Position
					</h4>
					<div
						class="bg-red-50 border border-red-300 p-4 rounded-lg mb-6 text-left max-w-md mx-auto"
					>
						<p class="text-red-900 font-semibold mb-2">
							Holding torque could not be re-enabled. The arm is limp.
						</p>
						<p class="text-red-800 text-sm mb-2">
							Support the arm before letting go. Power-cycle the servos or restart the machine to
							restore holding torque.
						</p>
						<p class="text-red-700 text-xs font-mono break-all">
							{torqueOutcome.error}
						</p>
					</div>

					{#if calibrationFile}
						<div class="bg-gray-50 p-4 rounded-lg mb-6">
							<h5 class="font-medium text-gray-900 mb-2">Calibration File Saved:</h5>
							<p class="text-gray-800 text-sm font-mono break-all">
								{calibrationFile}
							</p>
						</div>
					{/if}

					<button
						onclick={nextStep}
						class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
					>
						Continue Anyway
						<svg class="ml-2 -mr-1 w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
							<path
								fill-rule="evenodd"
								d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
								clip-rule="evenodd"
							/>
						</svg>
					</button>
				</div>
			{:else}
				<div class="text-center">
					<div
						class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<svg
							class="w-8 h-8 text-green-600"
							fill="none"
							stroke="currentColor"
							viewBox="0 0 24 24"
						>
							<path
								stroke-linecap="round"
								stroke-linejoin="round"
								stroke-width="2"
								d="M8 7H5a2 2 0 00-2 2v9a2 2 0 002 2h14a2 2 0 002-2V9a2 2 0 00-2-2h-3m-1 4l-3-3m0 0l-3 3m3-3v12"
							></path>
						</svg>
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Calibration Saved Successfully!</h4>
					<p class="text-gray-600 mb-6">
						All calibration data has been written to servo memory and saved to the robot's
						configuration files.
					</p>

					{#if calibrationFile}
						<div class="bg-green-50 p-4 rounded-lg mb-6">
							<h5 class="font-medium text-green-900 mb-2">Calibration File Saved:</h5>
							<p class="text-green-800 text-sm font-mono break-all">
								{calibrationFile}
							</p>
						</div>
					{/if}

					<div class="bg-blue-50 p-4 rounded-lg mb-6 text-left max-w-md mx-auto">
						<h5 class="font-medium text-blue-900 mb-2">Next Steps:</h5>
						<ul class="text-blue-800 text-sm space-y-1">
							<li>• Motors have been re-enabled with holding torque</li>
							<li>• Ready for normal operation and control</li>
						</ul>
					</div>

					<button
						onclick={nextStep}
						class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
					>
						Complete Setup
						<svg class="ml-2 -mr-1 w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
							<path
								fill-rule="evenodd"
								d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
								clip-rule="evenodd"
							/>
						</svg>
					</button>
				</div>
			{/if}
		{:else if canSave && hasFailedSave}
			<!-- Completed, but the last save attempt failed. State deliberately stays
			     'completed' (see workflow.go's saveCalibration) so save_calibration is
			     retryable -- this must not silently fall back to the plain "Ready to Save"
			     panel below once the transient banner clears. -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-amber-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<svg class="w-8 h-8 text-amber-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"
						></path>
					</svg>
				</div>
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Save Attempt Failed</h4>
				<p class="text-amber-800 mb-6 max-w-md mx-auto">
					{lastSaveError ||
						'The last attempt to save calibration data failed. Your recorded ranges are unaffected -- retry when ready.'}
				</p>

				<div class="space-y-3">
					<button
						onclick={saveCalibration}
						disabled={isLoading}
						class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500 disabled:opacity-50 disabled:cursor-not-allowed"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
							Retrying...
						{:else}
							Retry Save
						{/if}
					</button>

					<div>
						<!-- Ungated on purpose. sensor.go advertises reset in every state and
						     workflow.go's reset has no state guard, so canRun would always be true
						     here -- and this is the only escape hatch from a stuck 'completed' save
						     failure, so it must stay reachable even if that ever changes. -->
						<button
							onclick={resetCalibration}
							disabled={isLoading}
							class="inline-flex items-center px-4 py-2 border border-gray-300 text-sm font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50"
						>
							{#if isLoading}
								<div
									class="animate-spin rounded-full h-4 w-4 border-b-2 border-gray-400 mr-2"
								></div>
								Resetting...
							{:else}
								Start Over Instead
							{/if}
						</button>
					</div>
				</div>

				<div class="mt-4 bg-yellow-50 p-3 rounded text-left max-w-md mx-auto">
					<p class="text-yellow-800 text-xs">
						<strong>Troubleshooting:</strong> Check servo power, connections, and ensure no other applications
						are using the serial port.
					</p>
				</div>
			</div>
		{:else if canSave}
			<!-- Ready to save -->
			<div class="text-center">
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Ready to Save Calibration</h4>

				<div class="bg-blue-50 p-4 rounded-lg mb-6 text-left max-w-2xl mx-auto">
					<h5 class="font-medium text-blue-900 mb-3">What will be saved:</h5>
					<ul class="text-blue-800 text-sm space-y-2">
						<li>• <strong>Homing offsets</strong> written to servo EEPROM registers</li>
						<li>• <strong>Range limits</strong> (min/max positions) written to servo EEPROM</li>
						<li>
							• <strong>Calibration file</strong> saved to robot computer for future reference
						</li>
						<li>• <strong>Drive modes and normalization</strong> configured per servo</li>
					</ul>
				</div>

				<div class="bg-amber-50 p-4 rounded-lg mb-6">
					<div class="flex items-start">
						<div class="flex-shrink-0">
							<svg
								class="h-5 w-5 text-amber-600"
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
							<p class="text-amber-800 text-sm">
								<strong>Note:</strong> This process writes data to servo EEPROM memory. Once saved, the
								calibration will persist across power cycles and be ready for normal operation.
							</p>
						</div>
					</div>
				</div>

				<button
					onclick={saveCalibration}
					disabled={isLoading}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500 disabled:opacity-50 disabled:cursor-not-allowed"
				>
					{#if isLoading}
						<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
						Saving Calibration...
					{:else}
						<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
							<path
								stroke-linecap="round"
								stroke-linejoin="round"
								stroke-width="2"
								d="M8 7H5a2 2 0 00-2 2v9a2 2 0 002 2h14a2 2 0 002-2V9a2 2 0 00-2-2h-3m-1 4l-3-3m0 0l-3 3m3-3v12"
							></path>
						</svg>
						Save Calibration Data
					{/if}
				</button>

				<p class="mt-3 text-xs text-gray-600">
					This process typically takes 2-3 seconds to write to all servo registers.
				</p>
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
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Save Error</h4>
				<p class="text-red-600 mb-6">
					{readings?.error ||
						'Failed to save calibration data. This may be due to servo communication issues.'}
				</p>

				<div class="space-y-3">
					<!-- No "Retry Save" here: save_calibration requires state 'completed'
					     (workflow.go's saveCalibration), which this branch never has -- a save
					     failure deliberately stays 'completed' (see canSave && hasFailedSave
					     above), so a genuine 'error' state has nothing for save_calibration to
					     retry. Reset is the only way out. -->
					<button
						onclick={resetCalibration}
						disabled={isLoading}
						class="inline-flex items-center px-4 py-2 border border-gray-300 text-sm font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-4 w-4 border-b-2 border-gray-400 mr-2"></div>
							Resetting...
						{:else}
							Reset Calibration
						{/if}
					</button>
				</div>

				<div class="mt-4 bg-yellow-50 p-3 rounded text-left max-w-md mx-auto">
					<p class="text-yellow-800 text-xs">
						<strong>Troubleshooting:</strong> Check servo power, connections, and ensure no other applications
						are using the serial port.
					</p>
				</div>
			</div>
		{:else}
			<!-- Reached when the module is off this step's line: 'idle' with no save on record
			     (an abort or reset elsewhere), or a state before 'completed'. Trust the module's
			     own instruction, and offer the way back rather than a dead end. -->
			<div class="text-center">
				<p class="text-gray-600 mb-4">
					{readings?.instruction || `Unexpected calibration state: ${calibrationState}`}
				</p>
				<div class="flex justify-center space-x-3">
					<button
						onclick={() => sensorReadings.current.refetch()}
						class="px-4 py-2 bg-gray-600 text-white rounded-md hover:bg-gray-700 focus:outline-none focus:ring-2 focus:ring-gray-500"
					>
						Refresh Status
					</button>
					{#if calibrationState === 'idle'}
						<button
							onclick={() => goToNamedStep('calibration_start')}
							class="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
						>
							Start New Calibration
						</button>
					{/if}
				</div>
			</div>
		{/if}
	</div>

	<!-- Success Message. Omitted when torque failed -- the panel above already carries that
	     warning prominently, and "ready for operation" would contradict it. -->
	{#if showSaved && torqueOutcome?.enabled !== false}
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
					Calibration data saved successfully! Your SO-101 arm is now ready for operation.
				</span>
			</div>
		</div>
	{/if}
</div>
