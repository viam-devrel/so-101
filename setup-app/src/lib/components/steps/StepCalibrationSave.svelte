<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';
	import { ADVISORY_RANGE_TICKS, hasRecordedRange, sortedJoints } from '$lib/constants';
	import { canRun } from '$lib/calibrationCommands';
	import { resetCalibrationProgress } from '$lib/calibrationSession';
	import { Icon, InfoPanel, Button } from '$lib/components/ui';
	import CalibrationStatusGate from '$lib/components/calibration/CalibrationStatusGate.svelte';
	import CalibrationStatusPanel from '$lib/components/calibration/CalibrationStatusPanel.svelte';

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
			// keep Next unlocked and let StepComplete claim the old torque outcome. Not
			// resetCalibrationProgress -- this un-completes only calibration_save, not the whole
			// line, since the module state hasn't moved.
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
			resetCalibrationProgress({ markStepIncomplete, setTorqueOutcome });
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

	<CalibrationStatusPanel {calibrationState} instruction={readings?.instruction} />

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
					{#each sortedJoints(readings) as [jointName, joint] (joint.id)}
						{@const quality = getCalibrationQuality(joint)}

						<div class="px-6 py-4 hover:bg-gray-50">
							<div class="grid grid-cols-6 gap-4 items-center">
								<div class="col-span-2">
									<div class="flex items-center">
										<h5 class="font-medium text-gray-900 capitalize">
											{jointName.replace('_', ' ')}
										</h5>
										{#if joint.is_completed}
											<Icon name="checkSmall" className="w-4 h-4 text-green-500 ml-2" />
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
		<CalibrationStatusGate {sensorReadings}>
			{#if showSaved}
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
							<Icon name="warningTriangle" className="w-8 h-8 text-red-600" />
						</div>
						<h4 class="text-xl font-semibold text-gray-900 mb-4">
							Calibration Saved -- Arm Is Not Holding Position
						</h4>
						<div
							class="bg-red-50 border border-red-300 p-4 rounded-lg mb-6 text-left max-w-md mx-auto"
							role="alert"
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

						<Button variant="primary" size="lg" onclick={nextStep}>
							Continue Anyway
							<Icon name="arrowRight" className="ml-2 -mr-1 w-5 h-5" />
						</Button>
					</div>
				{:else}
					<div class="text-center">
						<div
							class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
						>
							<Icon name="download" className="w-8 h-8 text-green-600" />
						</div>
						<h4 class="text-xl font-semibold text-gray-900 mb-4">
							Calibration Saved Successfully!
						</h4>
						<p class="text-gray-600 mb-6">
							All calibration data has been written to servo memory and saved to the robot's
							configuration files.
						</p>

						{#if calibrationFile}
							<InfoPanel
								tone="success"
								title="Calibration File Saved:"
								icon={null}
								className="mb-6"
							>
								<p class="text-sm font-mono break-all">{calibrationFile}</p>
							</InfoPanel>
						{/if}

						<InfoPanel
							tone="info"
							title="Next Steps:"
							icon={null}
							className="mb-6 text-left max-w-md mx-auto"
						>
							<ul class="text-sm space-y-1">
								<li>• Motors have been re-enabled with holding torque</li>
								<li>• Ready for normal operation and control</li>
							</ul>
						</InfoPanel>

						<Button variant="primary" size="lg" onclick={nextStep}>
							Complete Setup
							<Icon name="arrowRight" className="ml-2 -mr-1 w-5 h-5" />
						</Button>
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
						<Icon name="warningTriangle" className="w-8 h-8 text-amber-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Save Attempt Failed</h4>
					<p class="text-amber-800 mb-6 max-w-md mx-auto" role="alert">
						{lastSaveError ||
							'The last attempt to save calibration data failed. Your recorded ranges are unaffected -- retry when ready.'}
					</p>

					<div class="space-y-3">
						<Button variant="success" size="lg" loading={isLoading} onclick={saveCalibration}>
							{isLoading ? 'Retrying...' : 'Retry Save'}
						</Button>

						<div>
							<!-- Ungated on purpose. sensor.go advertises reset in every state and
							     workflow.go's reset has no state guard, so canRun would always be true
							     here -- and this is the only escape hatch from a stuck 'completed' save
							     failure, so it must stay reachable even if that ever changes. -->
							<Button variant="secondary" loading={isLoading} onclick={resetCalibration}>
								{isLoading ? 'Resetting...' : 'Start Over Instead'}
							</Button>
						</div>
					</div>

					<InfoPanel tone="warning" icon={null} className="mt-4 text-left max-w-md mx-auto">
						<p class="text-xs">
							<strong>Troubleshooting:</strong> Check servo power, connections, and ensure no other applications
							are using the serial port.
						</p>
					</InfoPanel>
				</div>
			{:else if canSave}
				<!-- Ready to save -->
				<div class="text-center">
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Ready to Save Calibration</h4>

					<InfoPanel
						tone="info"
						title="What will be saved:"
						icon={null}
						className="mb-6 text-left max-w-2xl mx-auto"
					>
						<ul class="text-sm space-y-2">
							<li>• <strong>Homing offsets</strong> written to servo EEPROM registers</li>
							<li>
								• <strong>Range limits</strong> (min/max positions) written to servo EEPROM
							</li>
							<li>
								• <strong>Calibration file</strong> saved to robot computer for future reference
							</li>
							<li>• <strong>Drive modes and normalization</strong> configured per servo</li>
						</ul>
					</InfoPanel>

					<InfoPanel tone="warning" className="mb-6">
						<p class="text-sm">
							<strong>Note:</strong> This process writes data to servo EEPROM memory. Once saved, the
							calibration will persist across power cycles and be ready for normal operation.
						</p>
					</InfoPanel>

					<Button variant="success" size="lg" loading={isLoading} onclick={saveCalibration}>
						{#if isLoading}
							Saving Calibration...
						{:else}
							<Icon name="download" className="w-5 h-5 mr-3" />
							Save Calibration Data
						{/if}
					</Button>

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
						<Icon name="warningTriangle" className="w-8 h-8 text-red-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Save Error</h4>
					<p class="text-red-600 mb-6" role="alert">
						{readings?.error ||
							'Failed to save calibration data. This may be due to servo communication issues.'}
					</p>

					<div class="space-y-3">
						<!-- No "Retry Save" here: save_calibration requires state 'completed'
						     (workflow.go's saveCalibration), which this branch never has -- a save
						     failure deliberately stays 'completed' (see canSave && hasFailedSave
						     above), so a genuine 'error' state has nothing for save_calibration to
						     retry. Reset is the only way out. -->
						<Button variant="secondary" loading={isLoading} onclick={resetCalibration}>
							{isLoading ? 'Resetting...' : 'Reset Calibration'}
						</Button>
					</div>

					<InfoPanel tone="warning" icon={null} className="mt-4 text-left max-w-md mx-auto">
						<p class="text-xs">
							<strong>Troubleshooting:</strong> Check servo power, connections, and ensure no other applications
							are using the serial port.
						</p>
					</InfoPanel>
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
						<Button variant="secondary" onclick={() => sensorReadings.current.refetch()}>
							Refresh Status
						</Button>
						{#if calibrationState === 'idle'}
							<Button variant="primary" onclick={() => goToNamedStep('calibration_start')}>
								Start New Calibration
							</Button>
						{/if}
					</div>
				</div>
			{/if}
		</CalibrationStatusGate>
	</div>

	<!-- Success Message. Omitted when torque failed -- the panel above already carries that
	     warning prominently, and "ready for operation" would contradict it. -->
	{#if showSaved && torqueOutcome?.enabled !== false}
		<InfoPanel tone="success">
			<span class="text-green-900 font-medium">
				Calibration data saved successfully! Your SO-101 arm is now ready for operation.
			</span>
		</InfoPanel>
	{/if}
</div>
