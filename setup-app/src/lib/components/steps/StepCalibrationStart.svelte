<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';
	import { canRun } from '$lib/calibrationCommands';
	import { resetCalibrationProgress } from '$lib/calibrationSession';
	import { Icon, InfoPanel, Button } from '$lib/components/ui';
	import CalibrationStatusGate from '$lib/components/calibration/CalibrationStatusGate.svelte';
	import CalibrationStatusPanel from '$lib/components/calibration/CalibrationStatusPanel.svelte';
	import AbortControl from '$lib/components/calibration/AbortControl.svelte';

	let {
		sensorReadings,
		sendCommand,
		setError,
		clearError,
		nextStep,
		markStepIncomplete,
		setTorqueOutcome
	}: StepProps = $props();

	// Component state
	let isLoading = $state(false);
	let calibrationStarted = $state(false);
	// Which calibration_state an open abort confirmation belongs to; see AbortControl.svelte.
	let confirmStateFor = $state<string | null>(null);
	// Deliberately NOT $state: the effect below reads it, so a reactive write would
	// re-run the effect and its cleanup would clear the timer it just armed.
	let hasAdvanced = false;

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');

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
			confirmStateFor = null;
			resetCalibrationProgress({ markStepIncomplete, setTorqueOutcome });
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
			resetCalibrationProgress({ markStepIncomplete, setTorqueOutcome });
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
	<InfoPanel tone="info" className="mb-6" icon={null}>
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
	</InfoPanel>

	<CalibrationStatusPanel {calibrationState} instruction={readings?.instruction} />

	<!-- Safety Warning -->
	<InfoPanel tone="safety" title="⚠️ Important Safety Notice" className="mb-6">
		<p class="font-medium">When calibration starts:</p>
		<ul class="list-disc list-inside space-y-1 text-sm">
			<li>All servo motors will be <strong>disabled</strong> (no holding torque)</li>
			<li>The arm will become limp and moveable by hand</li>
			<li>Support the arm to prevent sudden dropping or movement</li>
			<li>Keep your workspace clear and emergency stop accessible</li>
			<li>Move joints slowly and smoothly during the process</li>
		</ul>
	</InfoPanel>

	<!-- Calibration Controls -->
	<div class="bg-white border border-gray-200 rounded-lg p-6 mb-6">
		<CalibrationStatusGate {sensorReadings}>
			{#if calibrationState === 'idle' || calibrationState === 'completed'}
				<!-- Ready to start calibration -->
				<div class="text-center">
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Ready to Begin Calibration</h4>
					<p class="text-gray-600 mb-6">
						Make sure the arm is in a safe position and you're ready to manually guide it through
						the calibration process.
					</p>

					<Button
						variant="success"
						size="lg"
						loading={isLoading}
						disabled={!canRun(readings, 'start')}
						onclick={startCalibration}
					>
						{#if isLoading}
							Starting Calibration...
						{:else}
							<Icon name="gear" className="w-5 h-5 mr-3" />
							Start Calibration
						{/if}
					</Button>
				</div>
			{:else if alreadyStarted}
				<!-- Calibration already started -->
				<div class="text-center">
					<div
						class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<Icon name="checkCircle" className="w-8 h-8 text-green-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Calibration Started</h4>
					<p class="text-gray-600 mb-6">
						The calibration process is now active. Motors have been disabled and the arm should be
						moveable by hand.
					</p>

					<div class="flex justify-center space-x-4">
						<Button variant="primary" onclick={nextStep}>
							Continue to Homing
							<Icon name="arrowRight" className="ml-2 -mr-1 w-4 h-4" />
						</Button>
					</div>

					<!-- Gated on the module's own advertisement, not on `alreadyStarted`: the pre-refactor
					     abort button carried `!canRun(readings, 'abort')` in its disabled expression, and
					     the module decides what may run (cf. the equivalent gate in
					     StepCalibrationHoming/Recording's unexpected-state branch). -->
					{#if canRun(readings, 'abort')}
						<AbortControl
							message={calibrationState === 'started'
								? 'Aborting cancels this calibration session and returns the machine to idle. You will need to start calibration again from the beginning.'
								: 'Aborting discards the homing position you just set and returns the machine to idle. Setting homing already cleared the position limits stored in the servos, so run a full calibration before relying on the arm.'}
							bind:confirmStateFor
							{calibrationState}
							{isLoading}
							onAbort={abortCalibration}
						/>
					{/if}
				</div>
			{:else if calibrationState === 'error'}
				<!-- Error state -->
				<div class="text-center">
					<div
						class="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<Icon name="warningTriangle" className="w-8 h-8 text-red-600" />
					</div>
					<h4 class="text-xl font-semibold text-gray-900 mb-4">Calibration Error</h4>
					<p class="text-red-600 mb-6" role="alert">
						{readings?.error || 'An error occurred during calibration initialization.'}
					</p>

					<Button
						variant="danger"
						loading={isLoading}
						disabled={!canRun(readings, 'reset')}
						onclick={resetCalibration}
					>
						{isLoading ? 'Resetting...' : 'Reset and Retry'}
					</Button>
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
					<Button variant="secondary" onclick={() => sensorReadings.current.refetch()}>
						Refresh Status
					</Button>
				</div>
			{/if}
		</CalibrationStatusGate>
	</div>

	<!-- Success Message -->
	{#if calibrationStarted}
		<InfoPanel tone="success">
			<span class="text-green-900 font-medium">
				Calibration started successfully! Motors are now disabled and ready for manual positioning.
			</span>
		</InfoPanel>
	{/if}
</div>
