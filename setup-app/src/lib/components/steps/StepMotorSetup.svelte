<script lang="ts">
	import type { StepProps } from '$lib/types';
	import { MOTOR_SETUP_ORDER } from '$lib/constants';
	import { Button, LoadingSpinner } from '$lib/components/ui';

	let {
		sendCommand,
		setError,
		clearError,
		nextStep,
		motorSetupResults,
		updateMotorSetupResult,
		clearMotorSetupResult,
		markStepComplete
	}: StepProps = $props();

	// First motor that's neither configured nor skipped, so resuming this step (it survives
	// unmount only via motorSetupResults -- currentMotorIndex itself does not) lands where the
	// user left off instead of restarting at motor 1. Once every motor is processed, prefer the
	// first *skipped* one: that is the only unfinished work left, and the alternative made a
	// returning user click "Previous Motor" back down the list to reach it. Last resort is the
	// final motor (everything is configured, in which case this panel isn't rendered at all).
	function findResumeIndex(): number {
		const unprocessed = MOTOR_SETUP_ORDER.findIndex((motor) => {
			const step = motorSetupResults[motor.name]?.step;
			return step !== 'configured' && step !== 'skipped';
		});
		if (unprocessed !== -1) return unprocessed;
		const skipped = MOTOR_SETUP_ORDER.findIndex(
			(motor) => motorSetupResults[motor.name]?.step === 'skipped'
		);
		return skipped !== -1 ? skipped : MOTOR_SETUP_ORDER.length - 1;
	}

	// Component state
	let isLoading = $state(false);
	let currentMotorIndex = $state(findResumeIndex());
	let discoveredMotor = $state<any>(null);

	// Computed values
	const currentMotor = $derived(MOTOR_SETUP_ORDER[currentMotorIndex]);
	const allMotorsConfigured = $derived(
		MOTOR_SETUP_ORDER.every((motor) => motorSetupResults[motor.name]?.step === 'configured')
	);
	const configuredCount = $derived(
		MOTOR_SETUP_ORDER.filter((motor) => motorSetupResults[motor.name]?.step === 'configured').length
	);
	// Every motor has reached a terminal state (configured or skipped) -- the point at which
	// there's nothing left to advance through, so a "continue anyway" escape hatch applies.
	const allMotorsProcessed = $derived(
		MOTOR_SETUP_ORDER.every((motor) => {
			const step = motorSetupResults[motor.name]?.step;
			return step === 'configured' || step === 'skipped';
		})
	);
	const showPartialContinue = $derived(allMotorsProcessed && !allMotorsConfigured);
	const currentMotorConfigured = $derived(
		motorSetupResults[currentMotor.name]?.step === 'configured'
	);

	// Once every motor is configured or skipped, this step is done -- lets the footer "Next"
	// button and the step's own "Continue" agree (see wizardProgress.ts). $effect (not a
	// direct call at assign/skip time) because two separate actions can flip
	// allMotorsProcessed to true. Safe to re-run only because markStepComplete writes a single
	// $state property without reading the record: a same-value property write notifies
	// nothing, whereas the object-spread reassign it used to do made this effect depend on the
	// state it writes and Svelte aborted the flush with effect_update_depth_exceeded.
	$effect(() => {
		if (allMotorsProcessed) {
			markStepComplete('motor_setup');
		}
	});

	// Get motor status for display
	function getMotorStatus(
		motorName: string
	): 'pending' | 'current' | 'discovered' | 'configured' | 'skipped' {
		const step = motorSetupResults[motorName]?.step;
		if (step === 'configured') return 'configured';
		if (step === 'discovered') return 'discovered';
		if (motorName === currentMotor?.name) return 'current';
		if (step === 'skipped') return 'skipped';
		return 'pending';
	}

	// Get status color class
	function getStatusColor(status: string): string {
		switch (status) {
			case 'current':
				return 'bg-blue-100 border-blue-300 text-blue-800';
			case 'discovered':
				return 'bg-yellow-100 border-yellow-300 text-yellow-800';
			case 'configured':
				return 'bg-green-100 border-green-300 text-green-800';
			case 'skipped':
				return 'bg-orange-100 border-orange-300 text-orange-800';
			default:
				return 'bg-gray-100 border-gray-300 text-gray-600';
		}
	}

	// Discover motor
	async function discoverMotor() {
		try {
			isLoading = true;
			clearError();

			const result = await sendCommand({
				command: 'motor_setup_discover',
				motor_name: currentMotor.name
			});

			discoveredMotor = result;

			// Update motor setup results
			updateMotorSetupResult(currentMotor.name, {
				motor_name: currentMotor.name,
				current_id: result.current_id,
				target_id: result.target_id,
				model: result.model,
				found_baudrate: result.found_baudrate,
				step: 'discovered',
				success: true
			});
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Discovery failed');
			discoveredMotor = null;
		} finally {
			isLoading = false;
		}
	}

	// Assign motor ID
	async function assignMotorId() {
		if (!discoveredMotor) return;

		try {
			isLoading = true;
			clearError();

			await sendCommand({
				command: 'motor_setup_assign_id',
				motor_name: currentMotor.name,
				current_id: discoveredMotor.current_id,
				target_id: discoveredMotor.target_id,
				current_baudrate: discoveredMotor.found_baudrate
			});

			// Update motor setup results to configured. Built from discoveredMotor rather than
			// spread from the prior result: the prior result's type is the MotorSetupResult
			// union, which may be the fieldless 'skipped' variant, so spreading it can't be
			// relied on to carry current_id/target_id/model/found_baudrate.
			updateMotorSetupResult(currentMotor.name, {
				motor_name: currentMotor.name,
				current_id: discoveredMotor.current_id,
				target_id: discoveredMotor.target_id,
				model: discoveredMotor.model,
				found_baudrate: discoveredMotor.found_baudrate,
				step: 'configured',
				success: true
			});
			// Always clear, even on the last motor -- previously this only ran inside the
			// advance branch below, so configuring the last motor left a stale "Motor
			// Discovered" panel on screen.
			discoveredMotor = null;

			// Move to next motor if there is one
			if (currentMotorIndex < MOTOR_SETUP_ORDER.length - 1) {
				currentMotorIndex++;
			}
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Configuration failed');
		} finally {
			isLoading = false;
		}
	}

	// Skip current motor. Never downgrades an already-configured motor to skipped -- writing
	// the result overwrites any prior 'discovered' entry too, so the grid can't show a stale
	// "Found: ID x -> y" for a motor now marked skipped.
	function skipMotor() {
		if (motorSetupResults[currentMotor.name]?.step !== 'configured') {
			updateMotorSetupResult(currentMotor.name, {
				motor_name: currentMotor.name,
				step: 'skipped',
				success: true
			});
		}
		if (currentMotorIndex < MOTOR_SETUP_ORDER.length - 1) {
			currentMotorIndex++;
		}
		discoveredMotor = null;
		clearError();
	}

	// Go back to previous motor
	function prevMotor() {
		if (currentMotorIndex > 0) {
			currentMotorIndex--;
			discoveredMotor = null;
			clearError();
		}
	}

	// Delete the stale discovery outright so the grid stops showing it as this motor's result.
	function rediscoverMotor() {
		clearMotorSetupResult(currentMotor.name);
		discoveredMotor = null;
	}
</script>

<div class="max-w-4xl mx-auto">
	<!-- Instructions -->
	<div class="mb-8">
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Motor Setup & Configuration</h3>
		<div class="bg-blue-50 p-6 rounded-lg mb-6">
			<div class="flex items-start">
				<div class="flex-shrink-0">
					<svg class="w-6 h-6 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
						></path>
					</svg>
				</div>
				<div class="ml-3">
					<h4 class="font-semibold text-blue-900 mb-2">Important: Connect ONE Motor at a Time</h4>
					<p class="text-blue-800 text-sm">
						To avoid ID conflicts, connect only the motor you're currently configuring. Disconnect
						all other servo motors from the daisy chain during this process.
					</p>
				</div>
			</div>
		</div>
	</div>

	<!-- Motor Progress Grid -->
	<div class="mb-8">
		<h4 class="text-lg font-semibold text-gray-900 mb-4">Configuration Progress</h4>
		<div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
			{#each MOTOR_SETUP_ORDER as motor, index}
				{@const status = getMotorStatus(motor.name)}
				{@const result = motorSetupResults[motor.name]}

				<div class="border-2 rounded-lg p-4 {getStatusColor(status)}">
					<div class="flex items-center justify-between mb-2">
						<h5 class="font-medium">
							{index + 1}. {motor.description}
						</h5>
						<span class="text-xs px-2 py-1 rounded-full border bg-white">
							ID: {motor.targetId}
						</span>
					</div>

					<div class="text-sm">
						{#if status === 'configured'}
							<div class="flex items-center text-green-700">
								<svg class="w-4 h-4 mr-1" fill="currentColor" viewBox="0 0 20 20">
									<path
										fill-rule="evenodd"
										d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
										clip-rule="evenodd"
									/>
								</svg>
								Configured
							</div>
						{:else if status === 'discovered' && result && result.step === 'discovered'}
							<div class="text-yellow-700">
								Found: ID {result.current_id} → {result.target_id}
							</div>
						{:else if status === 'current'}
							<div class="text-blue-700 font-medium">← Current Motor</div>
							<!-- 'current' outranks 'skipped', and skipping the last motor cannot advance
							     off it, so say so here rather than losing the skip mark. -->
							{#if motorSetupResults[motor.name]?.step === 'skipped'}
								<div class="text-blue-700">Skipped — not configured</div>
							{/if}
						{:else if status === 'skipped'}
							<div class="text-orange-700">Skipped</div>
						{:else}
							<div class="text-gray-600">Waiting...</div>
						{/if}
					</div>
				</div>
			{/each}
		</div>
	</div>

	<!-- Current Motor Configuration -->
	{#if !allMotorsConfigured}
		<div class="bg-white border border-gray-200 rounded-lg p-6 mb-6">
			<h4 class="text-xl font-semibold text-gray-900 mb-4">
				Configure {currentMotor.description}
			</h4>

			<div class="mb-6">
				<div class="bg-yellow-50 p-4 rounded-lg mb-4">
					<h5 class="font-medium text-yellow-900 mb-2">Step-by-step Instructions:</h5>
					<ol class="text-yellow-800 text-sm space-y-1">
						<li>1. Disconnect ALL servo motors from the daisy chain</li>
						<li>
							2. Connect ONLY the <strong>{currentMotor.description}</strong> to the controller
						</li>
						<li>3. Click "Discover Motor" to find the connected servo</li>
						<li>4. Click "Configure Motor" to set the correct ID and baudrate</li>
						<li>
							Already set up or unavailable? "Skip Motor" moves on without configuring it.
							Verification still pings every ID: a motor that was already set up passes, one that
							was never configured shows as not responding or not found.
						</li>
					</ol>
				</div>

				{#if discoveredMotor}
					<div class="bg-green-50 p-4 rounded-lg mb-4">
						<h5 class="font-medium text-green-900 mb-2">Motor Discovered:</h5>
						<div class="text-green-800 text-sm space-y-1">
							<p>• Current ID: {discoveredMotor.current_id}</p>
							<p>• Target ID: {discoveredMotor.target_id}</p>
							<p>• Model: {discoveredMotor.model}</p>
							<p>• Current Baudrate: {discoveredMotor.found_baudrate}</p>
						</div>
					</div>
				{/if}

				<div class="flex flex-wrap gap-3">
					{#if !discoveredMotor}
						<Button onclick={discoverMotor} disabled={isLoading} variant="primary">
							{#if isLoading}
								<LoadingSpinner size="sm" className="mr-2" />
								Discovering...
							{:else}
								<svg class="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
									<path
										stroke-linecap="round"
										stroke-linejoin="round"
										stroke-width="2"
										d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z"
									></path>
								</svg>
								Discover Motor
							{/if}
						</Button>
					{:else}
						<Button onclick={assignMotorId} disabled={isLoading} variant="success">
							{#if isLoading}
								<LoadingSpinner size="sm" className="mr-2" />
								Configuring...
							{:else}
								<svg class="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
									<path
										stroke-linecap="round"
										stroke-linejoin="round"
										stroke-width="2"
										d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"
									></path>
								</svg>
								Configure Motor
							{/if}
						</Button>

						<Button onclick={rediscoverMotor} disabled={isLoading} variant="secondary">
							Rediscover
						</Button>
					{/if}

					{#if currentMotorIndex > 0}
						<Button onclick={prevMotor} disabled={isLoading} variant="ghost">
							← Previous Motor
						</Button>
					{/if}

					{#if !currentMotorConfigured}
						<Button onclick={skipMotor} disabled={isLoading} variant="ghost">
							{currentMotorIndex < MOTOR_SETUP_ORDER.length - 1 ? 'Skip Motor →' : 'Skip Motor'}
						</Button>
					{/if}
				</div>
			</div>
		</div>

		{#if showPartialContinue}
			<div class="bg-orange-50 border border-orange-200 rounded-lg p-6 mb-6">
				<div class="flex items-start">
					<div class="flex-shrink-0">
						<svg class="w-6 h-6 text-orange-600" fill="currentColor" viewBox="0 0 20 20">
							<path
								fill-rule="evenodd"
								d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l6.518 11.59c.75 1.334-.213 2.98-1.742 2.98H3.48c-1.53 0-2.492-1.646-1.743-2.98l6.52-11.59zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z"
								clip-rule="evenodd"
							/>
						</svg>
					</div>
					<div class="ml-3">
						<h4 class="font-semibold text-orange-900 mb-2">
							{configuredCount} of {MOTOR_SETUP_ORDER.length} motors configured
						</h4>
						<p class="text-orange-800 text-sm mb-4">
							The rest were skipped, so this app did not configure them. Verification on the next
							step pings every ID anyway: any skipped motor that was not already set up will show as
							"not responding" or "not found" until you come back here and configure it.
						</p>
						<Button onclick={nextStep} variant="primary">
							Continue with {configuredCount} of {MOTOR_SETUP_ORDER.length} configured →
						</Button>
					</div>
				</div>
			</div>
		{/if}
	{:else}
		<!-- All motors configured -->
		<div class="bg-green-50 p-6 rounded-lg mb-6">
			<div class="flex items-center">
				<svg
					class="w-8 h-8 text-green-600 mr-4"
					fill="none"
					stroke="currentColor"
					viewBox="0 0 24 24"
				>
					<path
						stroke-linecap="round"
						stroke-linejoin="round"
						stroke-width="2"
						d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"
					></path>
				</svg>
				<div>
					<h4 class="text-lg font-semibold text-green-900">All Motors Configured!</h4>
					<p class="text-green-800">
						You can now connect all motors in a daisy chain and proceed to verification.
					</p>
				</div>
			</div>
		</div>

		<div class="text-center">
			<Button onclick={nextStep} variant="primary" size="lg">
				Continue to Verification
				<svg class="ml-2 -mr-1 w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
					<path
						fill-rule="evenodd"
						d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
						clip-rule="evenodd"
					/>
				</svg>
			</Button>
		</div>
	{/if}
</div>
