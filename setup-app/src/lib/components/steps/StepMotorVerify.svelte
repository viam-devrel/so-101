<script lang="ts">
	import type { StepProps, MotorVerificationResult, MotorVerificationResponse } from '$lib/types';
	import { MOTOR_SETUP_ORDER } from '$lib/constants';
	import { logger } from '$lib/utils/logger';
	import BusScanPanel from '$lib/components/BusScanPanel.svelte';

	let {
		sendCommand,
		error,
		setError,
		clearError,
		nextStep,
		markStepComplete,
		stepCompletion
	}: StepProps = $props();

	// Component state
	let isLoading = $state(false);
	let verificationResults = $state<Record<string, MotorVerificationResult> | null>(null);
	// verificationResults is local $state that the wizard's {#if} destroys on navigation, so a
	// user returning here sees the "Verify All Motors" button again. Re-verifying is a
	// read-only ping sweep, so the results are simply re-earned -- this only tells the user
	// why the grid is gone and that the wizard already counts the step as done.
	const verifiedEarlier = $derived(stepCompletion.motor_verify && !verificationResults);

	// Expected motors in daisy-chain order: MOTOR_SETUP_ORDER is reverse-assembly order (see
	// constants.ts), and this list is the same six motors read the other way, so derive it
	// rather than restating the names/IDs.
	const EXPECTED_MOTORS = [...MOTOR_SETUP_ORDER]
		.reverse()
		.map((motor) => ({ name: motor.name, id: motor.targetId, description: motor.description }));

	// Computed values
	//
	// A model_detection_failed motor answered its ping; only the model number it reported
	// was not one the driver knows (feetech DetectModel -> "unknown model number"). So it
	// warns rather than blocks -- matching motorSetupVerify, which leaves `allGood` true for
	// it. 'not_responding', 'not_found', and a missing entry all block.
	function isResponding(status: string | undefined): boolean {
		return status === 'ok' || status === 'model_detection_failed';
	}

	// Counted over EXPECTED_MOTORS, not over the response's own keys: a partial map must not
	// read as a pass just because every entry it does contain is ok.
	const totalMotorsResponding = $derived.by(() => {
		const results = verificationResults;
		if (!results) return 0;
		return EXPECTED_MOTORS.filter((expected) => isResponding(results[expected.name]?.status))
			.length;
	});

	const allMotorsOk = $derived(
		verificationResults !== null && totalMotorsResponding === EXPECTED_MOTORS.length
	);

	const hasWarnings = $derived(
		verificationResults !== null &&
			Object.values(verificationResults).some((motor) => motor.status === 'model_detection_failed')
	);

	// Verify motors
	//
	// motor_setup_verify's `success` now means what it means everywhere else in this wire
	// protocol: the call completed. The aggregate motor verdict travels separately as
	// `all_ok`. sendCommand throws on success:false, so a partial motor failure no longer
	// throws here -- it lands in result.motors like any other response.
	async function verifyMotors() {
		try {
			isLoading = true;
			clearError();

			const result = (await sendCommand({
				command: 'motor_setup_verify'
			})) as MotorVerificationResponse;

			if (!result?.motors || Object.keys(result.motors).length === 0) {
				// motorSetupVerify always populates all six motors and never returns an
				// error -- this guards against a malformed/unexpected response shape, not
				// a documented failure mode of this command.
				throw new Error(result?.error || 'Verification failed');
			}

			// allMotorsOk (below) is derived from the per-motor statuses, not from
			// result.all_ok -- the UI needs per-motor detail regardless, and must not be at
			// the mercy of the module's aggregate. Log rather than trust when they disagree,
			// since that only happens if this UI's notion of "responding" drifts from the
			// module's.
			const derivedAllOk = Object.values(result.motors).every((motor) =>
				isResponding(motor.status)
			);
			if (typeof result.all_ok === 'boolean' && result.all_ok !== derivedAllOk) {
				logger.warn('motor_setup_verify all_ok disagrees with per-motor derivation', {
					all_ok: result.all_ok,
					derivedAllOk,
					motors: result.motors
				});
			}

			verificationResults = result.motors;
			// Called here, not via $effect on allMotorsOk: this is the one place the result
			// that could make it true is produced, so it fires exactly once per successful
			// verification and never spuriously on an unrelated re-render.
			if (derivedAllOk) {
				markStepComplete('motor_verify');
			}
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Verification failed');
			verificationResults = null;
		} finally {
			isLoading = false;
		}
	}

	// Get status color for motor
	function getMotorStatusColor(status: string): string {
		switch (status) {
			case 'ok':
				return 'bg-green-100 border-green-300 text-green-800';
			case 'not_responding':
				return 'bg-red-100 border-red-300 text-red-800';
			case 'not_found':
				return 'bg-gray-100 border-gray-300 text-gray-600';
			case 'model_detection_failed':
				return 'bg-yellow-100 border-yellow-300 text-yellow-800';
			default:
				return 'bg-gray-100 border-gray-300 text-gray-600';
		}
	}

	// Get status icon
	function getStatusIcon(status: string): string {
		switch (status) {
			case 'ok':
				return '✅';
			case 'not_responding':
				return '❌';
			case 'not_found':
				return '⚠️';
			case 'model_detection_failed':
				return '🔍';
			default:
				return '❓';
		}
	}
</script>

<div class="max-w-4xl mx-auto">
	<!-- Instructions -->
	<div class="mb-8">
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Motor Verification</h3>
		<div class="bg-blue-50 p-6 rounded-lg">
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
					<h4 class="font-semibold text-blue-900 mb-2">Ready for Verification</h4>
					<p class="text-blue-800 text-sm mb-2">
						Now connect ALL servo motors in a daisy chain configuration and verify they communicate
						properly.
					</p>
					<div class="text-blue-800 text-sm">
						<p><strong>Connection Order:</strong></p>
						<p>
							Controller → Shoulder Pan → Shoulder Lift → Elbow → Wrist Flex → Wrist Roll → Gripper
						</p>
					</div>
				</div>
			</div>
		</div>
	</div>

	<!-- Verification Button (also serves as retry after a failed call, since
	     verificationResults stays null in that case) -->
	{#if !verificationResults}
		{#if verifiedEarlier}
			<div class="bg-green-50 border border-green-200 rounded-lg p-4 mb-4 text-center">
				<p class="text-green-800 text-sm">
					All motors passed verification earlier in this session. You can continue with "Next", or
					re-run the check below if anything about the wiring changed.
				</p>
			</div>
		{/if}
		<div class="text-center mb-8">
			<button
				onclick={verifyMotors}
				disabled={isLoading}
				class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50 disabled:cursor-not-allowed"
			>
				{#if isLoading}
					<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
					Verifying Motors...
				{:else}
					<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"
						></path>
					</svg>
					Verify All Motors
				{/if}
			</button>
		</div>

		{#if error}
			<!-- The verify call itself failed, so verificationResults never arrived and the
			     results branch below never renders. That is the case that most needs a bus
			     sweep: the command could not even complete. The wizard clears `error` on
			     navigation, so this only shows for a failure earned on this step. -->
			<div class="mb-6">
				<BusScanPanel {sendCommand} />
			</div>
		{/if}
	{/if}

	<!-- Verification Results -->
	{#if verificationResults}
		<div class="mb-8">
			<div class="flex items-center justify-between mb-6">
				<h4 class="text-xl font-semibold text-gray-900">Verification Results</h4>
				<div class="flex items-center space-x-4">
					<span class="text-sm text-gray-600">
						{totalMotorsResponding} / {EXPECTED_MOTORS.length} motors responding
					</span>
					{#if allMotorsOk && !hasWarnings}
						<span
							class="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-green-100 text-green-800"
						>
							<svg class="w-4 h-4 mr-1" fill="currentColor" viewBox="0 0 20 20">
								<path
									fill-rule="evenodd"
									d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
									clip-rule="evenodd"
								/>
							</svg>
							All Motors OK
						</span>
					{:else if allMotorsOk}
						<span
							class="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-yellow-100 text-yellow-800"
						>
							<svg class="w-4 h-4 mr-1" fill="currentColor" viewBox="0 0 20 20">
								<path
									fill-rule="evenodd"
									d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l6.518 11.59c.75 1.334-.213 2.98-1.742 2.98H3.48c-1.53 0-2.492-1.646-1.743-2.98l6.52-11.59zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z"
									clip-rule="evenodd"
								/>
							</svg>
							Verified with Warnings
						</span>
					{:else}
						<span
							class="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-red-100 text-red-800"
						>
							<svg class="w-4 h-4 mr-1" fill="currentColor" viewBox="0 0 20 20">
								<path
									fill-rule="evenodd"
									d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z"
									clip-rule="evenodd"
								/>
							</svg>
							Issues Found
						</span>
					{/if}
				</div>
			</div>

			<!-- Motor Status Grid -->
			<div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4 mb-6">
				{#each EXPECTED_MOTORS as expectedMotor}
					{@const motorResult = verificationResults[expectedMotor.name]}
					{@const status = motorResult?.status || 'not_found'}

					<div class="border-2 rounded-lg p-4 {getMotorStatusColor(status)}">
						<div class="flex items-center justify-between mb-2">
							<h5 class="font-medium">
								{expectedMotor.description}
							</h5>
							<span class="text-xl">
								{getStatusIcon(status)}
							</span>
						</div>

						<div class="text-sm space-y-1">
							<div class="flex justify-between">
								<span>Expected ID:</span>
								<span class="font-mono">{expectedMotor.id}</span>
							</div>
							{#if motorResult}
								<div class="flex justify-between">
									<span>Actual ID:</span>
									<span class="font-mono">{motorResult.id}</span>
								</div>
								{#if motorResult.model}
									<div class="flex justify-between">
										<span>Model:</span>
										<span class="font-mono">{motorResult.model}</span>
									</div>
								{/if}
								{#if motorResult.error}
									<div class="text-xs mt-2 p-2 bg-red-50 rounded border">
										{motorResult.error}
									</div>
								{/if}
							{:else}
								<div class="text-xs mt-2">Motor not found or not responding</div>
							{/if}
						</div>
					</div>
				{/each}
			</div>

			{#if !allMotorsOk}
				<!-- Evidence over guesswork: a real bus sweep, not just the cable-check tips
				     below. Offered only in this failure branch -- a working arm has nothing to
				     diagnose. -->
				<div class="mb-6">
					<BusScanPanel {sendCommand} />
				</div>
			{/if}

			<!-- Action Buttons -->
			<div class="flex flex-wrap gap-3 justify-center">
				{#if !allMotorsOk}
					<button
						onclick={verifyMotors}
						disabled={isLoading}
						class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
							Retrying...
						{:else}
							<svg class="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
								<path
									stroke-linecap="round"
									stroke-linejoin="round"
									stroke-width="2"
									d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"
								></path>
							</svg>
							Retry Verification
						{/if}
					</button>

					<div class="bg-yellow-50 p-4 rounded-lg border border-yellow-200 max-w-md">
						<h5 class="font-medium text-yellow-900 mb-2">Troubleshooting Tips:</h5>
						<ul class="text-yellow-800 text-sm space-y-1">
							<li>• Check all servo cable connections</li>
							<li>• Ensure power supply is stable</li>
							<li>• Verify motors are connected in daisy chain</li>
							<li>• Make sure no other applications are using the serial port</li>
						</ul>
					</div>
				{:else}
					<!-- Success - all motors communicating (model_detection_failed doesn't block) -->
					<div class="bg-green-50 p-6 rounded-lg border border-green-200 max-w-md text-center">
						<svg
							class="w-12 h-12 text-green-600 mx-auto mb-4"
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
						<h5 class="font-semibold text-green-900 mb-2">Verification Complete!</h5>
						<p class="text-green-800 text-sm mb-4">
							{#if hasWarnings}
								All motors are responding, but one or more reported a servo model the driver does
								not recognize -- see the grid above. Confirm you installed SO-101 (STS3215) servos
								before continuing.
							{:else}
								All motors are properly configured and responding. You can now proceed to the next
								step.
							{/if}
						</p>
						<button
							onclick={nextStep}
							class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500"
						>
							Continue
							<svg class="ml-2 -mr-1 w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
								<path
									fill-rule="evenodd"
									d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
									clip-rule="evenodd"
								/>
							</svg>
						</button>
					</div>
				{/if}
			</div>
		</div>
	{/if}
</div>
