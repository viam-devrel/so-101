<script lang="ts">
	import type { StepProps, MotorVerificationResult, MotorVerificationResponse } from '$lib/types';
	import { MOTOR_SETUP_ORDER } from '$lib/constants';
	import { logger } from '$lib/utils/logger';
	import BusScanPanel from '$lib/components/BusScanPanel.svelte';
	import { Button, Icon, InfoPanel } from '$lib/components/ui';

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
		<InfoPanel tone="info" title="Ready for Verification">
			<p class="text-blue-800 text-sm mb-2">
				Now connect ALL servo motors in a daisy chain configuration and verify they communicate
				properly.
			</p>
			<div class="text-blue-800 text-sm">
				<p><strong>Connection Order:</strong></p>
				<p>Controller → Shoulder Pan → Shoulder Lift → Elbow → Wrist Flex → Wrist Roll → Gripper</p>
			</div>
		</InfoPanel>
	</div>

	<!-- Verification Button (also serves as retry after a failed call, since
	     verificationResults stays null in that case) -->
	{#if !verificationResults}
		{#if verifiedEarlier}
			<InfoPanel tone="success" icon={null} className="mb-4 text-center">
				<p class="text-green-800 text-sm">
					All motors passed verification earlier in this session. You can continue with "Next", or
					re-run the check below if anything about the wiring changed.
				</p>
			</InfoPanel>
		{/if}
		<div class="text-center mb-8">
			<Button onclick={verifyMotors} loading={isLoading} variant="primary" size="lg">
				{#if !isLoading}
					<Icon name="checkCircle" className="w-5 h-5 mr-3" />
				{/if}
				{isLoading ? 'Verifying Motors...' : 'Verify All Motors'}
			</Button>
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
							<Icon name="checkSmall" className="w-4 h-4 mr-1" />
							All Motors OK
						</span>
					{:else if allMotorsOk}
						<span
							class="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-yellow-100 text-yellow-800"
						>
							<Icon name="warningTriangleSolid" className="w-4 h-4 mr-1" />
							Verified with Warnings
						</span>
					{:else}
						<span
							class="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-red-100 text-red-800"
						>
							<Icon name="xCircleSolid" className="w-4 h-4 mr-1" />
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
					<Button onclick={verifyMotors} loading={isLoading} variant="primary">
						{#if !isLoading}
							<Icon name="refresh" className="w-4 h-4 mr-2" />
						{/if}
						{isLoading ? 'Retrying...' : 'Retry Verification'}
					</Button>

					<InfoPanel tone="warning" icon={null} title="Troubleshooting Tips:" className="max-w-md">
						<ul class="text-yellow-800 text-sm space-y-1">
							<li>• Check all servo cable connections</li>
							<li>• Ensure power supply is stable</li>
							<li>• Verify motors are connected in daisy chain</li>
							<li>• Make sure no other applications are using the serial port</li>
						</ul>
					</InfoPanel>
				{:else}
					<!-- Success - all motors communicating (model_detection_failed doesn't block).
					     Not an InfoPanel: this is a stacked, centered confirmation card, not the
					     icon-beside-title layout InfoPanel renders. -->
					<div class="bg-green-50 p-6 rounded-lg border border-green-200 max-w-md text-center">
						<Icon name="checkCircle" className="w-12 h-12 text-green-600 mx-auto mb-4" />
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
						<Button onclick={nextStep} variant="success">
							Continue
							<Icon name="arrowRight" className="ml-2 -mr-1 w-4 h-4" />
						</Button>
					</div>
				{/if}
			</div>
		</div>
	{/if}
</div>
