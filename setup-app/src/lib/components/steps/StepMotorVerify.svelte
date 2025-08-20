<script lang="ts">
	import type { StepProps, MotorVerificationResult } from '$lib/types';

	let { 
		sendCommand, 
		setError, 
		clearError, 
		nextStep 
	}: StepProps = $props();

	// Component state
	let isLoading = $state(false);
	let verificationResults = $state<Record<string, MotorVerificationResult> | null>(null);
	let hasVerified = $state(false);

	// Expected motors in correct order
	const EXPECTED_MOTORS = [
		{ name: 'shoulder_pan', id: 1, description: 'Shoulder Pan Joint' },
		{ name: 'shoulder_lift', id: 2, description: 'Shoulder Lift Joint' },
		{ name: 'elbow_flex', id: 3, description: 'Elbow Flex Joint' },
		{ name: 'wrist_flex', id: 4, description: 'Wrist Flex Joint' },
		{ name: 'wrist_roll', id: 5, description: 'Wrist Roll Joint' },
		{ name: 'gripper', id: 6, description: 'Gripper (end effector)' }
	];

	// Computed values
	const allMotorsOk = $derived(
		verificationResults && 
		Object.values(verificationResults).every(motor => motor.status === 'ok')
	);

	const totalMotorsFound = $derived(
		verificationResults ? Object.keys(verificationResults).length : 0
	);

	// Verify motors
	async function verifyMotors() {
		try {
			isLoading = true;
			clearError();
			hasVerified = true;

			const result = await sendCommand({
				command: 'motor_setup_verify'
			});

			verificationResults = result.motors;

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
			case 'ok': return 'bg-green-100 border-green-300 text-green-800';
			case 'not_responding': return 'bg-red-100 border-red-300 text-red-800';
			case 'not_found': return 'bg-gray-100 border-gray-300 text-gray-600';
			default: return 'bg-gray-100 border-gray-300 text-gray-600';
		}
	}

	// Get status icon
	function getStatusIcon(status: string): string {
		switch (status) {
			case 'ok': return '✅';
			case 'not_responding': return '❌';
			case 'not_found': return '⚠️';
			default: return '❓';
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
						<path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"></path>
					</svg>
				</div>
				<div class="ml-3">
					<h4 class="font-semibold text-blue-900 mb-2">Ready for Verification</h4>
					<p class="text-blue-800 text-sm mb-2">
						Now connect ALL servo motors in a daisy chain configuration and verify they communicate properly.
					</p>
					<div class="text-blue-800 text-sm">
						<p><strong>Connection Order:</strong></p>
						<p>Controller → Shoulder Pan → Shoulder Lift → Elbow → Wrist Flex → Wrist Roll → Gripper</p>
					</div>
				</div>
			</div>
		</div>
	</div>

	<!-- Verification Button -->
	{#if !hasVerified}
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
						<path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"></path>
					</svg>
					Verify All Motors
				{/if}
			</button>
		</div>
	{/if}

	<!-- Verification Results -->
	{#if verificationResults}
		<div class="mb-8">
			<div class="flex items-center justify-between mb-6">
				<h4 class="text-xl font-semibold text-gray-900">Verification Results</h4>
				<div class="flex items-center space-x-4">
					<span class="text-sm text-gray-600">
						{totalMotorsFound} / 6 motors found
					</span>
					{#if allMotorsOk}
						<span class="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-green-100 text-green-800">
							<svg class="w-4 h-4 mr-1" fill="currentColor" viewBox="0 0 20 20">
								<path fill-rule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clip-rule="evenodd" />
							</svg>
							All Motors OK
						</span>
					{:else}
						<span class="inline-flex items-center px-3 py-1 rounded-full text-sm font-medium bg-red-100 text-red-800">
							<svg class="w-4 h-4 mr-1" fill="currentColor" viewBox="0 0 20 20">
								<path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clip-rule="evenodd" />
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
								<div class="text-xs mt-2">
									Motor not found or not responding
								</div>
							{/if}
						</div>
					</div>
				{/each}
			</div>

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
								<path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"></path>
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
					<!-- Success - all motors verified -->
					<div class="bg-green-50 p-6 rounded-lg border border-green-200 max-w-md text-center">
						<svg class="w-12 h-12 text-green-600 mx-auto mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
							<path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"></path>
						</svg>
						<h5 class="font-semibold text-green-900 mb-2">Verification Complete!</h5>
						<p class="text-green-800 text-sm mb-4">
							All motors are properly configured and responding. You can now proceed to calibration.
						</p>
						<button
							onclick={nextStep}
							class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500"
						>
							Start Calibration
							<svg class="ml-2 -mr-1 w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
								<path fill-rule="evenodd" d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z" clip-rule="evenodd" />
							</svg>
						</button>
					</div>
				{/if}
			</div>
		</div>
	{/if}
</div>