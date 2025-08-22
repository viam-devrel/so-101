<script lang="ts">
	import type { StepProps, MotorSetupConfig, MotorSetupResult } from '$lib/types';
	import { Button, LoadingSpinner } from '$lib/components/ui';

	let {
		sendCommand,
		setError,
		clearError,
		nextStep,
		motorSetupResults,
		updateMotorSetupResult
	}: StepProps = $props();

	// Motor configuration order (reverse assembly order to avoid ID conflicts)
	const MOTOR_SETUP_ORDER: MotorSetupConfig[] = [
		{ name: 'gripper', targetId: 6, description: 'Gripper (end effector)' },
		{ name: 'wrist_roll', targetId: 5, description: 'Wrist Roll Joint' },
		{ name: 'wrist_flex', targetId: 4, description: 'Wrist Flex Joint' },
		{ name: 'elbow_flex', targetId: 3, description: 'Elbow Flex Joint' },
		{ name: 'shoulder_lift', targetId: 2, description: 'Shoulder Lift Joint' },
		{ name: 'shoulder_pan', targetId: 1, description: 'Shoulder Pan Joint' }
	];

	// Component state
	let isLoading = $state(false);
	let currentMotorIndex = $state(0);
	let discoveredMotor = $state<any>(null);

	// Computed values
	const currentMotor = $derived(MOTOR_SETUP_ORDER[currentMotorIndex]);
	const allMotorsConfigured = $derived(
		MOTOR_SETUP_ORDER.every((motor) => motorSetupResults[motor.name]?.step === 'configured')
	);

	// Get motor status for display
	function getMotorStatus(motorName: string): 'pending' | 'current' | 'discovered' | 'configured' {
		const result = motorSetupResults[motorName];
		if (!result) {
			return motorName === currentMotor?.name ? 'current' : 'pending';
		}
		return result.step;
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

			// Update motor setup results to configured
			updateMotorSetupResult(currentMotor.name, {
				...motorSetupResults[currentMotor.name],
				step: 'configured'
			});

			// Move to next motor or complete
			if (currentMotorIndex < MOTOR_SETUP_ORDER.length - 1) {
				currentMotorIndex++;
				discoveredMotor = null;
			}
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Configuration failed');
		} finally {
			isLoading = false;
		}
	}

	// Skip to next motor
	function nextMotor() {
		if (currentMotorIndex < MOTOR_SETUP_ORDER.length - 1) {
			currentMotorIndex++;
			discoveredMotor = null;
			clearError();
		}
	}

	// Go back to previous motor
	function prevMotor() {
		if (currentMotorIndex > 0) {
			currentMotorIndex--;
			discoveredMotor = null;
			clearError();
		}
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
						{:else if status === 'discovered'}
							<div class="text-yellow-700">
								Found: ID {result?.current_id} → {result?.target_id}
							</div>
						{:else if status === 'current'}
							<div class="text-blue-700 font-medium">← Current Motor</div>
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
						<Button
							onclick={discoverMotor}
							disabled={isLoading}
							variant="primary"
						>
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
						<Button
							onclick={assignMotorId}
							disabled={isLoading}
							variant="success"
						>
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

						<Button
							onclick={() => (discoveredMotor = null)}
							disabled={isLoading}
							variant="secondary"
						>
							Rediscover
						</Button>
					{/if}

					{#if currentMotorIndex > 0}
						<Button
							onclick={prevMotor}
							disabled={isLoading}
							variant="ghost"
						>
							← Previous Motor
						</Button>
					{/if}

					{#if currentMotorIndex < MOTOR_SETUP_ORDER.length - 1}
						<Button
							onclick={nextMotor}
							disabled={isLoading}
							variant="ghost"
						>
							Skip Motor →
						</Button>
					{/if}
				</div>
			</div>
		</div>
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
			<Button
				onclick={nextStep}
				variant="primary"
				size="lg"
			>
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
