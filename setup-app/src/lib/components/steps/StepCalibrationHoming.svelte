<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';

	let { sensorReadings, sendCommand, setError, clearError, nextStep }: StepProps = $props();

	// Component state
	let isLoading = $state(false);
	let homingSet = $state(false);

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	const joints = $derived(readings?.joints || {});

	// Check if we're in the correct state
	const canSetHoming = $derived(calibrationState === 'started');
	const alreadyAtHomingPosition = $derived(calibrationState === 'homing_position');

	// Set homing position
	async function setHomingPosition() {
		try {
			isLoading = true;
			clearError();

			await sendCommand({ command: 'set_homing' });
			homingSet = true;

			// Auto-advance to next step after a short delay
			setTimeout(() => {
				nextStep();
			}, 2000);
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to set homing position');
		} finally {
			isLoading = false;
		}
	}

	// Get current positions command for debugging
	async function getCurrentPositions() {
		try {
			const result = await sendCommand({ command: 'get_current_positions' });
			console.log('Current positions:', result);
		} catch (error) {
			console.error('Failed to get positions:', error);
		}
	}
</script>

<div class="max-w-4xl mx-auto">
	<!-- Header -->
	<div class="mb-8">
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Set Homing Position</h3>
		<p class="text-lg text-gray-600">
			Position the arm to the center of its movement range and set this as the homing reference
			point.
		</p>
	</div>

	<!-- Instructions -->
	<div class="bg-blue-50 p-6 rounded-lg mb-6">
		<h4 class="text-lg font-semibold text-blue-900 mb-4">Positioning Guidelines:</h4>

		<div class="grid md:grid-cols-2 gap-6">
			<div>
				<h5 class="font-medium text-blue-800 mb-3">What is "Center Position"?</h5>
				<ul class="text-blue-700 text-sm space-y-2">
					<li>• <strong>Shoulder Pan:</strong> Facing straight forward (not rotated left/right)</li>
					<li>• <strong>Shoulder Lift:</strong> Horizontal or slightly elevated</li>
					<li>• <strong>Elbow:</strong> Bent approximately 90 degrees</li>
					<li>• <strong>Wrist Flex:</strong> Neutral, not bent up or down</li>
					<li>• <strong>Wrist Roll:</strong> Neutral rotation</li>
					<li>• <strong>Gripper:</strong> Closed position</li>
				</ul>
			</div>

			<div>
				<h5 class="font-medium text-blue-800 mb-3">Why This Matters:</h5>
				<ul class="text-blue-700 text-sm space-y-2">
					<li>• Establishes the "zero" reference point for all movements</li>
					<li>• Ensures consistent positioning after power cycles</li>
					<li>• Provides optimal starting point for motion planning</li>
					<li>• Must be roughly in the middle of each joint's range</li>
				</ul>
			</div>
		</div>
	</div>

	<!-- Current Status -->
	<div class="mb-6">
		<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Status:</h4>
		<div class="bg-gray-50 p-4 rounded-lg">
			<div class="flex items-center space-x-4 mb-2">
				<span class="text-sm font-medium text-gray-700">Calibration State:</span>
				<span
					class="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium {calibrationState ===
					'started'
						? 'bg-yellow-100 text-yellow-800'
						: calibrationState === 'homing_position'
							? 'bg-green-100 text-green-800'
							: calibrationState === 'error'
								? 'bg-red-100 text-red-800'
								: 'bg-gray-100 text-gray-800'}"
				>
					{calibrationState}
				</span>
			</div>
			{#if readings?.instruction}
				<div class="text-sm text-gray-700">
					<span class="font-medium">Instructions:</span>
					{readings.instruction}
				</div>
			{/if}
		</div>
	</div>

	<!-- Real-time Joint Positions -->
	{#if Object.keys(joints).length > 0}
		<div class="mb-6">
			<h4 class="text-lg font-semibold text-gray-900 mb-3">Current Joint Positions:</h4>
			<div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
				{#each Object.entries(joints) as [jointName, joint]}
					<div class="bg-white border border-gray-200 rounded-lg p-4">
						<h5 class="font-medium text-gray-900 mb-2 capitalize">
							{jointName.replace('_', ' ')}
						</h5>
						<div class="text-sm text-gray-600 space-y-1">
							<div class="flex justify-between">
								<span>ID:</span>
								<span class="font-mono">{joint.id}</span>
							</div>
							<div class="flex justify-between">
								<span>Position:</span>
								<span class="font-mono">{joint.current_position}</span>
							</div>
							{#if joint.homing_offset !== undefined}
								<div class="flex justify-between">
									<span>Homing Offset:</span>
									<span class="font-mono">{joint.homing_offset}</span>
								</div>
							{/if}
						</div>
					</div>
				{/each}
			</div>
		</div>
	{/if}

	<!-- Positioning Controls -->
	<div class="bg-white border border-gray-200 rounded-lg p-6 mb-6">
		{#if canSetHoming}
			<!-- Ready to set homing -->
			<div class="text-center">
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Position the Arm</h4>
				<div class="bg-yellow-50 p-4 rounded-lg mb-6">
					<div class="flex items-start">
						<div class="flex-shrink-0">
							<svg
								class="h-6 w-6 text-yellow-600"
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
							<h5 class="font-medium text-yellow-900 mb-2">Manual Positioning Required</h5>
							<p class="text-yellow-800 text-sm">
								Manually move each joint to approximately the center of its range of motion. The
								motors are disabled, so you can move the arm freely by hand.
							</p>
						</div>
					</div>
				</div>

				<p class="text-gray-600 mb-6">
					Once you're satisfied with the arm position, click "Set Homing Position" to record this as
					the reference point.
				</p>

				<div class="flex justify-center space-x-4">
					<button
						onclick={setHomingPosition}
						disabled={isLoading}
						class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-green-600 hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500 disabled:opacity-50 disabled:cursor-not-allowed"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-5 w-5 border-b-2 border-white mr-3"></div>
							Setting Homing Position...
						{:else}
							<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
								<path
									stroke-linecap="round"
									stroke-linejoin="round"
									stroke-width="2"
									d="M17.657 16.657L13.414 20.9a1.998 1.998 0 01-2.827 0l-4.244-4.243a8 8 0 1111.314 0z"
								></path>
								<path
									stroke-linecap="round"
									stroke-linejoin="round"
									stroke-width="2"
									d="M15 11a3 3 0 11-6 0 3 3 0 016 0z"
								></path>
							</svg>
							Set Homing Position
						{/if}
					</button>

					<button
						onclick={getCurrentPositions}
						class="inline-flex items-center px-4 py-2 border border-gray-300 text-sm font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500"
					>
						<svg class="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
							<path
								stroke-linecap="round"
								stroke-linejoin="round"
								stroke-width="2"
								d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"
							></path>
						</svg>
						Debug Positions
					</button>
				</div>
			</div>
		{:else if alreadyAtHomingPosition}
			<!-- Homing position already set -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<svg class="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M17.657 16.657L13.414 20.9a1.998 1.998 0 01-2.827 0l-4.244-4.243a8 8 0 1111.314 0z"
						></path>
						<path
							stroke-linecap="round"
							stroke-linejoin="round"
							stroke-width="2"
							d="M15 11a3 3 0 11-6 0 3 3 0 016 0z"
						></path>
					</svg>
				</div>
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Homing Position Set</h4>
				<p class="text-gray-600 mb-6">
					The homing reference point has been established. The arm is ready for range recording.
				</p>

				<button
					onclick={nextStep}
					class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
				>
					Continue to Range Recording
					<svg class="ml-2 -mr-1 w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
							clip-rule="evenodd"
						/>
					</svg>
				</button>
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
				<h4 class="text-xl font-semibold text-gray-900 mb-4">Error Occurred</h4>
				<p class="text-red-600 mb-6">
					{readings?.error || 'An error occurred during homing position setup.'}
				</p>

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
						Reset Calibration
					{/if}
				</button>
			</div>
		{:else}
			<!-- Unexpected state -->
			<div class="text-center">
				<p class="text-gray-600 mb-4">
					Unexpected calibration state: {calibrationState}
				</p>
				<p class="text-sm text-gray-500 mb-4">
					Please ensure calibration was started in the previous step.
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
	{#if homingSet}
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
					Homing position set successfully! Ready to proceed with range recording.
				</span>
			</div>
		</div>
	{/if}
</div>
