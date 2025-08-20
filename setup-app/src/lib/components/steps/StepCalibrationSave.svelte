<script lang="ts">
	import type { StepProps, CalibrationReadings } from '$lib/types';

	let { sensorReadings, sendCommand, setError, clearError, nextStep }: StepProps = $props();

	// Component state
	let isLoading = $state(false);
	let saveCompleted = $state(false);
	let calibrationFile = $state<string | null>(null);

	// Get current sensor readings
	const readings = $derived(sensorReadings.current.data as CalibrationReadings | undefined);
	const calibrationState = $derived(readings?.calibration_state || 'unknown');
	const joints = $derived(readings?.joints || {});

	// Check states
	const canSave = $derived(calibrationState === 'completed');
	const alreadySaved = $derived(calibrationState === 'idle' && saveCompleted);
	const isError = $derived(calibrationState === 'error');

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

			if (result.calibration_file) {
				calibrationFile = result.calibration_file;
			}

			// Auto-advance to next step after a short delay
			setTimeout(() => {
				nextStep();
			}, 2500);
		} catch (error) {
			setError(error instanceof Error ? error.message : 'Failed to save calibration');
		} finally {
			isLoading = false;
		}
	}

	// Format range span for display
	function formatRangeSpan(joint: any): string {
		if (!joint.recorded_min || !joint.recorded_max) return 'No data';
		const span = joint.recorded_max - joint.recorded_min;
		return span.toLocaleString();
	}

	// Get calibration quality indicator
	function getCalibrationQuality(joint: any): { label: string; color: string } {
		if (!joint.recorded_min || !joint.recorded_max) {
			return { label: 'No Data', color: 'text-gray-500' };
		}

		const span = joint.recorded_max - joint.recorded_min;
		if (joint.is_completed && span > 2000) {
			return { label: 'Excellent', color: 'text-green-600' };
		} else if (span > 1500) {
			return { label: 'Good', color: 'text-blue-600' };
		} else if (span > 1000) {
			return { label: 'Fair', color: 'text-yellow-600' };
		} else {
			return { label: 'Limited', color: 'text-red-600' };
		}
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
					{#each Object.entries(joints) as [jointName, joint]}
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
									{joint.recorded_min || 'N/A'}
								</div>

								<div class="font-mono text-sm text-gray-900">
									{joint.recorded_max || 'N/A'}
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
		{#if canSave}
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
		{:else if alreadySaved || (calibrationState === 'idle' && saveCompleted)}
			<!-- Successfully saved -->
			<div class="text-center">
				<div
					class="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<svg class="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
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
						<li>• Motors will be re-enabled with holding torque</li>
						<li>• Arm will move to homing position automatically</li>
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
					<button
						onclick={saveCalibration}
						disabled={isLoading}
						class="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50"
					>
						{#if isLoading}
							<div class="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
							Retrying...
						{:else}
							Retry Save
						{/if}
					</button>

					<button
						onclick={async () => {
							await sendCommand({ command: 'reset' });
						}}
						disabled={isLoading}
						class="inline-flex items-center px-4 py-2 border border-gray-300 text-sm font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50"
					>
						Reset Calibration
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
			<!-- Unexpected state -->
			<div class="text-center">
				<p class="text-gray-600 mb-4">
					Unexpected calibration state: {calibrationState}
				</p>
				<p class="text-sm text-gray-500 mb-4">
					Please ensure range recording was completed successfully.
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
	{#if saveCompleted}
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

