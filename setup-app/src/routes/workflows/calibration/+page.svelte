<script lang="ts">
	import { goto } from '$app/navigation';
	import { onMount } from 'svelte';
	import SensorProvider from '$lib/components/SensorProvider.svelte';
	import CalibrationWizard from '$lib/components/CalibrationWizard.svelte';
	import { getMachineRootPath } from '$lib/utils/connection';
	import { useWorkflowConfig } from '$lib/composables/useWorkflowConfig';
	import { logger } from '$lib/utils/logger';
	import type { SensorConfig } from '$lib/types';

	// Sensor configuration state
	let sensorConfig = $state<SensorConfig | null>(null);
	let configError = $state<string | null>(null);

	// Use workflow configuration composable
	const { initializeSensorConfig, markWorkflowCompleted } = useWorkflowConfig();

	// Initialize sensor configuration
	function initConfig() {
		logger.info('Initializing calibration workflow configuration');
		const { sensorConfig: config, source } = initializeSensorConfig();

		if (config) {
			sensorConfig = config;
			logger.debug('Sensor config loaded from', source);
		} else {
			configError = 'No sensor configuration found. Please configure your sensor first.';
			logger.warn('No sensor configuration available');
		}
	}

	// Mark workflow as completed when done
	function handleWorkflowComplete() {
		markWorkflowCompleted('calibration');
		logger.info('Calibration workflow completed');
	}

	// Handle navigation back to landing page
	function goToLandingPage() {
		goto(getMachineRootPath());
	}

	// Initialize on mount
	onMount(() => {
		initConfig();
	});
</script>

<svelte:head>
	<title>Calibration - SO-101</title>
	<meta name="description" content="Calibration workflow for SO-101 arm" />
</svelte:head>

<div class="container mx-auto px-4 py-8">
	<!-- Header -->
	<div class="text-center mb-6">
		<div class="flex items-center justify-center mb-2">
			<svg
				class="w-6 h-6 text-green-600 mr-2"
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
			<span class="text-lg font-medium text-gray-700">Calibration Workflow</span>
		</div>
	</div>

	{#if configError}
		<!-- Configuration Error -->
		<div class="max-w-2xl mx-auto">
			<div class="bg-white rounded-lg shadow-lg p-8 text-center">
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
				<h2 class="text-xl font-semibold text-gray-900 mb-4">Configuration Required</h2>
				<p class="text-gray-600 mb-6">{configError}</p>
				<button
					onclick={goToLandingPage}
					class="px-6 py-3 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
				>
					Configure Sensor
				</button>
			</div>
		</div>
	{:else if sensorConfig}
		<!-- Calibration Wizard -->
		<div class="max-w-4xl mx-auto">
			<SensorProvider {sensorConfig}>
				<CalibrationWizard />
			</SensorProvider>
		</div>

		<!-- Back to Landing Page -->
		<div class="text-center mt-8">
			<button
				onclick={goToLandingPage}
				class="text-blue-600 hover:text-blue-700 text-sm font-medium focus:outline-none focus:underline"
			>
				← Back to Workflow Selection
			</button>
		</div>
	{:else}
		<!-- Loading -->
		<div class="flex items-center justify-center py-16">
			<div class="text-center">
				<div
					class="animate-spin rounded-full h-12 w-12 border-b-2 border-green-600 mx-auto mb-4"
				></div>
				<h2 class="text-xl font-semibold text-gray-900 mb-2">Loading Calibration</h2>
				<p class="text-gray-600">Initializing workflow...</p>
			</div>
		</div>
	{/if}
</div>
