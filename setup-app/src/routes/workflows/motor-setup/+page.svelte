<script lang="ts">
	import { goto } from '$app/navigation';
	import { onMount } from 'svelte';
	import SensorProvider from '$lib/components/SensorProvider.svelte';
	import MotorSetupWizard from '$lib/components/MotorSetupWizard.svelte';
	import { useWorkflowConfig } from '$lib/composables/useWorkflowConfig';
	import { getMachineRootPath } from '$lib/utils/connection';
	import { logger } from '$lib/utils/logger';
	import type { SensorConfig } from '$lib/types';

	// Sensor configuration state
	let sensorConfig = $state<SensorConfig | null>(null);
	let configError = $state<string | null>(null);

	const { initializeSensorConfig } = useWorkflowConfig();

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
	<title>Motor Setup - SO-101</title>
	<meta name="description" content="Motor setup workflow for SO-101 arm" />
</svelte:head>

<div class="container mx-auto px-4 py-8">
	<!-- Header -->
	<div class="text-center mb-6">
		<div class="flex items-center justify-center mb-2">
			<svg
				class="w-6 h-6 text-orange-600 mr-2"
				fill="none"
				stroke="currentColor"
				viewBox="0 0 24 24"
			>
				<path
					stroke-linecap="round"
					stroke-linejoin="round"
					stroke-width="2"
					d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z M15 12a3 3 0 11-6 0 3 3 0 016 0z"
				></path>
			</svg>
			<span class="text-lg font-medium text-gray-700">Motor Setup Workflow</span>
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
		<!-- Motor Setup Wizard -->
		<div class="max-w-4xl mx-auto">
			<SensorProvider {sensorConfig}>
				<MotorSetupWizard />
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
					class="animate-spin rounded-full h-12 w-12 border-b-2 border-orange-600 mx-auto mb-4"
				></div>
				<h2 class="text-xl font-semibold text-gray-900 mb-2">Loading Motor Setup</h2>
				<p class="text-gray-600">Initializing workflow...</p>
			</div>
		</div>
	{/if}
</div>
