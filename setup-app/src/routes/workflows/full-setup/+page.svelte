<script lang="ts">
	import { page } from '$app/state';
	import { goto } from '$app/navigation';
	import { onMount } from 'svelte';
	import SensorProvider from '$lib/components/SensorProvider.svelte';
	import FullSetupWizard from '$lib/components/FullSetupWizard.svelte';
	import { getMachineRootPath } from '$lib/utils/connection';
	import type { SensorConfig } from '$lib/types';

	// Sensor configuration state
	let sensorConfig = $state<SensorConfig | null>(null);
	let configError = $state<string | null>(null);

	// Get sensor config from URL parameters
	function getSensorConfigFromURL(): SensorConfig | null {
		const urlParams = page.url.searchParams;
		const partId = urlParams.get('part');
		const sensorName = urlParams.get('sensor');

		if (partId && sensorName) {
			return { partId, sensorName };
		}
		return null;
	}

	// Get sensor config from session storage
	function getSensorConfigFromSession(): SensorConfig | null {
		try {
			const stored = sessionStorage.getItem('so101-setup-state');
			if (stored) {
				const sessionState = JSON.parse(stored);
				// Only use if less than 1 hour old
				if (Date.now() - (sessionState.timestamp || 0) < 3600000) {
					return sessionState.sensorConfig;
				}
			}
		} catch (error) {
			console.warn('Invalid session data');
		}
		return null;
	}

	// Initialize sensor configuration
	function initializeSensorConfig() {
		// Try URL parameters first
		let config = getSensorConfigFromURL();

		// Fall back to session storage
		if (!config) {
			config = getSensorConfigFromSession();
		}

		if (config) {
			sensorConfig = config;
		} else {
			configError = 'No sensor configuration found. Please configure your sensor first.';
		}
	}

	// Handle navigation back to landing page
	function goToLandingPage() {
		goto(getMachineRootPath());
	}

	// Initialize on mount
	onMount(() => {
		initializeSensorConfig();
	});
</script>

<svelte:head>
	<title>Complete Setup - SO-101</title>
	<meta name="description" content="Complete setup workflow for SO-101 arm" />
</svelte:head>

<div class="container mx-auto px-4 py-8">
	<!-- Header -->
	<div class="text-center mb-6">
		<div class="flex items-center justify-center mb-2">
			<svg class="w-6 h-6 text-blue-600 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
				<path
					stroke-linecap="round"
					stroke-linejoin="round"
					stroke-width="2"
					d="M19 14l-7 7m0 0l-7-7m7 7V3"
				></path>
			</svg>
			<span class="text-lg font-medium text-gray-700">Complete Setup Workflow</span>
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
		<!-- Full Setup Wizard -->
		<div class="max-w-4xl mx-auto">
			<SensorProvider {sensorConfig}>
				<FullSetupWizard />
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
					class="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600 mx-auto mb-4"
				></div>
				<h2 class="text-xl font-semibold text-gray-900 mb-2">Loading Complete Setup</h2>
				<p class="text-gray-600">Initializing workflow...</p>
			</div>
		</div>
	{/if}
</div>
