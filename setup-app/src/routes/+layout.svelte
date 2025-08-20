<script lang="ts">
	import '../app.css';
	import favicon from '$lib/assets/favicon.svg';
	import { onMount } from 'svelte';
	import { ViamProvider } from '@viamrobotics/svelte-sdk';
	import {
		parseConnectionFromCookies,
		createDialConfig,
		getConnectionErrorMessage
	} from '$lib/utils/connection';

	let { children } = $props();

	// Connection state
	let isLoading = $state(true);
	let connectionError = $state<string | null>(null);
	let dialConfigs = $state<Record<string, any>>({});
	let retryCount = $state(0);

	// Parse connection details on mount
	onMount(() => {
		parseConnection();
	});

	function parseConnection() {
		try {
			isLoading = true;
			connectionError = null;

			const { connectionDetails, machineId, error } = parseConnectionFromCookies();

			if (error || !connectionDetails) {
				connectionError = getConnectionErrorMessage(error || 'Unknown connection error');
				isLoading = false;
				return;
			}

			// Create DialConf for ViamProvider
			const dialConfig = createDialConfig(connectionDetails);
			dialConfigs = { main: dialConfig };

			isLoading = false;
		} catch (error) {
			connectionError = getConnectionErrorMessage(
				error instanceof Error ? error.message : 'Failed to initialize connection'
			);
			isLoading = false;
		}
	}

	function retryConnection() {
		retryCount++;
		parseConnection();
	}
</script>

<svelte:head>
	<link rel="icon" href={favicon} />
	<title>SO-101 Setup Wizard</title>
	<meta name="description" content="SO-101 robotic arm setup and calibration wizard" />
</svelte:head>

<div class="min-h-screen bg-gray-50">
	{#if isLoading}
		<!-- Loading state -->
		<div class="flex items-center justify-center min-h-screen">
			<div class="text-center">
				<div
					class="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600 mx-auto mb-4"
				></div>
				<h2 class="text-xl font-semibold text-gray-900 mb-2">Connecting to Robot</h2>
				<p class="text-gray-600">Parsing connection details...</p>
			</div>
		</div>
	{:else if connectionError}
		<!-- Connection error state -->
		<div class="flex items-center justify-center min-h-screen p-4">
			<div class="bg-white rounded-lg shadow-lg p-8 max-w-md w-full">
				<div class="text-center">
					<div
						class="w-12 h-12 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<svg class="w-6 h-6 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
							<path
								stroke-linecap="round"
								stroke-linejoin="round"
								stroke-width="2"
								d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"
							></path>
						</svg>
					</div>
					<h2 class="text-xl font-semibold text-gray-900 mb-4">Connection Error</h2>
					<p class="text-gray-600 mb-6">{connectionError}</p>

					<div class="space-y-3">
						<button
							onclick={retryConnection}
							class="w-full px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
						>
							{retryCount > 0 ? 'Retry Connection' : 'Try Again'}
						</button>

						<p class="text-sm text-gray-500">
							Make sure you're navigating to this page from the Viam app with proper robot
							credentials.
						</p>
					</div>
				</div>
			</div>
		</div>
	{:else}
		<!-- Successfully connected - render app with ViamProvider -->
		<ViamProvider {dialConfigs}>
			{@render children?.()}
		</ViamProvider>
	{/if}
</div>
