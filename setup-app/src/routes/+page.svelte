<script lang="ts">
	import { SensorClient, Struct } from '@viamrobotics/sdk';
	import {
		createResourceClient,
		createResourceQuery,
		createResourceMutation
	} from '@viamrobotics/svelte-sdk';
	import SetupWizard from '$lib/components/SetupWizard.svelte';
	import type { DoCommandResponse } from '$lib/types';

	// Fixed configuration values from steering document
	const partID = 'main';
	const sensorName = 'calibrator';

	// Create sensor client using Viam Svelte SDK
	const sensorClient = createResourceClient(
		SensorClient,
		() => partID,
		() => sensorName
	);

	// Create reactive query for sensor readings (1 second interval)
	const sensorReadings = createResourceQuery(sensorClient, 'getReadings', undefined, {
		refetchInterval: 1000
	});

	// Create mutation for DoCommand calls
	const doCommand = createResourceMutation(sensorClient, 'doCommand');

	// Helper function to send commands with error handling
	const sendCommand = async (cmd: any): Promise<DoCommandResponse> => {
		try {
			const result = await doCommand.current.mutateAsync([Struct.fromJson(cmd)]);
			const response = result as any;
			if (response && !response.success) {
				throw new Error(response.error || 'Command failed');
			}
			return response as DoCommandResponse;
		} catch (error) {
			// Transform technical errors into user-friendly messages
			const errorMsg = getUserFriendlyError(error);
			throw new Error(errorMsg);
		}
	};

	// Transform technical errors into user-friendly messages
	function getUserFriendlyError(error: any): string {
		const message = error instanceof Error ? error.message : String(error);

		if (message.includes('communication')) {
			return 'Communication failed. Check servo connections and try again.';
		} else if (message.includes('state:')) {
			return 'Invalid operation for current state. Please follow the workflow steps.';
		} else if (message.includes('timeout')) {
			return 'Operation timed out. Please check connections and try again.';
		} else {
			return `Operation failed: ${message}`;
		}
	}
</script>

<div class="container mx-auto px-4 py-8">
	<div class="max-w-4xl mx-auto">
		<header class="text-center mb-8">
			<h1 class="text-4xl font-bold text-gray-900 mb-4">SO-101 Setup Wizard</h1>
			<p class="text-xl text-gray-600">Configure and calibrate your SO-101 robotic arm</p>
		</header>

		{#if sensorReadings.current.isPending}
			<!-- Loading state while connecting to sensor -->
			<div class="flex items-center justify-center py-16">
				<div class="text-center">
					<div
						class="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600 mx-auto mb-4"
					></div>
					<h2 class="text-xl font-semibold text-gray-900 mb-2">Connecting to SO-101 Sensor</h2>
					<p class="text-gray-600">Establishing communication with calibration sensor...</p>
				</div>
			</div>
		{:else if sensorReadings.current.error}
			<!-- Sensor connection error -->
			<div class="bg-white rounded-lg shadow-lg p-8 max-w-md mx-auto">
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
					<h2 class="text-xl font-semibold text-gray-900 mb-4">Sensor Connection Failed</h2>
					<p class="text-gray-600 mb-6">
						Unable to connect to the SO-101 calibration sensor. Please check:
					</p>
					<ul class="text-left text-sm text-gray-600 mb-6 space-y-2">
						<li>• Robot configuration includes the 'so101-calibration' sensor component</li>
						<li>• SO-101 arm is properly connected to the robot computer</li>
						<li>• Serial port has correct permissions and is available</li>
					</ul>
					<button
						onclick={() => sensorReadings.current.refetch()}
						class="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
					>
						Retry Connection
					</button>
				</div>
			</div>
		{:else}
			<!-- Successfully connected to sensor - show setup wizard -->
			<SetupWizard {sensorClient} {sensorReadings} {doCommand} {sendCommand} />
		{/if}
	</div>
</div>
