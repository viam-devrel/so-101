<script lang="ts">
	import { SensorClient } from '@viamrobotics/sdk';
	import { createResourceClient, createResourceQuery } from '@viamrobotics/svelte-sdk';
	import type { SensorConfig } from '$lib/types';
	import { validateSensorConfig } from '$lib/utils/validation';
	import { logger } from '$lib/utils/logger';
	import { Button, Alert, LoadingSpinner } from '$lib/components/ui';

	interface Props {
		onConfigValid: (config: SensorConfig) => void;
	}

	let { onConfigValid }: Props = $props();

	// Form state
	let partId = $state('main');
	let sensorName = $state('');
	let isTestingConnection = $state(false);
	let connectionStatus = $state<'idle' | 'testing' | 'success' | 'error'>('idle');
	let connectionError = $state<string | null>(null);
	let validationError = $state<string | null>(null);

	// Form validation
	const isFormValid = $derived(() => {
		const validation = validateSensorConfig(partId.trim(), sensorName.trim());
		validationError = validation.isValid ? null : validation.error!;
		return validation.isValid;
	});

	// Create reactive sensor client that updates when form values change
	const sensorClient = createResourceClient(
		SensorClient,
		() => partId.trim(),
		() => sensorName.trim()
	);

	// Create reactive query that updates when client changes
	const sensorQuery = createResourceQuery(sensorClient, 'getReadings', undefined, {
		enabled: false,
		refetchInterval: false
	});

	// Test sensor connection
	async function testConnection() {
		// Validate inputs first
		const validation = validateSensorConfig(partId.trim(), sensorName.trim());
		if (!validation.isValid) {
			connectionError = validation.error!;
			connectionStatus = 'error';
			return;
		}

		isTestingConnection = true;
		connectionStatus = 'testing';
		connectionError = null;

		try {
			logger.info('Testing sensor connection', {
				partId: partId.trim(),
				sensorName: sensorName.trim()
			});

			// Manually trigger the query
			const query = sensorQuery;
			await query.current.refetch();

			// Wait for the query to resolve or reject
			await new Promise((resolve, reject) => {
				// Check query result after refetch
				setTimeout(() => {
					const result = query.current;
					if (result.data) {
						logger.info('Connection test successful', result.data);
						resolve(result.data);
					} else if (result.error) {
						logger.warn('Connection test error', result.error);
						reject(result.error);
					} else if (result.isLoading) {
						// Still loading, reject with timeout
						reject(new Error('Connection test timed out'));
					} else {
						reject(new Error('Connection test failed - no data received'));
					}
				}, 2000);

				// Overall timeout after 5 seconds
				setTimeout(() => {
					reject(new Error('Connection test timed out'));
				}, 5000);
			});

			connectionStatus = 'success';

			// Auto-proceed after successful connection
			setTimeout(() => {
				const config = { partId: partId.trim(), sensorName: sensorName.trim() };
				logger.info('Sensor configuration validated, proceeding', config);
				onConfigValid(config);
			}, 1000);
		} catch (error) {
			connectionStatus = 'error';
			connectionError = error instanceof Error ? error.message : 'Connection test failed';
			logger.error('Connection test failed', error as Error);
		} finally {
			isTestingConnection = false;
		}
	}
</script>

<div class="max-w-2xl mx-auto">
	<!-- Header -->
	<div class="text-center mb-8">
		<h1 class="text-4xl font-bold text-gray-900 mb-4">SO-101 Setup</h1>
		<p class="text-xl text-gray-600">Configure your sensor connection to get started</p>
	</div>

	<div class="bg-white rounded-lg shadow-lg p-8">
		<h2 class="text-2xl font-semibold text-gray-900 mb-6">Sensor Configuration</h2>

		<div class="space-y-6">
			<div>
				<label for="sensorName" class="block text-sm font-medium text-gray-700 mb-2">
					Sensor Component Name <span class="text-red-500">*</span>
				</label>
				<input
					id="sensorName"
					type="text"
					bind:value={sensorName}
					placeholder="Enter your SO-101 sensor component name"
					class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
				/>
				<p class="mt-1 text-sm text-gray-600">
					The name of your SO-101 calibration sensor component as configured in your robot
				</p>
			</div>

			<div>
				<Button
					onclick={testConnection}
					disabled={!isFormValid || isTestingConnection}
					variant="primary"
					size="lg"
					className="w-full"
				>
					{#if isTestingConnection}
						<LoadingSpinner size="sm" className="mr-3" />
						Testing Connection...
					{:else if connectionStatus === 'success'}
						<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
							<path
								stroke-linecap="round"
								stroke-linejoin="round"
								stroke-width="2"
								d="M5 13l4 4L19 7"
							></path>
						</svg>
						Connection Successful
					{:else}
						<svg class="w-5 h-5 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
							<path
								stroke-linecap="round"
								stroke-linejoin="round"
								stroke-width="2"
								d="M8.111 16.404a5.5 5.5 0 017.778 0M12 20h.01m-7.08-7.071c3.904-3.905 10.236-3.905 14.141 0M1.394 9.393c5.857-5.857 15.355-5.857 21.213 0"
							></path>
						</svg>
						Test Connection
					{/if}
				</Button>
			</div>

			<!-- Connection Status -->
			{#if connectionStatus === 'success'}
				<Alert variant="success" title="Connection Successful">
					Successfully connected to sensor "{sensorName}" in part "{partId}". Proceeding to workflow
					selection...
				</Alert>
			{:else if connectionStatus === 'error'}
				<Alert variant="error" title="Connection Failed">
					<div>
						<p>{connectionError}</p>
						<div class="mt-3">
							<p class="font-medium mb-1">Please check:</p>
							<ul class="list-disc list-inside space-y-1">
								<li>The sensor component name matches your robot configuration</li>
								<li>Your robot is running and accessible</li>
								<li>The SO-101 arm is properly connected</li>
								<li>You have proper authentication (check browser cookies)</li>
							</ul>
						</div>
					</div>
				</Alert>
			{/if}
		</div>
	</div>

	<!-- Help Section -->
	<div class="mt-8 bg-blue-50 rounded-lg p-6">
		<h3 class="text-lg font-medium text-blue-900 mb-3">Need Help?</h3>
		<div class="text-blue-800 space-y-2">
			<p>
				<strong>Finding your sensor component name:</strong> Check your robot configuration in the Viam
				app. Look for the sensor component with model "devrel:so101:calibration".
			</p>
			<p>
				<strong>Common sensor names:</strong> "calibrator", "so101-calibration", "arm-calibrator"
			</p>
		</div>
	</div>
</div>
