<script lang="ts">
	import { SensorClient } from '@viamrobotics/sdk';
	import {
		createResourceClient,
		createResourceQuery,
		useResourceNames
	} from '@viamrobotics/svelte-sdk';
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

	// Form validation
	const validation = $derived.by(() => validateSensorConfig(partId.trim(), sensorName.trim()));
	const isFormValid = $derived(validation.isValid);

	// Every value components/calibration/sensor.go's CalibrationState.String() can return,
	// including its default 'unknown' branch. Keep in sync: a state added there but missing
	// here would reject the real sensor.
	const CALIBRATION_STATES = new Set([
		'idle',
		'started',
		'homing_position',
		'range_recording',
		'completed',
		'error',
		'unknown'
	]);

	// Confirms a reading came from devrel:so101:calibration (components/calibration/sensor.go's
	// Readings), not just "some sensor returned data". Any one of these fields could plausibly
	// appear on an unrelated sensor; requiring all three together is what rules out a false
	// positive.
	function looksLikeCalibrationSensor(data: unknown): boolean {
		if (!data || typeof data !== 'object') return false;
		const d = data as Record<string, unknown>;
		return (
			typeof d.calibration_state === 'string' &&
			CALIBRATION_STATES.has(d.calibration_state) &&
			Array.isArray(d.available_commands) &&
			typeof d.joints === 'object' &&
			d.joints !== null
		);
	}

	const resources = useResourceNames(() => partId, 'sensor');

	// Auto-select the only available sensor
	$effect(() => {
		const names = resources.current;
		if (!sensorName && names?.length === 1) {
			sensorName = names[0].name;
		}
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
		if (!validation.isValid) {
			connectionError = validation.error!;
			connectionStatus = 'error';
			return;
		}

		isTestingConnection = true;
		connectionStatus = 'testing';
		connectionError = null;

		let timeoutId: ReturnType<typeof setTimeout> | undefined;
		try {
			logger.info('Testing sensor connection', {
				partId: partId.trim(),
				sensorName: sensorName.trim()
			});

			const timeout = new Promise<never>((_, reject) => {
				timeoutId = setTimeout(() => reject(new Error('Connection test timed out')), 5000);
			});

			const res = await Promise.race([sensorQuery.current.refetch(), timeout]);

			if (res.error) {
				throw res.error;
			}
			if (!res.data) {
				throw new Error('Connection test failed - no data received');
			}

			if (!looksLikeCalibrationSensor(res.data)) {
				throw new Error(
					'That sensor responded, but it is not a devrel:so101:calibration component (unexpected reading shape).'
				);
			}

			logger.info('Connection test successful', res.data);
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
			clearTimeout(timeoutId);
			isTestingConnection = false;
		}
	}
</script>

<div class="max-w-2xl mx-auto">
	<!-- Header -->
	<div class="text-center mb-8">
		<h1 class="text-4xl font-bold text-gray-900 mb-4">SO-101 Arm Setup</h1>
		<p class="text-xl text-gray-600">
			Connect to the `devrel:so101:calibration` sensor to get started
		</p>
	</div>

	<!-- Help Section -->
	<div class="mt-8 bg-blue-50 rounded-lg p-6 mb-4">
		<div class="text-blue-800 space-y-2">
			<p>
				<strong>Finding your sensor component name:</strong> Check your robot configuration in the Viam
				app. Look for the sensor component with model "devrel:so101:calibration".
			</p>
			<p>
				<strong>Don't see any sensors?</strong> Add a component with model "devrel:so101:calibration"
				to this machine's config in the Viam app, then reload this page.
			</p>
			<p>
				<strong>Not sure which one is right?</strong> Pick a candidate and press "Test Connection" —
				it checks the sensor's response shape and tells you if it isn't the calibration sensor.
			</p>
		</div>
	</div>

	<div class="bg-white rounded-lg shadow-lg p-8">
		<h2 class="text-2xl font-semibold text-gray-900 mb-6">Calibration Sensor Configuration</h2>

		<div class="space-y-6">
			<div>
				<label for="sensorName" class="block text-sm font-medium text-gray-700 mb-2">
					Calibration Sensor Component Name <span class="text-red-500">*</span>
				</label>
				<select
					id="sensorName"
					bind:value={sensorName}
					class="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
				>
					<option value="" disabled selected={!sensorName}>
						Select an available sensor component
					</option>
					{#if resources.current && resources.current.length > 0}
						{#each resources.current as resource}
							<option value={resource.name}>{resource.name}</option>
						{/each}
					{:else if resources.loading}
						<option value="" disabled>Searching for possible resources...</option>
					{/if}
				</select>
				{#if !resources.loading && (resources.current?.length ?? 0) === 0}
					<p class="mt-1 text-sm text-red-500">
						No sensor components found on currently connected machine.
					</p>
				{:else if !resources.loading}
					<p class="mt-1 text-sm text-gray-600">
						Select the name of your SO-101 calibration sensor component as configured in your robot
					</p>
				{/if}
				{#if sensorName && validation.error}
					<p class="mt-1 text-sm text-red-500">{validation.error}</p>
				{/if}
			</div>

			<div>
				<Button
					onclick={testConnection}
					disabled={!isFormValid || isTestingConnection || connectionStatus === 'success'}
					variant="primary"
					size="lg"
					fullWidth
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
								<li>
									The selected component's model is "devrel:so101:calibration", not another sensor
								</li>
								<li>The calibration sensor component name matches your robot configuration</li>
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
</div>
