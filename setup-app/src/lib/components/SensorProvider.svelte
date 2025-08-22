<script lang="ts">
	import { setContext } from 'svelte';
	import { SensorClient, Struct } from '@viamrobotics/sdk';
	import {
		createResourceClient,
		createResourceQuery,
		createResourceMutation
	} from '@viamrobotics/svelte-sdk';
	import type { DoCommandResponse, SensorConfig, SensorContext } from '$lib/types';
	import { logger } from '$lib/utils/logger';

	interface Props {
		sensorConfig: SensorConfig;
		children: any;
	}

	let { sensorConfig, children }: Props = $props();

	// Create sensor client using dynamic configuration
	const sensorClient = createResourceClient(
		SensorClient,
		() => sensorConfig.partId,
		() => sensorConfig.sensorName
	);

	// Create reactive query for sensor readings (1 second interval)
	const sensorReadings = createResourceQuery(sensorClient, 'getReadings', undefined, {
		refetchInterval: 1000
	});

	// Create mutation for DoCommand calls
	const doCommand = createResourceMutation(sensorClient, 'doCommand');

	logger.info('SensorProvider initialized', {
		partId: sensorConfig.partId,
		sensorName: sensorConfig.sensorName
	});

	// Helper function to send commands with error handling
	const sendCommand = async (cmd: Record<string, any>): Promise<DoCommandResponse> => {
		try {
			logger.debug('Sending sensor command', cmd);
			const result = await doCommand.current.mutateAsync([Struct.fromJson(cmd)]);

			// Type-safe response handling
			const response = result as DoCommandResponse;

			if (response && !response.success) {
				const errorMsg = response.error || 'Command failed';
				logger.error('Sensor command failed', new Error(errorMsg));
				throw new Error(errorMsg);
			}

			logger.debug('Sensor command successful', response);
			return response;
		} catch (error) {
			// Transform technical errors into user-friendly messages
			const errorMsg = getUserFriendlyError(error);
			logger.error('Sensor command error', error as Error);
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

	// Create sensor context for child components
	const sensorContext: SensorContext = {
		sensorClient: sensorClient as { current: SensorClient },
		sensorReadings,
		doCommand,
		sendCommand,
		sensorConfig
	};

	// Set context for child components to consume
	setContext('sensor', sensorContext);
</script>

<!-- Render children with sensor context available -->
{@render children()}
