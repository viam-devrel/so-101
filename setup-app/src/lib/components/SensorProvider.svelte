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

	// Create mutation for DoCommand calls
	const doCommand = createResourceMutation(sensorClient, 'doCommand');

	// getReadings only reads the sensor's in-memory struct under its RLock -- it never touches
	// the serial bus -- so the interval trades network/CPU against UI latency, nothing more.
	// range_recording: the user watches live per-joint progress, so poll near-continuously.
	const RANGE_RECORDING_INTERVAL_MS = 300;
	// started / homing_position: mid-workflow, no continuous data, but still moving forward.
	const ACTIVE_INTERVAL_MS = 1000;
	// idle / completed / error / unknown: nothing changes without a user action.
	const IDLE_INTERVAL_MS = 3000;

	// Last calibration_state seen, tracked separately from sensorReadings so the query's own
	// options callback can read it without referencing sensorReadings before it exists.
	let lastCalibrationState = $state<string | undefined>(undefined);

	// Deliberately never returns false while a DoCommand is in flight. The SDK's poller waits
	// for each round trip before rearming, and Readings' RLock already queues behind the
	// command's write lock, so an in-flight command cannot pile reads up. Pausing instead
	// freezes every step's auto-advance on stale readings for as long as the mutation is
	// pending -- permanently if it never settles -- and each interval change tears down and
	// rearms the SDK's polling effect (see the leak noted below).
	function refetchIntervalFor(state: string | undefined): number {
		if (state === 'range_recording') return RANGE_RECORDING_INTERVAL_MS;
		if (state === 'started' || state === 'homing_position') return ACTIVE_INTERVAL_MS;
		return IDLE_INTERVAL_MS;
	}

	// An options *function* is what makes the interval adaptive: createResourceQuery forces
	// refetchInterval: false into TanStack and polls via its own usePolling effect, which
	// re-reads options() reactively. Keep interval changes rare -- usePolling's poll loop
	// reschedules itself after its await, so a change landing mid-round-trip orphans a poll
	// chain that outlives the effect (upstream bug in @viamrobotics/svelte-sdk).
	const sensorReadings = createResourceQuery(sensorClient, 'getReadings', undefined, () => ({
		refetchInterval: refetchIntervalFor(lastCalibrationState)
	}));

	$effect(() => {
		const state = sensorReadings.current.data?.calibration_state;
		if (typeof state === 'string') {
			lastCalibrationState = state;
		}
	});

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
