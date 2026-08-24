<script lang="ts">
	import { onMount } from 'svelte';
	import { goto } from '$app/navigation';
	import SensorConfigForm from '$lib/components/SensorConfigForm.svelte';
	import WorkflowSelector from '$lib/components/WorkflowSelector.svelte';
	import type { SensorConfig, WorkflowType } from '$lib/types';
	import { getMachineRootPath, parseConnectionFromCookies } from '$lib/utils/connection';
	import { useWorkflowConfig } from '$lib/composables/useWorkflowConfig';

	const { getSensorConfigFromSession } = useWorkflowConfig();

	// Landing page state
	let sensorConfig = $state<SensorConfig | null>(null);

	// Handle successful sensor configuration
	function handleConfigValid(config: SensorConfig) {
		sensorConfig = config;

		const { machineId } = parseConnectionFromCookies();
		// Store config in sessionStorage for workflow pages to use
		const sessionState = {
			sensorConfig: config,
			completedWorkflows: [],
			timestamp: Date.now()
		};
		sessionStorage.setItem(`so101-setup-state-${machineId}`, JSON.stringify(sessionState));
	}

	// Handle workflow selection
	function handleWorkflowSelected(workflow: WorkflowType) {
		if (!sensorConfig) return;

		// Navigate to selected workflow with sensor config as URL params
		const params = new URLSearchParams({
			part: sensorConfig.partId,
			sensor: sensorConfig.sensorName
		});

		goto(`${getMachineRootPath()}workflows/${workflow}?${params}`);
	}

	// Check for existing session state on page load
	function checkExistingSession() {
		sensorConfig = getSensorConfigFromSession();
	}

	// Initialize once on mount
	onMount(() => {
		checkExistingSession();
	});
</script>

<svelte:head>
	<title>SO-101 Setup</title>
	<meta name="description" content="Configure and calibrate your SO-101 robotic arm" />
</svelte:head>

<div class="container mx-auto px-4 py-8">
	<!-- Main Content -->
	{#if !sensorConfig}
		<!-- Step 1: Sensor Configuration -->
		<SensorConfigForm onConfigValid={handleConfigValid} />
	{:else}
		<!-- Step 2: Workflow Selection -->
		<WorkflowSelector {sensorConfig} onWorkflowSelected={handleWorkflowSelected} />
	{/if}

	<!-- Back to Configuration Button (only show in workflow selection) -->
	{#if sensorConfig}
		<div class="text-center mt-8">
			<button
				onclick={() => (sensorConfig = null)}
				class="text-blue-600 hover:text-blue-700 text-sm font-medium focus:outline-none focus:underline"
			>
				← Change Sensor Configuration
			</button>
		</div>
	{/if}
</div>
