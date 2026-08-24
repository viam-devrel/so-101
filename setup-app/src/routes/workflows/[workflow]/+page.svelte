<script lang="ts">
	import { goto } from '$app/navigation';
	import { page } from '$app/state';
	import SensorProvider from '$lib/components/SensorProvider.svelte';
	import Wizard from '$lib/components/Wizard.svelte';
	import { useWorkflowConfig } from '$lib/composables/useWorkflowConfig';
	import { getMachineRootPath } from '$lib/utils/connection';
	import { logger } from '$lib/utils/logger';
	import { Icon } from '$lib/components/ui';
	import type { IconName } from '$lib/icons';
	import type { SensorConfig, WorkflowType } from '$lib/types';

	interface WorkflowPageConfig {
		documentTitle: string;
		metaDescription: string;
		headerLabel: string;
		headerIconClass: string;
		headerIcon: IconName;
		spinnerBorderClass: string;
		loadingTitle: string;
		logNoun: string;
	}

	// Route-display chrome that used to differ across the three +page.svelte files (title,
	// icon, colour, one log string). Kept local to the route rather than in wizardProgress.ts,
	// since it's page cosmetics, not step/progress data. Typed as Record<WorkflowType, ...> so
	// TS itself enforces all three (and only those three) are covered -- this doubles as the
	// route's set of valid workflow values, see pageConfigFor below.
	const PAGE_CONFIG: Record<WorkflowType, WorkflowPageConfig> = {
		'motor-setup': {
			documentTitle: 'Motor Setup - SO-101',
			metaDescription: 'Motor setup workflow for SO-101 arm',
			headerLabel: 'Motor Setup Workflow',
			headerIconClass: 'w-6 h-6 text-orange-600 mr-2',
			headerIcon: 'gear',
			spinnerBorderClass: 'border-orange-600',
			loadingTitle: 'Loading Motor Setup',
			logNoun: 'motor setup'
		},
		calibration: {
			documentTitle: 'Calibration - SO-101',
			metaDescription: 'Calibration workflow for SO-101 arm',
			headerLabel: 'Calibration Workflow',
			headerIconClass: 'w-6 h-6 text-green-600 mr-2',
			headerIcon: 'checkCircle',
			spinnerBorderClass: 'border-green-600',
			loadingTitle: 'Loading Calibration',
			logNoun: 'calibration'
		},
		'full-setup': {
			documentTitle: 'Complete Setup - SO-101',
			metaDescription: 'Complete setup workflow for SO-101 arm',
			headerLabel: 'Complete Setup Workflow',
			headerIconClass: 'w-6 h-6 text-blue-600 mr-2',
			headerIcon: 'arrowDown',
			spinnerBorderClass: 'border-blue-600',
			loadingTitle: 'Loading Complete Setup',
			logNoun: 'full setup'
		}
	};

	function pageConfigFor(param: string | undefined): WorkflowPageConfig | undefined {
		if (!param) return undefined;
		return (PAGE_CONFIG as Record<string, WorkflowPageConfig>)[param];
	}

	// Sensor configuration state
	let sensorConfig = $state<SensorConfig | null>(null);
	let configError = $state<string | null>(null);

	const { initializeSensorConfig } = useWorkflowConfig();

	const workflowParam = $derived(page.params.workflow);
	const config = $derived(pageConfigFor(workflowParam));
	const workflow = $derived(workflowParam as WorkflowType);

	// Re-runs whenever the URL's workflow segment resolves to a different config -- covers
	// both the initial mount (onMount elsewhere in this app) and navigating directly between
	// two workflow URLs, which (unlike the old one-route-per-workflow pages) reuses this same
	// component instance rather than remounting it. Reads only `config` and writes only
	// sensorConfig/configError, so it has no self-dependency and settles in one run per change.
	// The null reset is for the failure path (a second run that resolves nothing must not leave
	// the previous config mounted); remounting <Wizard> is the {#key} block's job below, since
	// both writes land in one flush and the template never observes the intermediate null.
	$effect(() => {
		const currentConfig = config;
		sensorConfig = null;
		configError = null;
		if (!currentConfig) return;

		logger.info(`Initializing ${currentConfig.logNoun} workflow configuration`);
		const { sensorConfig: resolved, source } = initializeSensorConfig();

		if (resolved) {
			sensorConfig = resolved;
			logger.debug('Sensor config loaded from', source);
		} else {
			configError = 'No sensor configuration found. Please configure your sensor first.';
			logger.warn('No sensor configuration available');
		}
	});

	// Handle navigation back to landing page
	function goToLandingPage() {
		goto(getMachineRootPath());
	}
</script>

<svelte:head>
	<title>{config?.documentTitle ?? 'SO-101 Setup'}</title>
	<meta name="description" content={config?.metaDescription ?? 'SO-101 arm setup workflow'} />
</svelte:head>

<div class="container mx-auto px-4 py-8">
	{#if config}
		<!-- Header -->
		<div class="text-center mb-6">
			<div class="flex items-center justify-center mb-2">
				<Icon name={config.headerIcon} className={config.headerIconClass} />
				<span class="text-lg font-medium text-gray-700">{config.headerLabel}</span>
			</div>
		</div>

		{#if configError}
			<!-- Configuration Error -->
			<div class="max-w-2xl mx-auto">
				<div class="bg-white rounded-lg shadow-lg p-8 text-center">
					<div
						class="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
					>
						<Icon name="warningTriangle" className="w-8 h-8 text-red-600" />
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
			<!-- Wizard -->
			<div class="max-w-4xl mx-auto">
				<SensorProvider {sensorConfig}>
					<!-- Keyed on the workflow, not just passed as a prop: SvelteKit reuses this route
					     component across /workflows/<a> -> /workflows/<b>, and Wizard reads its step
					     list from WORKFLOW_DEFINITIONS once at init. Without the key the instance is
					     reused and keeps the previous workflow's steps (and its currentStep). The key
					     is on Wizard alone so SensorProvider's polling query survives the switch. -->
					{#key workflow}
						<Wizard {workflow} />
					{/key}
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
						class="animate-spin rounded-full h-12 w-12 border-b-2 {config.spinnerBorderClass} mx-auto mb-4"
					></div>
					<h2 class="text-xl font-semibold text-gray-900 mb-2">{config.loadingTitle}</h2>
					<p class="text-gray-600">Initializing workflow...</p>
				</div>
			</div>
		{/if}
	{:else}
		<!-- Unknown workflow: not one of PAGE_CONFIG's keys -->
		<div class="max-w-2xl mx-auto">
			<div class="bg-white rounded-lg shadow-lg p-8 text-center">
				<div
					class="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4"
				>
					<Icon name="warningTriangle" className="w-8 h-8 text-red-600" />
				</div>
				<h2 class="text-xl font-semibold text-gray-900 mb-4">Unknown Workflow</h2>
				<p class="text-gray-600 mb-6">
					"{workflowParam}" isn't a recognized setup workflow. Please choose one from the workflow
					selection screen.
				</p>
				<button
					onclick={goToLandingPage}
					class="px-6 py-3 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500"
				>
					Back to Workflow Selection
				</button>
			</div>
		</div>
	{/if}
</div>
