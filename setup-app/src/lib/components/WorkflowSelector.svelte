<script lang="ts">
	import type { SensorConfig, WorkflowType, WorkflowInfo } from '$lib/types';
	import { Button, Icon } from '$lib/components/ui';
	import type { IconName } from '$lib/icons';

	interface Props {
		sensorConfig: SensorConfig;
		onWorkflowSelected: (workflow: WorkflowType) => void;
	}

	let { sensorConfig, onWorkflowSelected }: Props = $props();

	const workflows: WorkflowInfo[] = [
		{
			id: 'motor-setup',
			title: 'Motor Setup Only',
			description:
				'Configure servo motor IDs and verify communication. Choose this if your motors need to be set up or reconfigured.',
			duration: '~5-10 minutes',
			steps: 4,
			stepNames: '4 steps: Overview → Motor Setup → Verification → Complete'
		},
		{
			id: 'calibration',
			title: 'Calibration Only',
			description:
				'Set homing positions and record joint ranges. Choose this if your motors are already configured.',
			duration: '~10-15 minutes',
			steps: 6,
			stepNames: '6 steps: Overview → Start → Homing → Recording → Save → Complete'
		},
		{
			id: 'full-setup',
			title: 'Complete Setup',
			description:
				'Full end-to-end setup from motor configuration through calibration. Choose this for new arms.',
			duration: '~15-25 minutes',
			steps: 8,
			stepNames: '8 steps: Complete motor setup + calibration process'
		}
	];

	// Was raw heroicons-OUTLINE path data rendered with fill="currentColor" -- silently dropped
	// the thin interior strokes (e.g. the gear's inner circle) into a blob. Icon.svelte picks
	// fill-vs-stroke from icons.ts, so it can't recur.
	function getWorkflowIcon(workflowId: WorkflowType): IconName {
		switch (workflowId) {
			case 'motor-setup':
				return 'gear';
			case 'calibration':
				return 'checkCircle';
			case 'full-setup':
				return 'arrowDown';
		}
	}

	function getWorkflowColor(workflowId: WorkflowType): string {
		switch (workflowId) {
			case 'motor-setup':
				return 'from-orange-400 to-orange-600';
			case 'calibration':
				return 'from-green-400 to-green-600';
			case 'full-setup':
				return 'from-blue-400 to-blue-600';
			default:
				return 'from-gray-400 to-gray-600';
		}
	}
</script>

<div class="max-w-4xl mx-auto">
	<!-- Header -->
	<div class="text-center mb-8">
		<div class="flex items-center justify-center mb-4">
			<Icon name="checkSmall" className="w-6 h-6 text-green-600 mr-2" />
			<span class="text-green-700 font-medium">Connected to "{sensorConfig.sensorName}"</span>
		</div>
		<h1 class="text-4xl font-bold text-gray-900 mb-4">Choose Your Workflow</h1>
		<p class="text-xl text-gray-600">Select the setup process that matches your needs</p>
	</div>

	<!-- Workflow Cards -->
	<div class="grid md:grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
		{#each workflows as workflow}
			<div
				class="bg-white rounded-xl shadow-lg hover:shadow-xl transition-shadow duration-300 overflow-hidden border border-gray-200"
			>
				<!-- Gradient Header -->
				<div class="bg-gradient-to-r {getWorkflowColor(workflow.id)} p-6 text-gray-900">
					<div class="flex items-center mb-4">
						<div class="w-12 h-12 bg-white/20 rounded-lg flex items-center justify-center mr-4">
							<Icon name={getWorkflowIcon(workflow.id)} className="w-6 h-6" />
						</div>
						<div>
							<h3 class="text-xl font-bold">{workflow.title}</h3>
							<p class="text-gray-950 text-sm font-medium">{workflow.duration}</p>
						</div>
					</div>
				</div>

				<!-- Card Content -->
				<div class="p-6">
					<p class="text-gray-600 mb-4 leading-relaxed">{workflow.description}</p>

					<div class="mb-6">
						<div class="flex items-center text-sm text-gray-500 mb-2">
							<Icon name="clipboardList" className="w-4 h-4 mr-2" />
							{workflow.stepNames}
						</div>
					</div>

					<Button
						onclick={() => onWorkflowSelected(workflow.id)}
						variant="primary"
						size="lg"
						fullWidth
					>
						Start {workflow.title}
					</Button>
				</div>
			</div>
		{/each}
	</div>

	<!-- Current Configuration Info -->
	<div class="bg-gray-50 rounded-lg p-6">
		<h3 class="text-lg font-medium text-gray-900 mb-3">Current Configuration</h3>
		<div class="grid sm:grid-cols-2 gap-4 text-sm">
			<div>
				<span class="font-medium text-gray-700">Part ID:</span>
				<span class="ml-2 text-gray-600">{sensorConfig.partId}</span>
			</div>
			<div>
				<span class="font-medium text-gray-700">Sensor Name:</span>
				<span class="ml-2 text-gray-600">{sensorConfig.sensorName}</span>
			</div>
		</div>
	</div>

	<!-- Help Section -->
	<div class="mt-8 bg-blue-50 rounded-lg p-6">
		<h3 class="text-lg font-medium text-blue-900 mb-3">Not sure which workflow to choose?</h3>
		<div class="grid md:grid-cols-2 gap-4 text-blue-800">
			<div>
				<h4 class="font-medium mb-2">Choose Motor Setup if:</h4>
				<ul class="text-sm space-y-1 list-disc list-inside">
					<li>You have a new SO-101 arm</li>
					<li>Motors are not responding</li>
					<li>You need to change motor IDs</li>
					<li>You replaced a servo motor</li>
				</ul>
			</div>
			<div>
				<h4 class="font-medium mb-2">Choose Calibration if:</h4>
				<ul class="text-sm space-y-1 list-disc list-inside">
					<li>Motors are already configured</li>
					<li>You want to re-calibrate joint ranges</li>
					<li>You moved the arm's mounting position</li>
					<li>Calibration data was lost</li>
				</ul>
			</div>
		</div>
	</div>
</div>
