<script lang="ts">
	import type { SensorConfig, WorkflowType, WorkflowInfo } from '$lib/types';
	import { Button } from '$lib/components/ui';

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
			steps: 3,
			stepNames: '3 steps: Overview → Motor Setup → Verification'
		},
		{
			id: 'calibration',
			title: 'Calibration Only',
			description:
				'Set homing positions and record joint ranges. Choose this if your motors are already configured.',
			duration: '~10-15 minutes',
			steps: 5,
			stepNames: '5 steps: Overview → Start → Homing → Recording → Save'
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

	function getWorkflowIcon(workflowId: WorkflowType): string {
		switch (workflowId) {
			case 'motor-setup':
				return 'M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z M15 12a3 3 0 11-6 0 3 3 0 016 0z';
			case 'calibration':
				return 'M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z';
			case 'full-setup':
				return 'M19 14l-7 7m0 0l-7-7m7 7V3';
			default:
				return '';
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
			<svg class="w-6 h-6 text-green-600 mr-2" fill="currentColor" viewBox="0 0 20 20">
				<path
					fill-rule="evenodd"
					d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
					clip-rule="evenodd"
				></path>
			</svg>
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
				<div class="bg-gradient-to-r {getWorkflowColor(workflow.id)} p-6 text-black">
					<div class="flex items-center mb-4">
						<div
							class="w-12 h-12 bg-white bg-opacity-20 rounded-lg flex items-center justify-center mr-4"
						>
							<svg class="w-6 h-6" fill="currentColor" viewBox="0 0 24 24">
								<path fill-rule="evenodd" d={getWorkflowIcon(workflow.id)} clip-rule="evenodd"
								></path>
							</svg>
						</div>
						<div>
							<h3 class="text-xl font-bold">{workflow.title}</h3>
							<p class="text-white text-opacity-90 text-sm">{workflow.duration}</p>
						</div>
					</div>
				</div>

				<!-- Card Content -->
				<div class="p-6">
					<p class="text-gray-600 mb-4 leading-relaxed">{workflow.description}</p>

					<div class="mb-6">
						<div class="flex items-center text-sm text-gray-500 mb-2">
							<svg class="w-4 h-4 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
								<path
									stroke-linecap="round"
									stroke-linejoin="round"
									stroke-width="2"
									d="M9 5H7a2 2 0 00-2 2v10a2 2 0 002 2h8a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2"
								></path>
							</svg>
							{workflow.stepNames}
						</div>
					</div>

					<Button
						onclick={() => onWorkflowSelected(workflow.id)}
						variant="primary"
						size="lg"
						className="w-full"
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
