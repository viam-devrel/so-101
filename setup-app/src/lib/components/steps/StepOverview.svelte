<script lang="ts">
	import type { StepProps, WorkflowType } from '$lib/types';

	interface Props extends StepProps {
		workflowType?: WorkflowType;
	}

	let { nextStep, workflowType = 'full-setup' }: Props = $props();

	// Workflow-specific content
	const workflowContent = {
		'motor-setup': {
			title: 'Motor Setup Wizard',
			description:
				'This wizard will guide you through configuring and verifying your SO-101 servo motors.',
			accomplishments: [
				{
					title: 'Motor Configuration',
					description: 'Set unique servo IDs for each motor',
					color: 'orange'
				},
				{
					title: 'Communication Verification',
					description: 'Test motor connectivity and responsiveness',
					color: 'orange'
				}
			],
			duration: '5-10 minutes',
			steps: '3 steps'
		},
		calibration: {
			title: 'Calibration Wizard',
			description:
				'This wizard will guide you through calibrating your SO-101 arm joints and ranges.',
			accomplishments: [
				{
					title: 'Homing Position',
					description: 'Set reference "zero" positions for each joint',
					color: 'green'
				},
				{
					title: 'Range Recording',
					description: 'Capture safe movement limits for all joints',
					color: 'green'
				}
			],
			duration: '10-15 minutes',
			steps: '5 steps'
		},
		'full-setup': {
			title: 'Complete Setup Wizard',
			description:
				'This wizard will guide you through the complete setup and calibration process for your SO-101 robotic arm.',
			accomplishments: [
				{
					title: 'Motor Configuration',
					description: 'Set unique servo IDs for each motor',
					color: 'blue'
				},
				{
					title: 'Communication Verification',
					description: 'Test motor connectivity and responsiveness',
					color: 'blue'
				},
				{
					title: 'Homing Position',
					description: 'Set reference "zero" positions for each joint',
					color: 'blue'
				},
				{
					title: 'Range Recording',
					description: 'Capture safe movement limits for all joints',
					color: 'blue'
				}
			],
			duration: '15-25 minutes',
			steps: '8 steps'
		}
	};

	const content = workflowContent[workflowType];
</script>

<div class="max-w-4xl mx-auto">
	<!-- Welcome Section -->
	<div class="text-center mb-8">
		<h3 class="text-2xl font-bold text-gray-900 mb-4">Welcome to the {content.title}</h3>
		<p class="text-lg text-gray-600">
			{content.description}
		</p>
		<div class="mt-4 text-sm text-gray-500">
			Estimated time: {content.duration} • {content.steps}
		</div>
	</div>

	<!-- What You'll Do Section -->
	<div class="mb-8">
		<h4 class="text-xl font-semibold text-gray-900 mb-4">What You'll Accomplish:</h4>
		<div
			class="grid md:grid-cols-{content.accomplishments.length > 2
				? '2'
				: content.accomplishments.length} gap-6"
		>
			{#each content.accomplishments as accomplishment}
				<div class="bg-{accomplishment.color}-50 p-6 rounded-lg">
					<h5 class="font-semibold text-{accomplishment.color}-900 mb-3">{accomplishment.title}</h5>
					<p class="text-{accomplishment.color}-800 text-sm">{accomplishment.description}</p>
				</div>
			{/each}
		</div>
	</div>

	<!-- Safety Warning Section -->
	<div class="bg-amber-50 border-l-4 border-amber-400 p-6 mb-8">
		<div class="flex items-start">
			<div class="flex-shrink-0">
				<svg class="h-6 w-6 text-amber-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
					<path
						stroke-linecap="round"
						stroke-linejoin="round"
						stroke-width="2"
						d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"
					></path>
				</svg>
			</div>
			<div class="ml-3">
				<h4 class="text-lg font-semibold text-amber-900 mb-3">⚠️ Important Safety Information</h4>
				<div class="text-amber-800 space-y-2">
					<p class="font-medium">Before proceeding, please ensure:</p>
					<ul class="list-disc list-inside space-y-1 text-sm">
						<li>The SO-101 arm is securely mounted to a stable surface</li>
						<li>The workspace is clear of obstacles and people</li>
						<li>You have easy access to the emergency stop button</li>
						<li>All servo cables are properly connected</li>
						<li>Power supply is connected and stable</li>
					</ul>
				</div>
			</div>
		</div>
	</div>

	<!-- Workspace Requirements -->
	<div class="bg-gray-50 p-6 rounded-lg mb-8">
		<h4 class="text-lg font-semibold text-gray-900 mb-4">Workspace Requirements</h4>
		<div class="grid md:grid-cols-2 gap-6">
			<div>
				<h5 class="font-medium text-gray-800 mb-3">Physical Setup:</h5>
				<ul class="text-gray-700 space-y-1 text-sm">
					<li>• Minimum 1.5m clearance around the arm</li>
					<li>• Stable, level mounting surface</li>
					<li>• Good lighting for visual inspection</li>
					<li>• Access to all sides of the arm</li>
				</ul>
			</div>

			<div>
				<h5 class="font-medium text-gray-800 mb-3">During Calibration:</h5>
				<ul class="text-gray-700 space-y-1 text-sm">
					<li>• Motors will be disabled (no holding torque)</li>
					<li>• You'll manually move joints to their limits</li>
					<li>• Process takes approximately 5-10 minutes</li>
					<li>• Smooth, controlled movements are essential</li>
				</ul>
			</div>
		</div>
	</div>

	<!-- Prerequisites Section -->
	<div class="bg-white border border-gray-200 p-6 rounded-lg mb-8">
		<h4 class="text-lg font-semibold text-gray-900 mb-4">Prerequisites Checklist</h4>
		<div class="space-y-3">
			<div class="flex items-start">
				<div
					class="flex-shrink-0 w-5 h-5 bg-blue-100 rounded-full flex items-center justify-center mt-0.5"
				>
					<svg class="w-3 h-3 text-blue-600" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
							clip-rule="evenodd"
						/>
					</svg>
				</div>
				<p class="ml-3 text-gray-700">SO-101 arm is physically assembled and mounted</p>
			</div>
			<div class="flex items-start">
				<div
					class="flex-shrink-0 w-5 h-5 bg-blue-100 rounded-full flex items-center justify-center mt-0.5"
				>
					<svg class="w-3 h-3 text-blue-600" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
							clip-rule="evenodd"
						/>
					</svg>
				</div>
				<p class="ml-3 text-gray-700">USB serial connection established to robot computer</p>
			</div>
			<div class="flex items-start">
				<div
					class="flex-shrink-0 w-5 h-5 bg-blue-100 rounded-full flex items-center justify-center mt-0.5"
				>
					<svg class="w-3 h-3 text-blue-600" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
							clip-rule="evenodd"
						/>
					</svg>
				</div>
				<p class="ml-3 text-gray-700">SO-101 calibration sensor configured in robot config</p>
			</div>
			<div class="flex items-start">
				<div
					class="flex-shrink-0 w-5 h-5 bg-blue-100 rounded-full flex items-center justify-center mt-0.5"
				>
					<svg class="w-3 h-3 text-blue-600" fill="currentColor" viewBox="0 0 20 20">
						<path
							fill-rule="evenodd"
							d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
							clip-rule="evenodd"
						/>
					</svg>
				</div>
				<p class="ml-3 text-gray-700">Power supply connected and servo motors receiving power</p>
			</div>
		</div>
	</div>

	<!-- Time Estimate -->
	<div class="bg-blue-50 p-4 rounded-lg mb-8">
		<div class="flex items-center">
			<svg class="w-5 h-5 text-blue-600 mr-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
				<path
					stroke-linecap="round"
					stroke-linejoin="round"
					stroke-width="2"
					d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z"
				></path>
			</svg>
			<span class="text-blue-900 font-medium"> Estimated completion time: 15-20 minutes </span>
		</div>
	</div>

	<!-- Action Button -->
	<div class="text-center">
		<button
			onclick={nextStep}
			class="inline-flex items-center px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 transition-colors duration-200"
		>
			Begin Setup Process
			<svg class="ml-2 -mr-1 w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
				<path
					fill-rule="evenodd"
					d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z"
					clip-rule="evenodd"
				/>
			</svg>
		</button>
		<p class="mt-2 text-sm text-gray-600">
			You can navigate back to this page at any time using the progress bar above.
		</p>
	</div>
</div>
