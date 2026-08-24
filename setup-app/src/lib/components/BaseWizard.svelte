<script lang="ts">
	import type { WorkflowStep, WorkflowType } from '$lib/types';

	interface Props {
		workflowType: WorkflowType;
		steps: WorkflowStep[];
		stepTitles: Record<string, string>;
		currentStep: number;
		error: string | null;
		canAdvance: boolean;
		onNextStep: () => void;
		onPrevStep: () => void;
		onGoToStep: (stepIndex: number) => void;
		onClearError: () => void;
		children: any;
	}

	let {
		workflowType,
		steps,
		stepTitles,
		currentStep,
		error,
		canAdvance,
		onNextStep,
		onPrevStep,
		onGoToStep,
		onClearError,
		children
	}: Props = $props();

	// Unique per instance: two wizards mounted at once must not share the hint's id.
	// ($props.id() has to be its own declaration's whole initializer.)
	const uid = $props.id();
	const hintId = `${uid}-next-step-hint`;

	// Computed values
	const currentStepName = $derived(steps[currentStep]);
	const totalSteps = steps.length;
	const progressPercentage = $derived(Math.round(((currentStep + 1) / totalSteps) * 100));
	const isLastStep = $derived(currentStep === steps.length - 1);
	const nextDisabled = $derived(isLastStep || !canAdvance);

	// Workflow type display names
	const workflowDisplayNames = {
		'motor-setup': 'Motor Setup',
		calibration: 'Calibration',
		'full-setup': 'Complete Setup'
	};

	const workflowDisplayName = workflowDisplayNames[workflowType] || workflowType;
</script>

<div class="bg-white rounded-lg shadow-lg overflow-hidden">
	<!-- Progress Header -->
	<div class="bg-gray-50 px-6 py-4 border-b">
		<div class="flex items-center justify-between mb-4">
			<div>
				<h2 class="text-2xl font-semibold text-gray-900">
					{stepTitles[currentStepName]}
				</h2>
				<p class="text-sm text-gray-600 mt-1">
					{workflowDisplayName} Workflow
				</p>
			</div>
			<div class="text-sm text-gray-600">
				Step {currentStep + 1} of {totalSteps}
			</div>
		</div>

		<!-- Progress Bar -->
		<div class="w-full bg-gray-200 rounded-full h-2">
			<div
				class="bg-blue-600 h-2 rounded-full transition-all duration-300 ease-in-out"
				style="width: {progressPercentage}%"
			></div>
		</div>
		<div class="text-right text-sm text-gray-600 mt-1">
			{progressPercentage}% complete
		</div>

		<!-- Step Navigation Breadcrumbs -->
		<div class="flex flex-wrap gap-2 mt-4">
			{#each steps as step, index}
				<button
					onclick={() => onGoToStep(index)}
					disabled={index > currentStep}
					class="text-xs px-2 py-1 rounded-full border transition-colors duration-200 {index ===
					currentStep
						? 'bg-blue-600 text-white border-blue-600'
						: index < currentStep
							? 'bg-green-100 text-green-800 border-green-300 hover:bg-green-200'
							: 'bg-gray-100 text-gray-500 border-gray-300 cursor-not-allowed'}"
				>
					{index + 1}. {stepTitles[step]}
				</button>
			{/each}
		</div>
	</div>

	<!-- Error Display -->
	{#if error}
		<div class="bg-red-50 border-l-4 border-red-400 p-4 mx-6 mt-4">
			<div class="flex items-start">
				<div class="flex-shrink-0">
					<svg class="h-5 w-5 text-red-400" viewBox="0 0 20 20" fill="currentColor">
						<path
							fill-rule="evenodd"
							d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z"
							clip-rule="evenodd"
						/>
					</svg>
				</div>
				<div class="ml-3">
					<p class="text-sm text-red-800">{error}</p>
					<button
						onclick={onClearError}
						class="mt-2 text-sm text-red-600 hover:text-red-500 underline"
					>
						Dismiss
					</button>
				</div>
			</div>
		</div>
	{/if}

	<!-- Step Content -->
	<div class="p-6">
		{@render children()}
	</div>

	<!-- Navigation Footer -->
	<div class="bg-gray-50 px-6 py-4 border-t flex justify-between items-center">
		<button
			onclick={onPrevStep}
			disabled={currentStep === 0}
			class="px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50 disabled:cursor-not-allowed"
		>
			← Previous
		</button>

		<div class="flex flex-col items-end">
			<div class="text-sm text-gray-600">
				{currentStep + 1} / {totalSteps}
			</div>
			{#if !isLastStep && !canAdvance}
				<p id={hintId} class="mt-1 text-xs text-gray-500">Finish this step to continue</p>
			{/if}
		</div>

		<button
			onclick={onNextStep}
			disabled={nextDisabled}
			title={nextDisabled && !isLastStep ? 'Finish this step to continue' : undefined}
			aria-describedby={nextDisabled && !isLastStep ? hintId : undefined}
			class="px-4 py-2 text-sm font-medium text-white bg-blue-600 border border-transparent rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50 disabled:cursor-not-allowed"
		>
			Next →
		</button>
	</div>
</div>
