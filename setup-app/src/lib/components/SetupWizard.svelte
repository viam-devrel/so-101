<script lang="ts">
	import type { WorkflowStep, MotorSetupResult } from '$lib/types';
	import StepOverview from './steps/StepOverview.svelte';
	import StepMotorSetup from './steps/StepMotorSetup.svelte';
	import StepMotorVerify from './steps/StepMotorVerify.svelte';
	import StepCalibrationStart from './steps/StepCalibrationStart.svelte';
	import StepCalibrationHoming from './steps/StepCalibrationHoming.svelte';
	import StepCalibrationRecording from './steps/StepCalibrationRecording.svelte';
	import StepCalibrationSave from './steps/StepCalibrationSave.svelte';
	import StepComplete from './steps/StepComplete.svelte';

	// Props from parent component
	interface Props {
		sensorClient: any;
		sensorReadings: any;
		doCommand: any;
		sendCommand: (cmd: any) => Promise<any>;
	}

	let { sensorClient, sensorReadings, doCommand, sendCommand }: Props = $props();

	// Workflow step configuration
	const WORKFLOW_STEPS: WorkflowStep[] = [
		'overview',
		'motor_setup',
		'motor_verify',
		'calibration_start',
		'calibration_homing',
		'calibration_recording',
		'calibration_save',
		'complete'
	];

	const STEP_TITLES = {
		overview: 'Overview & Safety',
		motor_setup: 'Motor Setup',
		motor_verify: 'Motor Verification',
		calibration_start: 'Start Calibration',
		calibration_homing: 'Set Homing Position',
		calibration_recording: 'Record Ranges',
		calibration_save: 'Save Calibration',
		complete: 'Setup Complete'
	};

	// Wizard state management
	let currentStep = $state(0);
	let error = $state<string | null>(null);
	let motorSetupResults = $state<Record<string, MotorSetupResult>>({});

	// Computed values
	const currentStepName = $derived(WORKFLOW_STEPS[currentStep]);
	const totalSteps = WORKFLOW_STEPS.length;
	const progressPercentage = $derived(Math.round(((currentStep + 1) / totalSteps) * 100));

	// Navigation functions
	function nextStep() {
		if (currentStep < WORKFLOW_STEPS.length - 1) {
			currentStep++;
			clearError();
		}
	}

	function prevStep() {
		if (currentStep > 0) {
			currentStep--;
			clearError();
		}
	}

	function goToStep(stepIndex: number) {
		if (stepIndex >= 0 && stepIndex < WORKFLOW_STEPS.length) {
			currentStep = stepIndex;
			clearError();
		}
	}

	// Error handling
	function setError(errorMessage: string | null) {
		error = errorMessage;
	}

	function clearError() {
		error = null;
	}

	// Motor setup results management
	function setMotorSetupResults(results: Record<string, MotorSetupResult>) {
		motorSetupResults = results;
	}

	function updateMotorSetupResult(motorName: string, result: MotorSetupResult) {
		motorSetupResults = {
			...motorSetupResults,
			[motorName]: result
		};
	}

	// Step component props
	const stepProps = $derived({
		sensorClient,
		sensorReadings,
		doCommand,
		sendCommand,
		error,
		setError,
		clearError,
		nextStep,
		prevStep,
		motorSetupResults,
		setMotorSetupResults,
		updateMotorSetupResult
	});
</script>

<div class="bg-white rounded-lg shadow-lg overflow-hidden">
	<!-- Progress Header -->
	<div class="bg-gray-50 px-6 py-4 border-b">
		<div class="flex items-center justify-between mb-4">
			<h2 class="text-2xl font-semibold text-gray-900">
				{STEP_TITLES[currentStepName]}
			</h2>
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
			{#each WORKFLOW_STEPS as step, index}
				<button
					onclick={() => goToStep(index)}
					disabled={index > currentStep}
					class="text-xs px-2 py-1 rounded-full border transition-colors duration-200 {
						index === currentStep
							? 'bg-blue-600 text-white border-blue-600'
							: index < currentStep
							? 'bg-green-100 text-green-800 border-green-300 hover:bg-green-200'
							: 'bg-gray-100 text-gray-500 border-gray-300 cursor-not-allowed'
					}"
				>
					{index + 1}. {STEP_TITLES[step]}
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
						<path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clip-rule="evenodd" />
					</svg>
				</div>
				<div class="ml-3">
					<p class="text-sm text-red-800">{error}</p>
					<button
						onclick={clearError}
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
		{#if currentStepName === 'overview'}
			<StepOverview {...stepProps} />
		{:else if currentStepName === 'motor_setup'}
			<StepMotorSetup {...stepProps} />
		{:else if currentStepName === 'motor_verify'}
			<StepMotorVerify {...stepProps} />
		{:else if currentStepName === 'calibration_start'}
			<StepCalibrationStart {...stepProps} />
		{:else if currentStepName === 'calibration_homing'}
			<StepCalibrationHoming {...stepProps} />
		{:else if currentStepName === 'calibration_recording'}
			<StepCalibrationRecording {...stepProps} />
		{:else if currentStepName === 'calibration_save'}
			<StepCalibrationSave {...stepProps} />
		{:else if currentStepName === 'complete'}
			<StepComplete {...stepProps} />
		{/if}
	</div>

	<!-- Navigation Footer -->
	<div class="bg-gray-50 px-6 py-4 border-t flex justify-between items-center">
		<button
			onclick={prevStep}
			disabled={currentStep === 0}
			class="px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50 disabled:cursor-not-allowed"
		>
			← Previous
		</button>

		<div class="text-sm text-gray-600">
			{currentStep + 1} / {totalSteps}
		</div>

		<button
			onclick={nextStep}
			disabled={currentStep === WORKFLOW_STEPS.length - 1}
			class="px-4 py-2 text-sm font-medium text-white bg-blue-600 border border-transparent rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 disabled:opacity-50 disabled:cursor-not-allowed"
		>
			Next →
		</button>
	</div>
</div>