<script lang="ts">
	import { getContext } from 'svelte';
	import BaseWizard from './BaseWizard.svelte';
	import StepOverview from './steps/StepOverview.svelte';
	import StepMotorSetup from './steps/StepMotorSetup.svelte';
	import StepMotorVerify from './steps/StepMotorVerify.svelte';
	import StepComplete from './steps/StepComplete.svelte';
	import type { WorkflowStep, MotorSetupResult } from '$lib/types';

	// Get sensor context - use $derived to ensure it's accessed during component lifecycle
	const sensorContext = $derived(() => getContext('sensor') as any);
	const sensorClient = $derived(() => sensorContext()?.sensorClient);
	const sensorReadings = $derived(() => sensorContext()?.sensorReadings);
	const doCommand = $derived(() => sensorContext()?.doCommand);
	const sendCommand = $derived(() => sensorContext()?.sendCommand);

	// Motor Setup Workflow Configuration
	const WORKFLOW_STEPS: WorkflowStep[] = ['overview', 'motor_setup', 'motor_verify', 'complete'];

	const STEP_TITLES = {
		overview: 'Overview & Safety',
		motor_setup: 'Motor Setup',
		motor_verify: 'Motor Verification',
		complete: 'Setup Complete'
	};

	// Wizard state management
	let currentStep = $state(0);
	let error = $state<string | null>(null);
	let motorSetupResults = $state<Record<string, MotorSetupResult>>({});

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
		sensorClient: sensorClient(),
		sensorReadings: sensorReadings(),
		doCommand: doCommand(),
		sendCommand: sendCommand() || (() => Promise.reject(new Error('Send command not available'))),
		error,
		setError,
		clearError,
		nextStep,
		prevStep,
		motorSetupResults,
		setMotorSetupResults,
		updateMotorSetupResult
	});

	// Current step name for rendering
	const currentStepName = $derived(WORKFLOW_STEPS[currentStep]);
</script>

<BaseWizard
	workflowType="motor-setup"
	steps={WORKFLOW_STEPS}
	stepTitles={STEP_TITLES}
	{currentStep}
	{error}
	onNextStep={nextStep}
	onPrevStep={prevStep}
	onGoToStep={goToStep}
	onClearError={clearError}
>
	{#if currentStepName === 'overview'}
		<StepOverview {...stepProps} workflowType="motor-setup" />
	{:else if currentStepName === 'motor_setup'}
		<StepMotorSetup {...stepProps} />
	{:else if currentStepName === 'motor_verify'}
		<StepMotorVerify {...stepProps} />
	{:else if currentStepName === 'complete'}
		<StepComplete {...stepProps} workflowType="motor-setup" />
	{/if}
</BaseWizard>
