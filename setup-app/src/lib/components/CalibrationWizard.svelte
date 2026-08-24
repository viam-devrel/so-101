<script lang="ts">
	import { getContext } from 'svelte';
	import BaseWizard from './BaseWizard.svelte';
	import StepOverview from './steps/StepOverview.svelte';
	import StepCalibrationStart from './steps/StepCalibrationStart.svelte';
	import StepCalibrationHoming from './steps/StepCalibrationHoming.svelte';
	import StepCalibrationRecording from './steps/StepCalibrationRecording.svelte';
	import StepCalibrationSave from './steps/StepCalibrationSave.svelte';
	import StepComplete from './steps/StepComplete.svelte';
	import type {
		CalibrationReadings,
		WorkflowStep,
		MotorSetupResult,
		SensorContext,
		TorqueOutcome
	} from '$lib/types';
	import { logger } from '$lib/utils/logger';
	import { canAdvanceFromStep } from '$lib/wizardProgress';

	// getContext is only valid at component init, so call it once here (not inside $derived).
	const sensorContext = getContext<SensorContext>('sensor');

	// Calibration Workflow Configuration
	const WORKFLOW_STEPS: WorkflowStep[] = [
		'overview',
		'calibration_start',
		'calibration_homing',
		'calibration_recording',
		'calibration_save',
		'complete'
	];

	const STEP_TITLES = {
		overview: 'Overview & Safety',
		calibration_start: 'Start Calibration',
		calibration_homing: 'Set Homing Position',
		calibration_recording: 'Record Ranges',
		calibration_save: 'Save Calibration',
		complete: 'Calibration Complete'
	};

	// Wizard state management
	let currentStep = $state(0);
	let error = $state<string | null>(null);
	let motorSetupResults = $state<Record<string, MotorSetupResult>>({});
	// Explicit per-step completion, independent of any readings-derived signal -- see
	// wizardProgress.ts's canAdvanceFromStep.
	let stepCompletion = $state<Record<WorkflowStep, boolean>>({
		overview: false,
		motor_setup: false,
		motor_verify: false,
		calibration_start: false,
		calibration_homing: false,
		calibration_recording: false,
		calibration_save: false,
		complete: false
	});
	// Outcome of the last save_calibration's torque re-enable. See types.ts's TorqueOutcome.
	let torqueOutcome = $state<TorqueOutcome | null>(null);

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

	// Jump to a step by name within this workflow's own step list, so a step component never
	// has to import WORKFLOW_STEPS itself. No-op if this workflow doesn't include that step.
	function goToNamedStep(step: WorkflowStep) {
		const index = WORKFLOW_STEPS.indexOf(step);
		if (index >= 0) {
			goToStep(index);
		}
	}

	// A property write, NOT a whole-object reassign. StepMotorSetup calls this from inside an
	// $effect: reassigning would read stepCompletion (the spread) inside that effect's
	// reactive scope and then write it, which is a self-dependency Svelte 5 aborts with
	// effect_update_depth_exceeded. A property write reads nothing, and writing the same
	// boolean into a $state object property does not notify, so a re-run is a true no-op.
	function markStepComplete(step: WorkflowStep) {
		stepCompletion[step] = true;
	}

	// Same property-write rule as markStepComplete.
	function markStepIncomplete(step: WorkflowStep) {
		stepCompletion[step] = false;
	}

	function setTorqueOutcome(outcome: TorqueOutcome | null) {
		torqueOutcome = outcome;
	}

	// Error handling
	function setError(errorMessage: string | null) {
		error = errorMessage;
		if (errorMessage) {
			logger.warn('Workflow error set', errorMessage);
		}
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

	function clearMotorSetupResult(motorName: string) {
		const { [motorName]: _removed, ...rest } = motorSetupResults;
		motorSetupResults = rest;
	}

	// Step component props
	const stepProps = $derived({
		sensorClient: sensorContext.sensorClient,
		sensorReadings: sensorContext.sensorReadings,
		doCommand: sensorContext.doCommand,
		sendCommand:
			sensorContext.sendCommand || (() => Promise.reject(new Error('Send command not available'))),
		error,
		setError,
		clearError,
		nextStep,
		prevStep,
		goToStep,
		goToNamedStep,
		motorSetupResults,
		setMotorSetupResults,
		updateMotorSetupResult,
		clearMotorSetupResult,
		markStepComplete,
		markStepIncomplete,
		stepCompletion,
		torqueOutcome,
		setTorqueOutcome
	});

	// Current step name for rendering
	const currentStepName = $derived(WORKFLOW_STEPS[currentStep]);

	// Gates the footer "Next" button; see wizardProgress.ts for the per-step rules.
	const canAdvance = $derived(
		canAdvanceFromStep(
			currentStepName,
			sensorContext.sensorReadings.current.data as CalibrationReadings | undefined,
			motorSetupResults,
			stepCompletion
		)
	);
</script>

<BaseWizard
	workflowType="calibration"
	steps={WORKFLOW_STEPS}
	stepTitles={STEP_TITLES}
	{currentStep}
	{error}
	{canAdvance}
	onNextStep={nextStep}
	onPrevStep={prevStep}
	onGoToStep={goToStep}
	onClearError={clearError}
>
	{#if currentStepName === 'overview'}
		<StepOverview {...stepProps} workflowType="calibration" />
	{:else if currentStepName === 'calibration_start'}
		<StepCalibrationStart {...stepProps} />
	{:else if currentStepName === 'calibration_homing'}
		<StepCalibrationHoming {...stepProps} />
	{:else if currentStepName === 'calibration_recording'}
		<StepCalibrationRecording {...stepProps} />
	{:else if currentStepName === 'calibration_save'}
		<StepCalibrationSave {...stepProps} />
	{:else if currentStepName === 'complete'}
		<StepComplete {...stepProps} workflowType="calibration" />
	{/if}
</BaseWizard>
