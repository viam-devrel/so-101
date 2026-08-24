import type { CallOptions } from '@connectrpc/connect';
import type { JsonValue, SensorClient, Struct } from '@viamrobotics/sdk';
import type { CreateBaseMutationResult, QueryObserverResult } from '@tanstack/svelte-query';
import type { CalibrationCommand } from '$lib/calibrationCommands';

// Connection and authentication types
export interface ConnectionDetails {
	apiKey: {
		id: string;
		key: string;
	};
	machineId: string;
	hostname: string;
}

// Motor setup configuration
export interface MotorSetupConfig {
	name: string;
	targetId: number;
	description: string;
}

// Discriminated on `step`: a skipped motor was never discovered, so it carries none of the
// discovery fields -- a plain optional-fields shape would let a 'skipped' result be read with
// a stale current_id/model from a prior discovery.
export type MotorSetupResult =
	| {
			motor_name: string;
			step: 'discovered' | 'configured';
			current_id: number;
			target_id: number;
			model: string;
			found_baudrate: number;
			success: boolean;
	  }
	| {
			motor_name: string;
			step: 'skipped';
			success: boolean;
	  };

export interface MotorVerificationResult {
	id: number;
	status: 'ok' | 'not_responding' | 'not_found' | 'model_detection_failed';
	model?: string;
	error?: string;
}

// Calibration types
export interface CalibrationJoint {
	id: number;
	name: string;
	current_position: number;
	homing_offset: number;
	recorded_min: number;
	recorded_max: number;
	is_completed: boolean;
}

export interface CalibrationReadings {
	calibration_state:
		| 'idle'
		| 'started'
		| 'homing_position'
		| 'range_recording'
		| 'completed'
		| 'error'
		// components/calibration/sensor.go's CalibrationState.String() default branch; see
		// SensorConfigForm's CALIBRATION_STATES, which accepts it too.
		| 'unknown';
	instruction: string;
	// Narrowed to the command names sensor.go's per-state switch can emit (see
	// calibrationCommands.ts), so canRun's argument and this array share one vocabulary.
	available_commands: CalibrationCommand[];
	servo_count: number;
	recording_time_seconds?: number;
	position_samples?: number;
	joints: Record<string, CalibrationJoint>;
	motor_setup: {
		in_progress: boolean;
		step: number;
		status: string;
	};
	error?: string;
	// Durable signal for a failed save; `error` is only set in the `error` state, and a save
	// failure deliberately stays `completed`.
	last_save_error?: string;
}

// Wizard workflow types
export type WorkflowStep =
	| 'overview'
	| 'motor_setup'
	| 'motor_verify'
	| 'calibration_start'
	| 'calibration_homing'
	| 'calibration_recording'
	| 'calibration_save'
	| 'complete';

// Sensor context types
export interface SensorContext {
	sensorClient: { current: SensorClient };
	sensorReadings: { current: QueryObserverResult<Record<string, JsonValue>> };
	doCommand: {
		current: CreateBaseMutationResult<
			JsonValue,
			Error,
			[command: Struct, callOptions?: CallOptions | undefined],
			unknown
		>;
	};
	sendCommand: (cmd: Record<string, any>) => Promise<DoCommandResponse>;
	sensorConfig: SensorConfig;
}

export interface StepProps {
	sensorReadings: { current: QueryObserverResult<Record<string, JsonValue>> };
	sendCommand: (cmd: Record<string, any>) => Promise<DoCommandResponse>;
	error: string | null;
	setError: (error: string | null) => void;
	clearError: () => void;
	nextStep: () => void;
	// Jump to the step named `step` in the current workflow's own step list, without the step
	// component needing to import that list. A no-op if the current workflow doesn't have it.
	goToNamedStep: (step: WorkflowStep) => void;
	motorSetupResults: Record<string, MotorSetupResult>;
	updateMotorSetupResult: (motorName: string, result: MotorSetupResult) => void;
	// Deletes a motor's stored result entirely (e.g. "Rediscover"), rather than overwriting it.
	clearMotorSetupResult: (motorName: string) => void;
	// Marks a step done in the wizard's own completion record, independent of any
	// readings-derived signal. See wizardProgress.ts's canAdvanceFromStep.
	markStepComplete: (step: WorkflowStep) => void;
	// Reverses markStepComplete for one step. A property write, same rule as markStepComplete:
	// never reassign stepCompletion wholesale (see that doc) -- StepMotorSetup's $effect reads
	// the record and would self-invalidate on a reassign.
	markStepIncomplete: (step: WorkflowStep) => void;
	// The record markStepComplete writes to. A step's own success flags are local $state that
	// the wizard's {#if} destroys on navigation, so this is the only signal that survives a
	// remount -- a step whose module-side state does not distinguish "already done" (see
	// StepCalibrationSave) must read it to render correctly on return.
	stepCompletion: Record<WorkflowStep, boolean>;
	// Outcome of the most recent save_calibration's torque re-enable (workflow.go's
	// saveCalibration). Null until a save completes this session. Wizard-level, not step-local,
	// so a remounted StepCalibrationSave and StepComplete (which never calls save_calibration
	// itself) can both render the true outcome instead of assuming success.
	torqueOutcome: TorqueOutcome | null;
	setTorqueOutcome: (outcome: TorqueOutcome | null) => void;
}

// save_calibration's torque_enabled / torque_enable_error response fields, once known.
export interface TorqueOutcome {
	enabled: boolean;
	error?: string;
}

// Command response types
export interface DoCommandResponse {
	success: boolean;
	error?: string;
	[key: string]: any;
}

export interface MotorVerificationResponse extends DoCommandResponse {
	motors: Record<string, MotorVerificationResult>;
	// Aggregate verdict across all six motors. The UI derives its own allMotorsOk from the
	// per-motor statuses instead of trusting this directly -- see StepMotorVerify.svelte.
	all_ok: boolean;
	status: string;
}

// One servo found by motor_setup_scan_bus's full 1..253 ID sweep. `expected_name` is only
// present when `status` is 'expected' -- an ID outside 1-6 has no name to attach.
export interface MotorScanServo {
	id: number;
	model: string;
	status: 'expected' | 'unexpected';
	expected_name?: string;
}

export interface MotorScanBusResponse extends DoCommandResponse {
	servos_found: number;
	unexpected_count: number;
	servos: MotorScanServo[];
	status: string;
}

// Sensor configuration types
export interface SensorConfig {
	partId: string; // Default: 'main'
	sensorName: string; // User input, no default
}

// Workflow types
export type WorkflowType = 'motor-setup' | 'calibration' | 'full-setup';

export interface WorkflowInfo {
	id: WorkflowType;
	title: string;
	description: string;
	duration: string;
	steps: number;
	stepNames: string;
}

// Session state for workflow management
export interface SessionState {
	sensorConfig: SensorConfig;
	completedWorkflows: string[];
	timestamp?: number;
}

// Validation types
export interface ValidationResult {
	isValid: boolean;
	error?: string;
}

// Logger types
export interface Logger {
	debug(message: string, data?: any): void;
	info(message: string, data?: any): void;
	warn(message: string, data?: any): void;
	error(message: string, error?: Error): void;
}
