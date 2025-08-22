import type { CallOptions } from '@connectrpc/connect';
import type { JsonValue, SensorClient, Struct } from '@viamrobotics/sdk';
import type { CreateBaseMutationResult, QueryObserverResult } from '@tanstack/svelte-query';

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

export interface MotorSetupResult {
  motor_name: string;
  current_id: number;
  target_id: number;
  model: string;
  found_baudrate: number;
  step: 'discovered' | 'configured';
  success: boolean;
}

export interface MotorVerificationResult {
  id: number;
  status: 'ok' | 'not_responding' | 'not_found';
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
  | 'error';
  instruction: string;
  available_commands: string[];
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
  doCommand: { current: CreateBaseMutationResult<JsonValue, Error, [command: Struct, callOptions?: CallOptions | undefined], unknown> };
  sendCommand: (cmd: Record<string, any>) => Promise<DoCommandResponse>;
  sensorConfig: SensorConfig;
}

export interface StepProps {
  sensorClient: { current: SensorClient };
  sensorReadings: { current: QueryObserverResult<Record<string, JsonValue>> };
  doCommand: { current: CreateBaseMutationResult<JsonValue, Error, [command: Struct, callOptions?: CallOptions | undefined], unknown> };
  sendCommand: (cmd: Record<string, any>) => Promise<DoCommandResponse>;
  error: string | null;
  setError: (error: string | null) => void;
  clearError: () => void;
  nextStep: () => void;
  prevStep: () => void;
  motorSetupResults: Record<string, MotorSetupResult>;
  setMotorSetupResults: (results: Record<string, MotorSetupResult>) => void;
  updateMotorSetupResult: (motorName: string, result: MotorSetupResult) => void;
}

// Command response types
export interface DoCommandResponse {
  success: boolean;
  error?: string;
  [key: string]: any;
}

export interface MotorDiscoveryResponse extends DoCommandResponse {
  motor_name: string;
  current_id: number;
  target_id: number;
  model: string;
  found_baudrate: number;
  status: string;
}

export interface MotorAssignmentResponse extends DoCommandResponse {
  motor_name: string;
  old_id: number;
  new_id: number;
  new_baudrate: number;
  status: string;
}

export interface MotorVerificationResponse extends DoCommandResponse {
  motors: Record<string, MotorVerificationResult>;
  status: string;
}

export interface CalibrationCommandResponse extends DoCommandResponse {
  state?: string;
  message?: string;
  [key: string]: any;
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
  motorSetupResults?: Record<string, MotorSetupResult>;
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
