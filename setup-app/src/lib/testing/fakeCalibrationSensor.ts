import type { CalibrationCommand } from '$lib/calibrationCommands';
import type {
	CalibrationJoint,
	CalibrationReadings,
	DoCommandResponse,
	MotorVerificationResult
} from '$lib/types';

// In-memory stand-in for devrel:so101:calibration's readings + DoCommand contract, derived
// from components/calibration/{sensor,workflow,motorsetup}.go. It is a spec transcription, not
// a servo simulator -- see DIVERGENCES.
//
// DIVERGENCES from the Go module (all accepted; none affect what setup-app can observe):
// - No real serial bus: motor discovery/scan/assign-id are not modeled (only motor_setup_verify
//   is, since that is the one the priority-order test list needs).
// - Homing offset is a plain `pos - 2047`, without clampHomingOffset's sign-bit trim or
//   encodePositionOffset's range check -- setup-app only ever displays homing_offset, never
//   encodes it.
// - No goroutine/ticker: recorded ranges are whatever a test seeds directly on the joints.
// - Close()/recordingDone-channel joins and the mutex have no analogue; this is single-threaded.
// - Every Go handler that returns `success: false` also returns a non-nil error. At the real
//   gRPC boundary a returned error becomes a REJECTED call, not a resolved `{success:false}`
//   payload (a unary RPC yields one or the other, never both). So `sendCommand` here never
//   resolves with `success:false` -- every modeled failure throws, matching what
//   SensorProvider's real `sendCommand` actually sees on the wire.

export type FakeCalibrationState = CalibrationReadings['calibration_state'];

// sensor.go Readings' per-state switch, verbatim. 'reset' in every branch, including 'unknown'
// (the switch has no default, so an unreached state falls through to the empty slice literal).
const AVAILABLE_COMMANDS: Record<FakeCalibrationState, CalibrationCommand[]> = {
	idle: ['start', 'reset'],
	started: ['set_homing', 'abort', 'reset'],
	homing_position: ['start_range_recording', 'abort', 'reset'],
	range_recording: ['stop_range_recording', 'abort', 'reset'],
	completed: ['save_calibration', 'start', 'reset'],
	error: ['reset', 'start'],
	unknown: []
};

const JOINT_NAMES: Record<number, string> = {
	1: 'shoulder_pan',
	2: 'shoulder_lift',
	3: 'elbow_flex',
	4: 'wrist_flex',
	5: 'wrist_roll',
	6: 'gripper'
};

// math.MaxInt32 / math.MinInt32 -- the "no data yet" sentinels NewSO101CalibrationSensor seeds
// RecordedMin/RecordedMax with.
export const RECORDED_MIN_SENTINEL = 2147483647;
export const RECORDED_MAX_SENTINEL = -2147483648;

interface JointState {
	id: number;
	name: string;
	current_position: number;
	homing_offset: number;
	range_min: number;
	range_max: number;
	recorded_min: number;
	recorded_max: number;
	is_completed: boolean;
}

const EXPECTED_MOTOR_NAMES = [
	'shoulder_pan',
	'shoulder_lift',
	'elbow_flex',
	'wrist_flex',
	'wrist_roll',
	'gripper'
];

export class FakeCalibrationSensor {
	private state: FakeCalibrationState = 'idle';
	private instruction = 'Ready to start calibration.';
	private errorMsg = '';
	private lastSaveError = '';
	private recordingActive = false;
	private recordingStartedAt = 0;
	private positionSamples = 0;
	private readonly servoIds: number[];
	private readonly joints = new Map<number, JointState>();
	private readonly motorSetup = { in_progress: false, step: 0, status: '' };
	// Per-command scripted failures, consumed once -- forces exactly the failure path
	// workflow.go otherwise reaches only via a real bus error.
	private readonly scriptedFailures = new Map<CalibrationCommand, string>();
	private scriptedTorqueFailure: string | null = null;
	private readonly motorResults = new Map<string, MotorVerificationResult>();

	constructor(servoIds: number[] = [1, 2, 3, 4, 5, 6]) {
		this.servoIds = servoIds;
		for (const id of servoIds) this.resetJoint(id);
	}

	private resetJoint(id: number): void {
		this.joints.set(id, {
			id,
			name: JOINT_NAMES[id] ?? `servo_${id}`,
			current_position: 0,
			homing_offset: 0,
			range_min: 0,
			range_max: 4095,
			recorded_min: RECORDED_MIN_SENTINEL,
			recorded_max: RECORDED_MAX_SENTINEL,
			is_completed: false
		});
	}

	private requireJoint(id: number): JointState {
		const joint = this.joints.get(id);
		if (!joint) throw new Error(`fake sensor has no joint ${id}`);
		return joint;
	}

	private transition(state: FakeCalibrationState, instruction: string): void {
		this.state = state;
		this.instruction = instruction;
		this.errorMsg = state === 'error' ? instruction : '';
	}

	// ---- Test-driving controls ----

	/** Escape hatch for gating tests: sets state directly, bypassing workflow transitions. */
	forceState(state: FakeCalibrationState, instruction = ''): void {
		this.transition(state, instruction);
	}

	getState(): FakeCalibrationState {
		return this.state;
	}

	/** Directly stamps a joint's recorded range, bypassing start_range_recording/samples. */

	/** One sample tick of recordPositions: updates current/min/max for the given joints. */

	/** Makes the next call to `command` throw `message`, mirroring a Go handler returning err. */
	queueFailure(command: CalibrationCommand, message: string): void {
		this.scriptedFailures.set(command, message);
	}

	/** Makes the next successful save_calibration report torque_enabled:false. */

	/** Seeds motor_setup_verify's per-motor result for the next call. */
	setMotorResult(name: string, result: MotorVerificationResult): void {
		this.motorResults.set(name, result);
	}

	getLastSaveError(): string {
		return this.lastSaveError;
	}

	private takeFailure(command: CalibrationCommand): string | undefined {
		const msg = this.scriptedFailures.get(command);
		if (msg !== undefined) this.scriptedFailures.delete(command);
		return msg;
	}

	// ---- Readings (sensor.go's Readings, verbatim shape) ----

	getReadings(): CalibrationReadings {
		const joints: Record<string, CalibrationJoint> = {};
		for (const joint of this.joints.values()) {
			joints[joint.name] = {
				id: joint.id,
				name: joint.name,
				current_position: joint.current_position,
				homing_offset: joint.homing_offset,
				recorded_min: joint.recorded_min,
				recorded_max: joint.recorded_max,
				is_completed: joint.is_completed
			};
		}

		const readings: CalibrationReadings = {
			calibration_state: this.state,
			instruction: this.instruction,
			available_commands: AVAILABLE_COMMANDS[this.state] ?? [],
			servo_count: this.servoIds.length,
			joints,
			motor_setup: { ...this.motorSetup }
		};

		if (this.state === 'error') readings.error = this.errorMsg;
		if (this.lastSaveError) readings.last_save_error = this.lastSaveError;
		if (this.state === 'range_recording' && this.recordingActive) {
			readings.recording_time_seconds = (Date.now() - this.recordingStartedAt) / 1000;
			readings.position_samples = this.positionSamples;
		}

		return readings;
	}

	// ---- DoCommand (sensor.go's switch) ----

	async sendCommand(cmd: Record<string, unknown>): Promise<DoCommandResponse> {
		const command = cmd.command as string;
		switch (command) {
			case 'start':
				return this.start();
			case 'set_homing':
				return this.setHoming();
			case 'start_range_recording':
				return this.startRangeRecording();
			case 'stop_range_recording':
				return this.stopRangeRecording();
			case 'save_calibration':
				return this.saveCalibration();
			case 'abort':
				return this.abort();
			case 'reset':
				return this.reset();
			case 'motor_setup_verify':
				return this.motorSetupVerify();
			default:
				throw new Error(`unknown command: ${command}`);
		}
	}

	private start(): DoCommandResponse {
		if (this.state !== 'idle' && this.state !== 'completed' && this.state !== 'error') {
			throw new Error(`calibration already in progress (state: ${this.state})`);
		}
		this.lastSaveError = '';
		const failure = this.takeFailure('start');
		if (failure) {
			this.transition('error', `Failed to disable torque: ${failure}`);
			throw new Error(failure);
		}
		for (const id of this.servoIds) this.resetJoint(id);
		this.transition(
			'started',
			'Calibration started. Torque is off -- move every joint to the middle of its range of motion, then set the homing position.'
		);
		return { success: true, state: this.state, message: this.instruction };
	}

	private setHoming(): DoCommandResponse {
		if (this.state !== 'started') {
			throw new Error(`must start calibration first (current state: ${this.state})`);
		}
		const failure = this.takeFailure('set_homing');
		if (failure) {
			this.transition('error', failure);
			throw new Error(failure);
		}
		for (const joint of this.joints.values()) {
			joint.homing_offset = joint.current_position - 2047;
		}
		this.transition(
			'homing_position',
			'Homing positions set. Start range recording, then move every joint through its entire range of motion.'
		);
		return { success: true, state: this.state, message: this.instruction };
	}

	private startRangeRecording(): DoCommandResponse {
		if (this.state !== 'homing_position') {
			throw new Error(`must set homing position first (current state: ${this.state})`);
		}
		this.recordingActive = true;
		this.recordingStartedAt = Date.now();
		this.positionSamples = 0;
		this.transition(
			'range_recording',
			'Recording range of motion. Move every joint through its full range, then stop recording.'
		);
		return { success: true, state: this.state, message: this.instruction };
	}

	private stopRangeRecording(): DoCommandResponse {
		if (this.state !== 'range_recording') {
			throw new Error(`range recording not active (current state: ${this.state})`);
		}
		this.recordingActive = false;

		let allValid = true;
		for (const joint of this.joints.values()) {
			if (joint.recorded_min >= joint.recorded_max) {
				allValid = false;
				continue;
			}
			joint.range_min = joint.recorded_min;
			joint.range_max = joint.recorded_max;
			joint.is_completed = true;
		}
		if (!allValid) {
			this.transition(
				'error',
				'Invalid ranges detected. Some joints may not have been moved through their full range.'
			);
			throw new Error('invalid ranges detected');
		}
		this.transition(
			'completed',
			'Range recording complete. Save the calibration to write it to the servos and to file.'
		);
		return { success: true, state: this.state, message: this.instruction };
	}

	private saveCalibration(): DoCommandResponse {
		if (this.state !== 'completed') {
			throw new Error(`calibration not completed (current state: ${this.state})`);
		}
		const failure = this.takeFailure('save_calibration');
		if (failure) {
			// Deliberately stays 'completed' -- see workflow.go's saveCalibration -- so the
			// command stays retryable.
			this.lastSaveError = failure;
			throw new Error(failure);
		}
		this.lastSaveError = '';

		let torqueEnabled = true;
		let torqueError: string | undefined;
		if (this.scriptedTorqueFailure) {
			torqueEnabled = false;
			torqueError = this.scriptedTorqueFailure;
			this.scriptedTorqueFailure = null;
		}

		const message = torqueEnabled
			? 'Calibration completed and saved successfully. Torque re-enabled -- ready for new calibration.'
			: `Calibration saved successfully, but torque was left disabled: ${torqueError}. The arm is limp -- support it before letting go, then power-cycle the servos or restart the machine to restore holding torque.`;
		this.transition('idle', message);

		const result: DoCommandResponse = {
			success: true,
			state: this.state,
			calibration_file: 'so101_calibration.json',
			joints_calibrated: this.joints.size,
			torque_enabled: torqueEnabled,
			message
		};
		if (!torqueEnabled) result.torque_enable_error = torqueError;
		return result;
	}

	private abort(): DoCommandResponse {
		if (this.state === 'completed') {
			throw new Error(
				`cannot abort: a completed recording is waiting to be saved (current state: ${this.state}); use save_calibration or reset instead`
			);
		}

		let response: DoCommandResponse;
		if (this.state === 'idle') {
			response = { success: true, state: this.state, message: 'Already idle; nothing to abort.' };
		} else {
			this.recordingActive = false;
			this.transition('idle', 'Calibration aborted. Ready to start new calibration.');
			response = { success: true, state: this.state, message: this.instruction };
		}
		// sensor.go's DoCommand clears lastSaveError whenever abort returns a nil error --
		// both branches above do, only the StateCompleted rejection above does not.
		this.lastSaveError = '';
		return response;
	}

	private reset(): DoCommandResponse {
		this.recordingActive = false;
		this.lastSaveError = '';
		for (const id of this.servoIds) this.resetJoint(id);
		this.transition('idle', 'Calibration sensor reset. Ready to start calibration.');
		return { success: true, state: this.state, message: this.instruction };
	}

	private motorSetupVerify(): DoCommandResponse {
		const motors: Record<string, MotorVerificationResult> = {};
		let allGood = true;
		EXPECTED_MOTOR_NAMES.forEach((name, index) => {
			const result = this.motorResults.get(name) ?? {
				id: index + 1,
				status: 'not_found',
				error: 'servo not in controller'
			};
			motors[name] = result;
			if (result.status !== 'ok' && result.status !== 'model_detection_failed') allGood = false;
		});
		// Always success:true -- the bug fix under test. A partial motor failure must never
		// throw here; the aggregate verdict travels only in all_ok (motorsetup.go).
		return { success: true, all_ok: allGood, motors, status: 'verification complete' };
	}
}
