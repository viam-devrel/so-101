# Viam SO-101 Robotic Arm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for the SO-101 5-DOF + Gripper collaborative robotic arm designed by TheRobotStudio and HuggingFace.
It can be used to control either the leader or follower arm, as well as configuring both has separate arm components for mirrored teleoperation!

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This SO-101 module is particularly useful in applications that require the SO-101 arm to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separately through your own code.

## First Time Arm Setup

**If you are setting up the arm for the first time:**
You'll need to follow some prerequisite steps before using this module.
1. [Install LeRobot software](https://huggingface.co/docs/lerobot/installation)
2. [Configure the motors](https://huggingface.co/docs/lerobot/so101#configure-the-motors)
3. [Build the arm](https://huggingface.co/docs/lerobot/so101#clean-parts)
4. [Calibrate the arm](https://huggingface.co/docs/lerobot/so101#calibrate)

## Model devrel:so101:discovery

Automatic discovery service that detects connected arms and suggests component configurations for the calibration sensor, arm and gripper. It will also look for existing calibration files.

**IF YOU ARE BUILDING THIS ARM FOR THE FIRST TIME: add the calibration sensor and follow the setup instructions**

**Usage:**

- save your robot configuration with this service
- open the Test panel on service configuration card or view the Control tab
- click "+ add component" for the relevant component you'd like to set up: arm, gripper, or calibration sensor

### Troubleshooting

1. **Serial Connection Failed**:
   - Check that the USB cable is properly connected
   - Verify the correct port (Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`; Windows: `COM3`, `COM4`, etc.)
   - Ensure no other applications are using the serial port
   - Check USB permissions on Linux: `sudo chmod 666 /dev/ttyUSB0`

## Model devrel:so101:arm

The arm component controls the first 5 joints of the SO-101: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, and wrist_roll.

**Use the `devrel:so101:discovery` service to help with configuring this component automatically.**

Follow the [arm setup steps](#first-time-arm-setup) to learn how to set up the arm for the first time.

### Configuration

```json
{
  "port": "/dev/ttyUSB0"
}
```

### Attributes

The following attributes are available for the arm component:

| Name               | Type     | Inclusion    | Description                                                                                                                                                            |
| ------------------ | -------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `port`             | string   | **Required** | The serial port for communication with the SO-101 (see Communication section below).                                                                                   |
| `calibration_file` | string   | Optional     | Path to the calibration file. If not provided, the module will attempt to read calibration from servo registers. If servo reads fail, uses default calibration values. |
| `baudrate`         | int      | Optional     | The baud rate for serial communication. Default is `1000000`.                                                                                                          |
| `servo_ids`        | []int    | Optional     | List of servo IDs for the arm joints. Default is `[1, 2, 3, 4, 5]`.                                                                                                    |
| `timeout`          | duration | Optional     | Communication timeout. Default is system default.                                                                                                                      |
| `visualize_ee_frame` | bool   | Optional     | When `true`, serves a colored XYZ coordinate-frame marker at the end-effector for the 3D viewer. Default is `false`.                                                    |
| `speed_degs_per_sec` | float  | Optional     | Real per-joint speed cap enforced on the servos, in degrees per second. Each joint moves at this speed independently, so joints with longer travel finish later (planned motion stays smooth because the planner sends fine-grained waypoints). Default `50`, valid range 3–180. |
| `acceleration_degs_per_sec_per_sec` | float | Optional | Validated and stored but **not yet enforced** on hardware (planned follow-up). Default `100`, valid range 10–500. |
| `use_urdf`         | bool     | Optional     | When `true`, sources kinematics and collision geometry from the bundled `arm/so101.urdf` instead of the embedded `so101.json`. Requires the `VIAM_MODULE_ROOT` environment variable to be set (viam-server sets this automatically). This is a drop-in swap: the bundled URDF uses `so101.json`'s frame names and TCP exactly, so the frame names, TCP pose, and kinematics are identical — only the collision geometry upgrades from primitive shapes to per-link meshes. Default `false`. |
| `mesh_decimation_ratios` | []float | Optional | Per-collision-mesh simplification ratios, one per arm link in document order (base, shoulder, upper_arm, lower_arm, wrist), each in `[0, 1]`. Only used when `use_urdf` is `true`. Only values strictly in `(0, 1)` decimate (lower = more aggressive). Defaults to `0.9` for each of the 5 meshes when empty. |
| `manual_mode`      | object   | Optional     | Tuning for hand-guided [manual mode](#manual-mode) (see below). Omit for the built-in defaults. |

**If you're building and setting up an arm for the first time, please see the [calibration sensor component](#model-devrelso101calibration) for setup instructions.**

This may also be necessary if you see inaccuracy issues while controlling the arm.

### Calibration Priority

The module uses this calibration priority:

1. **File-based** - If `calibration_file` is configured and exists, load from file
2. **Servo registers** - If no file configured/found, read homing offset and range limits from servo hardware
3. **Hardcoded defaults** - If servo reads fail, use default values (offset=0, range=500-3500)

The servo register fallback provides better out-of-box experience by using actual hardware settings instead of generic defaults. Each startup without a calibration file will re-read from servos to ensure fresh data.

**Note:** Calibration read from servos is used in-memory only and not automatically saved. To persist servo-read calibration, use the calibration sensor component's workflow.

### Motion and speed

The hardware arm enforces a real per-joint speed cap on the Feetech servos: each joint moves at the configured `speed_degs_per_sec` (or the per-move override, see below). Because joints move independently rather than coordinating arrival times, a joint with a larger travel angle will take longer to reach its target than one with smaller travel. For RDK motion-planned paths this remains smooth in practice because the planner generates many fine-grained waypoints, keeping per-waypoint joint displacements small.

`MoveThroughJointPositions` accepts an optional `MoveOptions.MaxVelRads` (in radians/second) that overrides the configured default for that move. `MaxAccRads` is accepted but intentionally ignored — acceleration is not yet enforced on hardware. `GoToInputs` routes through this same path.

`MoveThroughJointPositions` streams waypoints back-to-back: intermediate waypoints are commanded without waiting for the arm to settle (flythrough), and the arm waits to stop only after the **final** waypoint. As a result, intermediate waypoints are not guaranteed stop-points. `GoToInputs` shares this same streaming path.

`IsMoving()` reports whether a move call is currently in progress. On the internal safety-timeout edge case (a move times out waiting for servos to settle), `IsMoving` may briefly differ from whether the servos are physically moving.

`Stop()` does not currently interrupt an in-progress joint move: a move blocks until the servos settle (or the internal safety timeout elapses), so a `Stop()` issued mid-move takes effect only once that move returns. Interrupting in-flight motion is a planned follow-up.

`acceleration_degs_per_sec_per_sec` is validated and stored when set via config or `set_acceleration` DoCommand, but is not yet written to the servo hardware. Acceleration enforcement is a planned follow-up.

### Manual Mode

Manual mode is a live jog-by-hand mode using position-deviation admittance. While active, a background control loop keeps each arm servo in position mode holding a goal — so the arm holds against gravity and never goes limp. When you backdrive a joint past a small angular deadband, the goal follows your hand, trailing by the deadband (which leaves a small restoring force so the joint still holds against gravity); when released, the joint holds its new pose. Torque is never fully disabled. The servo's `present_load` register is not used for control — it saturates on the gentlest push and doesn't encode push direction — and is reported only as telemetry.

Manual mode is entered and exited with the `enter_manual_mode` / `exit_manual_mode` DoCommands (see below) and is tuned with the optional `manual_mode` config object. Calling `enter_manual_mode` again while already active re-applies with the new parameters (no need to `exit_manual_mode` first) — useful for live tuning.

| Name               | Type    | Inclusion | Description                                                                                                                                                   |
| ------------------ | ------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `pos_deadband_deg` | float   | Optional  | Degrees a joint must be backdriven from its goal before the goal follows your hand. Higher = stiffer/less sensitive. Valid range 0-45. Default `0.5`. |
| `loop_hz`          | float   | Optional  | Control loop rate, in Hz. Valid range 0-200 (`0` uses the default). Default `50`.                                                                               |
| `p_gain`           | int     | Optional  | Reduced servo position P-gain applied during manual mode, restored on exit. Valid range 0-255. Default `8`. Lower = easier to backdrive/lighter feel, but must stay high enough to hold gravity. |
| `torque_limit`     | int     | Optional  | Servo torque cap (STS3215 `torque_limit` register) applied during manual mode, restored on exit. Valid range 0-1000. Default `50`. Lower = easier to backdrive, but it **must stay above the gravity-hold load or the joint will sag** — and a sagging joint drifting past `pos_deadband_deg` re-triggers following, so watch that `dev_deg` at rest stays close to `0`. `p_gain` alone softens the position loop but doesn't cap applied force; `torque_limit` is what actually makes the servo compliant to a hand. |

All fields are optional; a zero/omitted value falls back to the built-in default, so manual mode is usable with no `manual_mode` config at all.

**The defaults are bench-validated** on an unloaded SO-101: `p_gain: 8`, `torque_limit: 50`, `pos_deadband_deg: 0.5` — easy to guide by hand, and holds a fully extended arm without drooping. Compliance is arm- and payload-dependent, so an arm carrying a payload (or with different servo tuning) may need a higher `torque_limit` to hold its pose; verify on your arm that `dev_deg` at rest stays close to `0`. To leave the servos' configured `p_gain`/`torque_limit` untouched entirely, pass an explicit `0` as an inline override to `enter_manual_mode` (a config `0` is indistinguishable from omitted and uses the default).

```json
{
  "manual_mode": {
    "pos_deadband_deg": 0.5,
    "loop_hz": 50,
    "p_gain": 8,
    "torque_limit": 50
  }
}
```

**Safety:** any motion command auto-exits manual mode first and re-stiffens the arm at its current pose — this includes `MoveToPosition`, `MoveToJointPositions`, `MoveThroughJointPositions`, motion-service planning (`GoToInputs`), and `Stop`. A hand may still be on the arm when this happens, so expect the arm to become rigid and then move. Manual mode is also torn down on `Close`/reconfigure. It has no idle timeout — it stays active until explicitly exited (`exit_manual_mode`) or auto-exited by one of the events above.

The arm's `GetStatus`/`Status` reports the current mode. Normally:

```json
{ "mode": "auto" }
```

While manual mode is active, it also reports live per-joint telemetry (`actual_deg`, `goal_deg`, `dev_deg`, and `load`), which is useful for tuning `pos_deadband_deg`. `load` is telemetry only and is not used for control:

```json
{
  "mode": "manual",
  "manual_mode": {
    "joints": [{ "servo": 1, "actual_deg": 12.3, "goal_deg": 10.3, "dev_deg": 2.0, "load": 0 }]
  }
}
```

**Tuning diagnostic:** the `set_torque_limit` DoCommand directly writes the `torque_limit` register on every arm servo (with a readback in the response) so you can find the lowest value that still holds the arm before dialing it into `manual_mode`. It takes a `value` (0–1000), requires manual mode to be off, and does not auto-restore — set it back to `1000` (or reinitialize) when done.

### Communication

The SO-101 uses serial communication over USB with Feetech STS3215 servos. The module uses a shared controller architecture to manage all 6 servos while preventing resource conflicts when both arm and gripper components are used.

You can use the included [discovery service](#model-devrelso101discovery) or find the available serial port options from your machine's command line.

On MacOS, look for `usbmodem` or `usbserial` in the name:

```
you@machine: ls /dev/tty.*
/dev/tty.Bluetooth-Incoming-Port
/dev/tty.debug-console
/dev/tty.usbmodem58CD1767051

you@machine: ls /dev/cu.*
/dev/cu.Bluetooth-Incoming-Port
/dev/cu.debug-console
/dev/cu.usbmodem58CD1767051
```

On Linux, look for `ACM` or `USB` in the name:

```
you@machine: ls /dev/tty*
/dev/ttyACM0
/dev/ttyUSB0
```

On Windows, look for `COM` in the name:

```
you@machine: mode
COM0
COM1
```

### DoCommand

The module provides several custom commands accessible through the `DoCommand` interface:

#### Set Torque Control

Enable or disable joint torque:

```json
{
  "command": "set_torque",
  "enable": true
}
```

#### Ping Servos

Test communication with all servos:

```json
{
  "command": "ping"
}
```

#### Controller Status

Check the shared controller status for debugging:

```json
{
  "command": "controller_status"
}
```

#### Connection Diagnostics

Run comprehensive connection diagnostics:

```json
{
  "command": "diagnose"
}
```

#### Verify Configuration

Verify servo configuration and communication:

```json
{
  "command": "verify_config"
}
```

#### Reinitialize Servos

Reinitialize servo communication with retry attempts:

```json
{
  "command": "reinitialize",
  "retries": 3
}
```

#### Test Servo Communication

Test communication and read positions from arm servos:

```json
{
  "command": "test_servo_communication"
}
```

#### Reload Calibration

Reload calibration from file:

```json
{
  "command": "reload_calibration"
}
```

#### Get Calibration

Retrieve current calibration data:

```json
{
  "command": "get_calibration"
}
```

#### Enter Manual Mode

Enter [manual (hand-guided) mode](#manual-mode). Accepts the same tuning keys as the `manual_mode` config object (`pos_deadband_deg`, `loop_hz`, `p_gain`, `torque_limit`) inline as optional overrides, which layer on top of the configured `manual_mode` values. Calling this while already in manual mode tears down the current session and starts a fresh one with the new parameters — no need to `exit_manual_mode` first.

```json
{ "command": "enter_manual_mode", "pos_deadband_deg": 3 }
```

Response:

```json
{ "mode": "manual", "servos": [1, 2, 3, 4, 5] }
```

#### Exit Manual Mode

Leave manual mode, restore normal servo stiffness, and hold the current pose:

```json
{ "command": "exit_manual_mode" }
```

Response:

```json
{ "mode": "auto" }
```

#### Set Speed

Update the per-joint speed cap at runtime without reconfiguring the component. `set_speed` is a key in the request map (not a `"command"` value). Valid range: 3–180 degrees/second.

```json
{
  "set_speed": 60.0
}
```

Response:

```json
{
  "speed_set": 60.0
}
```

#### Set Acceleration

Update the acceleration parameter at runtime. `set_acceleration` is a key in the request map (not a `"command"` value). Valid range: 10–500 degrees/second². **Note:** acceleration is validated and stored but not yet enforced on hardware.

```json
{
  "set_acceleration": 150.0
}
```

Response:

```json
{
  "acceleration_set": 150.0
}
```

#### Get Motion Parameters

Retrieve the current speed and acceleration values in use. `get_motion_params` is a key set to `true` in the request map (not a `"command"` value). Both `set_speed` and `get_motion_params` can be combined in one request.

```json
{
  "get_motion_params": true
}
```

Response:

```json
{
  "current_speed_degs_per_sec": 50.0,
  "current_acceleration_degs_per_sec_per_sec": 100.0
}
```

## Model devrel:so101:simulated

The simulated arm component emulates the SO-101 arm entirely in software, with **no hardware required**. It shares the same kinematics as the [`devrel:so101:arm`](#model-devrelso101arm) model, so it is useful for developing and testing configs, motion plans, and the 3D scene viewer without a physical robot.

When commanded to a joint configuration, the simulated arm interpolates its joints toward the target over time at a configurable speed, just like rdk's builtin `simulated` arm. It also serves 3D meshes for each link, so it renders as an SO-101 in the visualizer.

### Configuration

```json
{}
```

All attributes are optional, so the simulated arm works with an empty configuration.

### Attributes

The following attributes are available for the simulated arm component:

| Name                 | Type   | Inclusion | Description                                                                                                                                              |
| -------------------- | ------ | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `speed_degs_per_sec` | float  | Optional  | How fast each joint travels toward its target, in degrees per second. Default is `90`.                                                                   |
| `motion`             | string | Optional  | Name of the motion service used to plan `MoveToPosition` requests. Default is `"builtin"`.                                                               |
| `simulate_time`      | bool   | Optional  | Whether a background goroutine advances the arm's position in real time. Default is `true`. (Tests set it to `false` to drive the simulated clock.)        |
| `visualize_ee_frame` | bool   | Optional  | When `true`, serves a colored XYZ coordinate-frame marker at the end-effector for the 3D viewer. Default is `false`.                                       |
| `use_urdf`           | bool   | Optional  | When `true`, sources kinematics and collision geometry from the bundled `arm/so101.urdf` instead of the embedded `so101.json`. Requires the `VIAM_MODULE_ROOT` environment variable to be set (viam-server sets this automatically). This is a drop-in swap: the bundled URDF uses `so101.json`'s frame names and TCP exactly, so the frame names, TCP pose, and kinematics are identical — only the collision geometry upgrades from primitive shapes to per-link meshes. Default `false`. |
| `mesh_decimation_ratios` | []float | Optional | Per-collision-mesh simplification ratios, one per arm link in document order (base, shoulder, upper_arm, lower_arm, wrist), each in `[0, 1]`. Only used when `use_urdf` is `true`. Only values strictly in `(0, 1)` decimate (lower = more aggressive). Defaults to `0.9` for each of the 5 meshes when empty. |

Example configuration with attributes:

```json
{
  "speed_degs_per_sec": 60
}
```

### Behavior

- `MoveToJointPositions`, `MoveThroughJointPositions`, and `GoToInputs` interpolate the joints toward each target over time; the calls block until the arm arrives.
- `MoveToPosition` plans through the motion service. Because the SO-101 is a 5-DOF arm, it defaults the planner goal metric to `position_only`; pass your own `goal_metric_type` via `extra` to override.
- `JointPositions`, `EndPosition`, `Geometries`, and `IsMoving` report the simulated state.

### DoCommand

#### Get Motion Parameters

Retrieve the configured joint speed:

```json
{
  "command": "get_motion_params"
}
```

## Model devrel:so101:gripper

The gripper component controls the 6th servo of the SO-101, which functions as a parallel gripper.

**Use the `devrel:so101:discovery` service to help with configuring this component automatically.**

Follow the [arm setup steps](#first-time-arm-setup) to learn how to set up the arm for the first time.

### Configuration

```json
{
  "port": "/dev/ttyUSB0"
}
```

**Use the same `port` and `calibration_file` configuration as the associated arm component.**

### Attributes

| Name               | Type     | Inclusion | Description                                                   |
| ------------------ | -------- | --------- | ------------------------------------------------------------- |
| `port`             | string   | Required  | The serial port for communication with the SO-101.            |
| `calibration_file` | string   | Optional  | Path to the calibration file (shared with arm component).     |
| `baudrate`         | int      | Optional  | The baud rate for serial communication. Default is `1000000`. |
| `servo_id`         | int      | Optional  | The servo ID for the gripper. Default is `6`.                 |
| `timeout`          | duration | Optional  | Communication timeout. Default is system default.             |
| `gripper_type`     | string   | Optional  | Which gripper meshes `Geometries()` serves for the 3D viewer: `follower` (moving jaw, default) or `leader` (thumb-loop handle + trigger). The moving part articulates with the live gripper opening. |
| `mesh_detail`      | string   | Optional  | Gripper mesh resolution: `low` (decimated, the default) or `high` (full resolution). `Geometries()` is also the motion-planning collision geometry, so `low` keeps planning fast; `high` is mainly for visual comparison. |

### Communication

You can use the included [discovery service](#model-devrelso101discovery) or find the available serial port options from your machine's command line.

On MacOS, look for `usbmodem` or `usbserial` in the name:

```
you@machine: ls /dev/tty.*
/dev/tty.Bluetooth-Incoming-Port
/dev/tty.debug-console
/dev/tty.usbmodem58CD1767051

you@machine: ls /dev/cu.*
/dev/cu.Bluetooth-Incoming-Port
/dev/cu.debug-console
/dev/cu.usbmodem58CD1767051
```

On Linux, look for `ACM` or `USB` in the name:

```
you@machine: ls /dev/tty*
/dev/ttyACM0
/dev/ttyUSB0
```

On Windows, look for `COM` in the name:

```
you@machine: mode
COM0
COM1
```

### DoCommand

The gripper component provides several custom commands:

#### Get Gripper Position

Get the current gripper position:

```json
{
  "command": "get_position"
}
```

#### Set Gripper Position

Set the gripper to a specific servo position:

```json
{
  "command": "set_position",
  "servo_position": 2000
}
```

#### Controller Status

Check the shared controller status:

```json
{
  "command": "controller_status"
}
```

## Model devrel:so101:simulated-gripper

The simulated gripper emulates the SO-101 gripper entirely in software, with **no hardware required**. It serves the gripper meshes (a static body plus a moving jaw/trigger) to the 3D scene viewer, with the moving part posed by the gripper's opening — useful for developing and previewing without a physical robot.

`Open` opens the gripper and `Grab` closes it, interpolating the jaw open/closed over time so it animates in the 3D viewer (like the simulated arm). Both block until the motion finishes, and `IsMoving` reports `true` while the jaw is travelling. A simulated gripper never grasps an object, so `Grab` always reports `false`.

### Configuration

```json
{}
```

All attributes are optional, so the simulated gripper works with an empty configuration.

### Attributes

| Name           | Type   | Inclusion | Description                                                                                                |
| -------------- | ------ | --------- | ---------------------------------------------------------------------------------------------------------- |
| `gripper_type` | string | Optional  | Which gripper meshes to serve: `follower` (moving jaw, the default) or `leader` (thumb-loop handle + trigger). |
| `mesh_detail`  | string | Optional  | Gripper mesh resolution: `low` (decimated, the default) or `high` (full resolution). The mesh is also the motion-planning collision geometry, so `low` keeps planning fast; set `high` on a second gripper for a visual side-by-side. |

### DoCommand

#### Set Position

Set a target opening percentage (0 = closed, 100 = open); the jaw interpolates toward it:

```json
{
  "command": "set_position",
  "percentage": 50
}
```

#### Get Position

```json
{
  "command": "get_position"
}
```

## Model devrel:so101:calibration

When assembling the SO-101 arm from a kit, it requires calibration to map servo positions to joint angles based on how the arm was assembled.

The SO-101 Calibration Sensor provides a calibration workflow integrated into Viam's component system. It guides you through the calibration process using DoCommand calls and provides status updates through sensor readings.

**IF YOU ARE SETTING UP THIS ARM FOR THE FIRST TIME: Follow the [arm setup steps](#first-time-arm-setup) to learn how to set up the arm for the first time.**

**See the [Setup Application](https://so101-setup_devrel.viamapplications.com) for a visual walkthrough experience that uses this component.**

_If you have already calibrated the servos on this arm, use the `devrel:so101:discovery` service to help with configuring this component automatically._

### Configuration

```json
{
  "port": "/dev/ttyUSB0",
  "calibration_file": "my_awesome_arm.json"
}
```

#### Attributes

| Name               | Type     | Required     | Description                                                                                                                     |
| ------------------ | -------- | ------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `port`             | string   | **Required** | Serial port for servo communication (see Communication section below)                                                           |
| `calibration_file` | string   | Optional     | Path where calibration will be saved. If relative path, uses `$VIAM_MODULE_DATA` directory. Default: `"so101_calibration.json"` |
| `baudrate`         | int      | Optional     | Serial communication speed. Default: `1000000`                                                                                  |
| `timeout`          | duration | Optional     | Communication timeout. Default: `"5s"`                                                                                          |

### Communication

You can use the included [discovery service](#model-devrelso101discovery) or find the available serial port options from your machine's command line.

On MacOS, look for `usbmodem` or `usbserial` in the name:

```
you@machine: ls /dev/tty.*
/dev/tty.Bluetooth-Incoming-Port
/dev/tty.debug-console
/dev/tty.usbmodem58CD1767051

you@machine: ls /dev/cu.*
/dev/cu.Bluetooth-Incoming-Port
/dev/cu.debug-console
/dev/cu.usbmodem58CD1767051
```

On Linux, look for `ACM` or `USB` in the name:

```
you@machine: ls /dev/tty*
/dev/ttyACM0
/dev/ttyUSB0
```

On Windows, look for `COM` in the name:

```
you@machine: mode
COM0
COM1
```

### Usage

#### Monitor Progress

Example output:

```json
{
  "calibration_state": "range_recording",
  "instruction": "Recording range of motion. Move all joints through their full ranges.",
  "available_commands": ["stop_range_recording", "abort"],
  "servo_count": 5,
  "recording_time_seconds": 15.3,
  "position_samples": 306,
  "joints": {
    "shoulder_pan": {
      "id": 1,
      "current_position": 2150,
      "homing_offset": -103,
      "recorded_min": 758,
      "recorded_max": 3292,
      "is_completed": false
    }
  }
}
```

### Available Commands

```json
{
  "command": "command_name"
}
```

#### Workflow Commands

| Command                 | Description                                     | Required State               |
| ----------------------- | ----------------------------------------------- | ---------------------------- |
| `start`                 | Begin calibration workflow                      | `idle`, `completed`, `error` |
| `set_homing`            | Set homing offsets and write to servo registers | `started`                    |
| `start_range_recording` | Begin recording servo ranges                    | `homing_position`            |
| `stop_range_recording`  | Complete range recording                        | `range_recording`            |
| `save_calibration`      | Write limits to servos and save file            | `completed`                  |
| `abort`                 | Cancel calibration                              | Any                          |
| `reset`                 | Reset to initial state                          | `error`                      |

#### Utility Commands

| Command                 | Description                  |
| ----------------------- | ---------------------------- |
| `get_current_positions` | Read current servo positions |

#### Motor Setup Commands

The calibration sensor also provides motor setup commands for initial SO-101 servo configuration. These commands implement the systematic motor setup process described in `MOTOR_SETUP.md` and are separate from the calibration workflow.

| Command                    | Description                                       | Parameters                                                                             |
| -------------------------- | ------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `motor_setup_discover`     | Discover a single motor connected to the bus      | `motor_name` (string): Motor name (e.g., "gripper", "wrist_roll")                      |
| `motor_setup_assign_id`    | Assign target ID and baudrate to discovered motor | `motor_name` (string), `current_id` (int), `target_id` (int), `current_baudrate` (int) |
| `motor_setup_verify`       | Verify all SO-101 motors are properly configured  | None                                                                                   |
| `motor_setup_scan_bus`     | Scan the entire bus for connected servos          | None                                                                                   |
| `motor_setup_reset_status` | Reset motor setup status                          | None                                                                                   |

#### Motor Setup Workflow

The motor setup process should be performed in reverse order (gripper → shoulder_pan) to avoid ID conflicts:

1. **Connect only one motor** (e.g., gripper) to the controller
2. **Discover**: `{"command": "motor_setup_discover", "motor_name": "gripper"}`
3. **Assign ID**: `{"command": "motor_setup_assign_id", "motor_name": "gripper", "current_id": 1, "target_id": 6, "current_baudrate": 57600}`
4. Repeat for each motor in order: wrist_roll → wrist_flex → elbow_flex → shoulder_lift → shoulder_pan
5. **Verify**: `{"command": "motor_setup_verify"}` (connect all motors)

#### Motor Setup Status

Motor setup status is included in sensor readings:

```json
{
  "motor_setup": {
    "in_progress": false,
    "step": 0,
    "status": "Motor setup ready"
  }
}
```

### State Machine

The calibration sensor operates as a state machine:

- **`idle`**: Ready to start calibration
- **`started`**: Torque disabled, ready for homing position
- **`homing_position`**: Homing set, ready for range recording
- **`range_recording`**: Recording min/max positions
- **`completed`**: Calibration data ready to save
- **`error`**: Error occurred, use reset command

### Calibration File Output

The sensor saves calibration in the standard format:

```json
{
  "shoulder_pan": {
    "id": 1,
    "drive_mode": 0,
    "homing_offset": -1470,
    "range_min": 758,
    "range_max": 3292,
    "norm_mode": 3
  },
  "shoulder_lift": {
    "id": 2,
    "drive_mode": 0,
    "homing_offset": 157,
    "range_min": 612,
    "range_max": 3401,
    "norm_mode": 3
  },
  "gripper": {
    "id": 6,
    "drive_mode": 0,
    "homing_offset": 1407,
    "range_min": 2031,
    "range_max": 3476,
    "norm_mode": 1
  }
}
```

### Troubleshooting

#### Common Issues

- **Range recording not working**: Ensure you call `start_range_recording` and manually move joints
- **Invalid ranges**: Move joints through their complete range of motion
- **Servo communication errors**: Check port, baudrate, and servo connections
- **Permission denied**: Ensure proper access to serial port (`sudo chmod 666 /dev/ttyUSB0`)

## Model devrel:so101:teleop

A generic service that mirrors a configured **leader** SO-101 arm's joint positions (and gripper opening) onto a configured **follower** arm in real time, enabling hand-guided teleoperation. While the loop is running the leader's servo torque is disabled so the arm can be back-driven by hand; the follower continuously chases the latest leader pose.

Leader and follower must be separate SO-101 arm components, typically on separate serial ports (one USB cable per arm).

### Configuration

```json
{
  "leader_arm": "arm-leader",
  "follower_arm": "arm-follower",
  "leader_gripper": "gripper-leader",
  "follower_gripper": "gripper-follower"
}
```

### Attributes

| Name                     | Type   | Inclusion    | Description                                                                                                                                                                          |
| ------------------------ | ------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `leader_arm`             | string | **Required** | Name of the arm component to read joint positions from.                                                                                                                             |
| `follower_arm`           | string | **Required** | Name of the arm component to drive.                                                                                                                                                  |
| `leader_gripper`         | string | Optional     | Name of the leader gripper component. When both `leader_gripper` and `follower_gripper` are set, gripper opening is mirrored in addition to joint positions.                         |
| `follower_gripper`       | string | Optional     | Name of the follower gripper component.                                                                                                                                              |
| `rate_hz`                | number | Optional     | Mirror loop frequency in Hz. Default `20`. Must be > 0 and ≤ 1000.                                                                                                                  |
| `manage_leader_torque`   | bool   | Optional     | When `true` (default), disables leader arm torque on start so the arm can be back-driven by hand, and restores torque on stop. Because the leader arm and gripper share one servo bus, this also frees the leader gripper servo. |
| `auto_start`             | bool   | Optional     | When `true`, starts the mirror loop immediately when the service is configured. Default `false`.                                                                                     |
| `max_consecutive_errors` | int    | Optional     | Number of consecutive read/write failures after which the loop stops automatically and reports the error via `status`. Default `10`.                                                 |

### DoCommand

#### Start

Start the mirror loop:

```json
{ "command": "start" }
```

Response:

```json
{ "running": true }
```

#### Stop

Stop the mirror loop:

```json
{ "command": "stop" }
```

Response:

```json
{ "running": false }
```

#### Status

Query the current loop state:

```json
{ "command": "status" }
```

Response:

```json
{
  "running": true,
  "cycles": 142,
  "consecutive_errors": 0,
  "last_error": "",
  "rate_hz": 20
}
```

`cycles` is the total number of successful mirror cycles since the last start. `consecutive_errors` resets to zero on any successful cycle. When `max_consecutive_errors` is reached the loop sets `running` to `false` and records the triggering error in `last_error`.

### Notes

- Joint targets are sent to the follower with `wait: false` (non-blocking), so the follower continuously chases the most recent leader pose without queuing up moves.
- When `manage_leader_torque` is enabled (the default), torque is restored on the leader when the loop stops for any reason — including reaching the error threshold or calling `Close` on the service.
- To use without gripper mirroring, omit both `leader_gripper` and `follower_gripper`.

## Troubleshooting

### Connection Issues

1. **Serial Connection Failed**:
   - Check that the USB cable is properly connected
   - Verify the correct port (Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`; Windows: `COM3`, `COM4`, etc.)
   - Ensure no other applications are using the serial port
   - Check USB permissions on Linux: `sudo chmod 666 /dev/ttyUSB0`

2. **Servo Communication Errors**:
   - Verify servo IDs are correctly configured (1-6)
   - Check calibration file path and format
   - Use the `diagnose` DoCommand for detailed diagnostics
   - Try reinitializing with the `reinitialize` DoCommand

3. **Shared Controller Conflicts**:
   - Check controller status using the `controller_status` DoCommand
   - Ensure consistent configuration across arm and gripper components
   - Verify the same serial port and baudrate are used
   - Restart components if configuration changes are needed

## Hardware Setup

1. **Power**: Connect the properly rated power adapter to the arm's controller board, either 5-7.4V or 12V depending on your servos
2. **USB Communication**: Connect the USB cable between the controller board and your computer
3. **Servo Connections**: Ensure all servos are properly daisy-chained with 3-pin cables
4. **Initial Position**: Manually position the arm in a safe configuration before powering on

## Safety Notes

> [!WARNING]
>
> - Always ensure the arm's workspace is clear before operation
> - The arm can move quickly - maintain safe distances during operation
> - Use the torque control features to enable safe manual positioning when needed
> - Proper calibration is essential for safe operation within expected ranges

## Setup Application

This module includes a web-based setup application that provides guided workflows for configuring your SO-101 robotic arm. The application is hosted as a Viam App and automatically deployed with each module version.

**Access the Setup App**: https://so101-setup_devrel.viamapplications.com

_Add an extra `/` to the end of the URL if you see a "404 Not Found" after authenticating._

### Available Workflows

The setup application provides three main workflows to guide you through different aspects of SO-101 configuration:

#### Full Setup (Recommended)

Complete setup workflow from unboxed hardware to fully configured and calibrated SO-101 arm. This comprehensive process includes:

- Hardware connection verification
- Motor ID configuration for all servos (1-6)
- Joint calibration with homing positions and range limits
- Final system testing and validation

#### Motor Setup Only

Configure servo IDs and communication parameters for SO-101 motors. Use this workflow when you need to:

- Set up servos with proper IDs (1-6) from factory defaults
- Configure communication baudrate (1,000,000 bps)
- Verify motor connectivity and response
- Resolve servo ID conflicts

#### Calibration Only

Calibrate joint ranges and homing positions for arms with already configured motors. This workflow covers:

- Setting homing positions for all joints
- Recording joint range limits through guided manual movement
- Saving calibration data in the proper format
- Verifying calibration accuracy

### Using the Setup Application

1. **Connect to your robot**: The app integrates directly with your Viam machine through the Viam SDK
2. **Select your workflow**: Choose from Full Setup, Motor Setup Only, or Calibration Only
3. **Follow guided steps**: The application provides clear instructions and real-time feedback
4. **Save configuration**: Calibration data is automatically saved and applied to your SO-101 components

The setup application provides an intuitive alternative to the DoCommand-based calibration workflow described in the `devrel:so101:calibration` component documentation above.

## SO-101 Resources

For more information about the SO-101 robotic arm:

- [SO-101 Assembly Guide](https://github.com/TheRobotStudio/SO-ARM100)
- [LeRobot Integration](https://huggingface.co/docs/lerobot)
- [Feetech STS3215 Servo Documentation](http://www.feetechrc.com/)
