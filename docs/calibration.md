[← Part of the SO-101 module](../README.md)

# Model devrel:so101:calibration

When assembling the SO-101 arm from a kit, it requires calibration to map servo positions to joint angles based on how the arm was assembled.

The SO-101 Calibration Sensor provides a calibration workflow integrated into Viam's component system. It guides you through the calibration process using DoCommand calls and provides status updates through sensor readings.

**IF YOU ARE SETTING UP THIS ARM FOR THE FIRST TIME: Follow the [arm setup steps](../README.md#first-time-arm-setup) to learn how to set up the arm for the first time.**

**See the [Setup Application](https://so101-setup_devrel.viamapplications.com) for a visual walkthrough experience that uses this component.**

_If you have already calibrated the servos on this arm, use the `devrel:so101:discovery` service to help with configuring this component automatically._

## Configuration

```json
{
  "port": "/dev/ttyUSB0",
  "calibration_file": "my_awesome_arm.json"
}
```

### Attributes

| Name               | Type     | Required     | Description                                                                                                                     |
| ------------------ | -------- | ------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `port`             | string   | **Required** | Serial port for servo communication (see Communication section below)                                                           |
| `calibration_file` | string   | Optional     | Path where calibration will be saved. If relative path, uses `$VIAM_MODULE_DATA` directory. Default: `"so101_calibration.json"` |
| `baudrate`         | int      | Optional     | Serial communication speed. Default: `1000000`                                                                                  |
| `timeout`          | duration | Optional     | Communication timeout. Default: `"5s"`                                                                                          |
| `servo_ids`        | []int    | Optional     | Servos to calibrate. Defaults to all six (`[1, 2, 3, 4, 5, 6]`).                                                                 |

## Communication

You can use the included [discovery service](discovery.md#model-devrelso101discovery) or find the available serial port options from your machine's command line.

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

## Usage

### Monitor Progress

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

## Available Commands

```json
{
  "command": "command_name"
}
```

### Workflow Commands

| Command                 | Description                                     | Required State               |
| ----------------------- | ----------------------------------------------- | ---------------------------- |
| `start`                 | Begin calibration workflow                      | `idle`, `completed`, `error` |
| `set_homing`            | Set homing offsets and write to servo registers | `started`                    |
| `start_range_recording` | Begin recording servo ranges                    | `homing_position`            |
| `stop_range_recording`  | Complete range recording                        | `range_recording`            |
| `save_calibration`      | Write limits to servos and save file            | `completed`                  |
| `abort`                 | Cancel calibration                              | Any                          |
| `reset`                 | Reset to initial state                          | `error`                      |

Each returns `success`, the resulting `state`, and a `message` carrying the next instruction. A full run:

```json
{ "command": "start" }
```
```json
{ "success": true, "state": "started", "message": "Move each joint to the middle of its range of motion, then call set_homing." }
```

```json
{ "command": "set_homing" }
```
```json
{ "success": true, "state": "homing_position", "homing_offsets": { "shoulder_pan": -103, "shoulder_lift": 47 } }
```

```json
{ "command": "start_range_recording" }
```
```json
{ "success": true, "state": "range_recording", "message": "Recording. Move all joints through their full ranges." }
```

```json
{ "command": "stop_range_recording" }
```
```json
{
  "success": true,
  "state": "completed",
  "samples_collected": 306,
  "recording_duration": 15.3,
  "ranges": { "shoulder_pan": { "min": 758, "max": 3292, "range": 2534 } }
}
```

```json
{ "command": "save_calibration" }
```
```json
{ "success": true, "state": "idle", "joints_calibrated": 6, "calibration_file": "/root/.viam/module-data/.../so101_calibration.json" }
```

`abort` cancels a run from any state; `reset` clears the `error` state. Both take no arguments and
return the same `success`/`state`/`message` shape:

```json
{ "command": "abort" }
```
```json
{ "command": "reset" }
```

### Utility Commands

| Command                 | Description                  |
| ----------------------- | ---------------------------- |
| `get_current_positions` | Read current servo positions |

```json
{ "command": "get_current_positions" }
```
```json
{
  "success": true,
  "positions": {
    "shoulder_pan": { "servo_id": 1, "raw_position": 2150, "degrees": 9.0, "radians": 0.157 }
  }
}
```

### Motor Setup Commands

The calibration sensor also provides motor setup commands for initial SO-101 servo configuration. They assign each servo its ID and baudrate for the first time, and are separate from the calibration workflow. See [Motor Setup Workflow](#motor-setup-workflow) below for the order to run them in.

| Command                    | Description                                       | Parameters                                                                             |
| -------------------------- | ------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `motor_setup_discover`     | Discover a single motor connected to the bus      | `motor_name` (string): Motor name (e.g., "gripper", "wrist_roll")                      |
| `motor_setup_assign_id`    | Assign target ID and baudrate to discovered motor | `motor_name` (string), `current_id` (int), `target_id` (int), `current_baudrate` (int) |
| `motor_setup_verify`       | Verify all SO-101 motors are properly configured  | None                                                                                   |
| `motor_setup_scan_bus`     | Scan the entire bus for connected servos          | None                                                                                   |
| `motor_setup_reset_status` | Reset motor setup status                          | None                                                                                   |

`motor_setup_scan_bus` reports every servo answering on the bus, and flags any ID outside the
expected 1-6 as unexpected:

```json
{ "command": "motor_setup_scan_bus" }
```
```json
{
  "success": true,
  "servos_found": 6,
  "unexpected_count": 0,
  "servos": [{ "id": 1, "model": "STS3215", "status": "ok" }]
}
```

`motor_setup_reset_status` clears the recorded setup progress and takes no arguments:

```json
{ "command": "motor_setup_reset_status" }
```

### Motor Setup Workflow

The motor setup process should be performed in reverse order (gripper → shoulder_pan) to avoid ID conflicts:

1. **Connect only one motor** (e.g., gripper) to the controller
2. **Discover**: `{"command": "motor_setup_discover", "motor_name": "gripper"}`
3. **Assign ID**: `{"command": "motor_setup_assign_id", "motor_name": "gripper", "current_id": 1, "target_id": 6, "current_baudrate": 57600}`
4. Repeat for each motor in order: wrist_roll → wrist_flex → elbow_flex → shoulder_lift → shoulder_pan
5. **Verify**: `{"command": "motor_setup_verify"}` (connect all motors)

### Motor Setup Status

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

## State Machine

The calibration sensor operates as a state machine:

- **`idle`**: Ready to start calibration
- **`started`**: Torque disabled, ready for homing position
- **`homing_position`**: Homing set, ready for range recording
- **`range_recording`**: Recording min/max positions
- **`completed`**: Calibration data ready to save
- **`error`**: Error occurred, use reset command

## Calibration File Output

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

## Troubleshooting

### Common Issues

- **Range recording not working**: Ensure you call `start_range_recording` and manually move joints
- **Invalid ranges**: Move joints through their complete range of motion
- **Servo communication errors**: Check port, baudrate, and servo connections
- **Permission denied**: Ensure proper access to serial port (`sudo chmod 666 /dev/ttyUSB0`)

