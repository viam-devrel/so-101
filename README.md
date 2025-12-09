# Viam SO-101 Robotic Arm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for the SO-101 5-DOF + Gripper collaborative robotic arm designed by TheRobotStudio and HuggingFace.
It can be used to control either the leader or follower arm, as well as configuring both has separate arm components for mirrored teleoperation!

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This SO-101 module is particularly useful in applications that require the SO-101 arm to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separately through your own code.

Follow the [end-to-end tutorial](https://codelabs.viam.com/guide/so101/index.html?index=..%2F..index#0) to learn how to set up the arm for the first time.

## Model devrel:so101:discovery

Automatic discovery service that detects connected arms and suggests component configurations for the calibration sensor, arm and gripper. It will also look for existing calibration files. 

### Troubleshooting

1. **Serial Connection Failed**:
   - Check that the USB cable is properly connected
   - Verify the correct port (Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`; Windows: `COM3`, `COM4`, etc.)
   - Ensure no other applications are using the serial port
   - Check USB permissions on Linux: `sudo chmod 666 /dev/ttyUSB0`

## Model devrel:so101:arm

The arm component controls the first 5 joints of the SO-101: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, and wrist_roll.

Follow the [end-to-end tutorial](https://codelabs.viam.com/guide/so101/index.html?index=..%2F..index#0) to learn how to set up the arm for the first time.

### Configuration

```json
{
  "port": "/dev/ttyUSB0"
}
```

### Attributes

The following attributes are available for the arm component:

| Name                | Type     | Inclusion | Description                                                                                    |
|---------------------|----------|-----------|------------------------------------------------------------------------------------------------|
| `port`              | string   | **Required**  | The serial port for communication with the SO-101 (see Communication section below).   |
| `calibration_file`  | string   | Optional  | Path to the calibration file. If not provided, uses default calibration values.              |
| `baudrate`          | int      | Optional  | The baud rate for serial communication. Default is `1000000`.                                |
| `servo_ids`         | []int    | Optional  | List of servo IDs for the arm joints. Default is `[1, 2, 3, 4, 5]`.                         |
| `timeout`           | duration | Optional  | Communication timeout. Default is system default.                                            |

**If you're building and setting up an arm for the first time, please see the [calibration sensor component](#model-devrelso101calibration) for setup instructions.**

This may also be necessary if you see inaccuracy issues while controlling the arm.

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

## Model devrel:so101:gripper

The gripper component controls the 6th servo of the SO-101, which functions as a parallel gripper.

Follow the [end-to-end tutorial](https://codelabs.viam.com/guide/so101/index.html?index=..%2F..index#0) to learn how to set up the arm for the first time.

### Configuration

```json
{
  "port": "/dev/ttyUSB0"
}
```

**Use the same `port` and `calibration_file` configuration as the associated arm component.**

### Attributes

| Name               | Type     | Inclusion | Description                                                                                    |
|--------------------|----------|-----------|------------------------------------------------------------------------------------------------|
| `port`             | string   | Required  | The serial port for communication with the SO-101.                                           |
| `calibration_file` | string   | Optional  | Path to the calibration file (shared with arm component).                                    |
| `baudrate`         | int      | Optional  | The baud rate for serial communication. Default is `1000000`.                                |
| `servo_id`         | int      | Optional  | The servo ID for the gripper. Default is `6`.                                                |
| `timeout`          | duration | Optional  | Communication timeout. Default is system default.                                            |

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

## Model devrel:so101:calibration

The SO-101 requires calibration to map servo positions to joint angles based on how the arm was assembled.

The SO-101 Calibration Sensor provides a calibration workflow integrated into Viam's component system. It guides you through the calibration process using DoCommand calls and provides status updates through sensor readings.

**See the [Setup Application](#setup-application) for a visual walkthrough experience that uses this component.**

Follow the [end-to-end tutorial](https://codelabs.viam.com/guide/so101/index.html?index=..%2F..index#0) to learn how to set up the arm for the first time.

### Configuration

```json
{
    "port": "/dev/ttyUSB0",
    "calibration_file": "my_awesome_arm.json",
}
```

#### Attributes

| Name | Type | Required | Description |
|------|------|----------|-------------|
| `port` | string | **Required** | Serial port for servo communication (see Communication section below) |
| `calibration_file` | string | Optional | Path where calibration will be saved. If relative path, uses `$VIAM_MODULE_DATA` directory. Default: `"so101_calibration.json"` |
| `baudrate` | int | Optional | Serial communication speed. Default: `1000000` |
| `timeout` | duration | Optional | Communication timeout. Default: `"5s"` |

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

| Command | Description | Required State |
|---------|-------------|----------------|
| `start` | Begin calibration workflow | `idle`, `completed`, `error` |
| `set_homing` | Set homing offsets and write to servo registers | `started` |
| `start_range_recording` | Begin recording servo ranges | `homing_position` |
| `stop_range_recording` | Complete range recording | `range_recording` |
| `save_calibration` | Write limits to servos and save file | `completed` |
| `abort` | Cancel calibration | Any |
| `reset` | Reset to initial state | `error` |

#### Utility Commands

| Command | Description |
|---------|-------------|
| `get_current_positions` | Read current servo positions |

#### Motor Setup Commands

The calibration sensor also provides motor setup commands for initial SO-101 servo configuration. These commands implement the systematic motor setup process described in `MOTOR_SETUP.md` and are separate from the calibration workflow.

| Command | Description | Parameters |
|---------|-------------|------------|
| `motor_setup_discover` | Discover a single motor connected to the bus | `motor_name` (string): Motor name (e.g., "gripper", "wrist_roll") |
| `motor_setup_assign_id` | Assign target ID and baudrate to discovered motor | `motor_name` (string), `current_id` (int), `target_id` (int), `current_baudrate` (int) |
| `motor_setup_verify` | Verify all SO-101 motors are properly configured | None |
| `motor_setup_scan_bus` | Scan the entire bus for connected servos | None |
| `motor_setup_reset_status` | Reset motor setup status | None |

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
