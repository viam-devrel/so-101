# Viam SO-101 Robotic Arm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for the SO-101 5-DOF + Gripper collaborative robotic arm designed by TheRobotStudio and HuggingFace.
It can be used to control either the leader or follower arm, as well as configuring both has separate arm components for mirrored teleoperation!

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This SO-101 module is particularly useful in applications that require the SO-101 arm to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separately through your own code.

Navigate to the **CONFIGURE** tab of your machine's page in [the Viam app](https://app.viam.com/). Click the **+** icon next to your machine part in the left-hand menu and select **Component**. Select the `arm` type, then search for and select the `arm / devrel:so101:arm` model. Click **Add module**, then enter a name or use the suggested name for your arm and click **Create**.

> [!NOTE]
> Before configuring your SO-101, you must [add a machine](https://docs.viam.com/fleet/machines/#add-a-new-machine).

## Model devrel:so101:arm

The arm component controls the first 5 joints of the SO-101: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, and wrist_roll.

### Configuration

```json
{
  "port": "/dev/ttyUSB0",
  "baudrate": 1000000,
  "calibration_file": "/path/to/calibration.json"
}
```

### Attributes

The following attributes are available for the arm component:

| Name                | Type     | Inclusion | Description                                                                                    |
|---------------------|----------|-----------|------------------------------------------------------------------------------------------------|
| `port`              | string   | Required  | The serial port for communication with the SO-101 (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`).   |
| `baudrate`          | int      | Optional  | The baud rate for serial communication. Default is `1000000`.                                |
| `servo_ids`         | []int    | Optional  | List of servo IDs for the arm joints. Default is `[1, 2, 3, 4, 5]`.                         |
| `timeout`           | duration | Optional  | Communication timeout. Default is system default.                                            |
| `calibration_file`  | string   | Optional  | Path to the calibration file. If not provided, uses default calibration values.              |

### Communication

The SO-101 uses serial communication over USB with Feetech STS3215 servos. The module uses a shared controller architecture to manage all 6 servos while preventing resource conflicts when both arm and gripper components are used.

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

### Configuration

```json
{
  "port": "/dev/ttyUSB0",
  "baudrate": 1000000,
  "servo_id": 6,
  "calibration_file": "/path/to/calibration.json"
}
```

### Attributes

| Name               | Type     | Inclusion | Description                                                                                    |
|--------------------|----------|-----------|------------------------------------------------------------------------------------------------|
| `port`             | string   | Required  | The serial port for communication with the SO-101.                                           |
| `baudrate`         | int      | Optional  | The baud rate for serial communication. Default is `1000000`.                                |
| `servo_id`         | int      | Optional  | The servo ID for the gripper. Default is `6`.                                                |
| `timeout`          | duration | Optional  | Communication timeout. Default is system default.                                            |
| `calibration_file` | string   | Optional  | Path to the calibration file (shared with arm component).                                    |

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
| `port` | string | **Required** | Serial port for servo communication (e.g., `/dev/ttyUSB0`) |
| `baudrate` | int | Optional | Serial communication speed. Default: `1000000` |
| `servo_ids` | []int | Optional | List of servo IDs to calibrate. Default: `[1,2,3,4,5]` (arm only) |
| `calibration_file` | string | Optional | Path where calibration will be saved. If relative path, uses `$VIAM_MODULE_DATA` directory. Default: `"so101_calibration.json"` |
| `timeout` | duration | Optional | Communication timeout. Default: `"5s"` |

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

## SO-101 Resources

For more information about the SO-101 robotic arm:

- [SO-101 Assembly Guide](https://github.com/TheRobotStudio/SO-ARM100)
- [LeRobot Integration](https://huggingface.co/docs/lerobot)
- [Feetech STS3215 Servo Documentation](http://www.feetechrc.com/)
