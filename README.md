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

## Calibration

The SO-101 requires calibration to map servo positions to joint angles. Follow [LeRobot's calibration docs](https://huggingface.co/docs/lerobot/so101#calibrate) to generate this file.

Calibration data is stored in JSON format:

```json
{
  "shoulder_pan": {
    "id": 1,
    "drive_mode": 0,
    "homing_offset": 0,
    "range_min": 500,
    "range_max": 3500
  },
  "shoulder_lift": {
    "id": 2,
    "drive_mode": 0,
    "homing_offset": 0,
    "range_min": 500,
    "range_max": 3500
  },
  "elbow_flex": {
    "id": 3,
    "drive_mode": 0,
    "homing_offset": 0,
    "range_min": 500,
    "range_max": 3500
  },
  "wrist_flex": {
    "id": 4,
    "drive_mode": 0,
    "homing_offset": 0,
    "range_min": 500,
    "range_max": 3500
  },
  "wrist_roll": {
    "id": 5,
    "drive_mode": 0,
    "homing_offset": 0,
    "range_min": 500,
    "range_max": 3500
  },
  "gripper": {
    "id": 6,
    "drive_mode": 0,
    "homing_offset": 0,
    "range_min": 500,
    "range_max": 3500
  }
}
```

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
