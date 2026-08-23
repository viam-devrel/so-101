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

## Documentation

This module ships seven models, split into focused per-model docs:

- [`devrel:so101:arm`](docs/arm.md) — the hardware arm component
- [`devrel:so101:simulated`](docs/simulated.md) — hardware-free simulated arm
- [`devrel:so101:gripper`](docs/gripper.md) — the hardware gripper component
- [`devrel:so101:simulated-gripper`](docs/simulated.md) — hardware-free simulated gripper
- [`devrel:so101:calibration`](docs/calibration.md) — the calibration sensor workflow
- [`devrel:so101:discovery`](docs/discovery.md) — the auto-discovery service
- [`devrel:so101:teleop`](docs/teleop.md) — leader/follower teleoperation service

Also see:

- [Troubleshooting](docs/troubleshooting.md)
- [Setup Application](docs/setup-app.md)

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

## SO-101 Resources

For more information about the SO-101 robotic arm:

- [SO-101 Assembly Guide](https://github.com/TheRobotStudio/SO-ARM100)
- [LeRobot Integration](https://huggingface.co/docs/lerobot)
- [Feetech STS3215 Servo Documentation](http://www.feetechrc.com/)
