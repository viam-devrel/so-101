[← Part of the SO-101 module](../README.md)

# Setup Application

This module includes a web-based setup application that provides guided workflows for configuring your SO-101 robotic arm. The application is hosted as a Viam App and automatically deployed with each module version.

**Access the Setup App**: https://so101-setup_devrel.viamapplications.com

_Add an extra `/` to the end of the URL if you see a "404 Not Found" after authenticating._

## Available Workflows

The setup application provides three main workflows to guide you through different aspects of SO-101 configuration:

### Full Setup (Recommended, 8 steps)

Complete setup workflow from unboxed hardware to fully configured and calibrated SO-101 arm. This comprehensive process includes:

- Motor ID configuration for all servos (1-6)
- Motor communication verification
- Joint calibration: homing positions and recorded range limits
- Saving the resulting calibration data

### Motor Setup Only (4 steps)

Configure servo IDs and communication parameters for SO-101 motors. Use this workflow when you need to:

- Set up servos with proper IDs (1-6) from factory defaults
- Configure communication baudrate (1,000,000 bps)
- Verify motor connectivity and response
- Resolve servo ID conflicts

### Calibration Only (6 steps)

Calibrate joint ranges and homing positions for arms with already configured motors. This workflow covers:

- Setting homing positions for all joints
- Recording joint range limits through guided manual movement
- Reviewing and saving the calibration data

## Using the Setup Application

1. **Select and test your sensor**: The app connects to your Viam machine and lets you pick and test the `devrel:so101:calibration` sensor before starting
2. **Select your workflow**: Choose from Full Setup, Motor Setup Only, or Calibration Only
3. **Follow guided steps**: The application provides clear instructions and real-time feedback
4. **Save configuration**: Calibration data is saved to the robot at the end of the workflow

Each step's "Next" button unlocks only once that step's work has actually succeeded, so a
workflow cannot be clicked past. Progress is tracked for the current browser session only —
reloading the page starts the workflow over, though anything already written to the servos or
the calibration file is unaffected.

The setup application provides an intuitive alternative to the DoCommand-based calibration workflow described in the `devrel:so101:calibration` component documentation above.

