[← Part of the SO-101 module](../README.md)

# Model devrel:so101:discovery

Automatic discovery service that detects connected arms and suggests component configurations for the calibration sensor, arm and gripper. It will also look for existing calibration files.

**IF YOU ARE BUILDING THIS ARM FOR THE FIRST TIME: add the calibration sensor and follow the setup instructions**

**Usage:**

- save your robot configuration with this service
- open the Test panel on service configuration card or view the Control tab
- click "+ add component" for the relevant component you'd like to set up: arm, gripper, or calibration sensor

## Troubleshooting

1. **Serial Connection Failed**:
   - Check that the USB cable is properly connected
   - Verify the correct port (Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`; Windows: `COM3`, `COM4`, etc.)
   - Ensure no other applications are using the serial port
   - Check USB permissions on Linux: `sudo chmod 666 /dev/ttyUSB0`

