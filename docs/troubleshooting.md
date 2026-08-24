[← Part of the SO-101 module](../README.md)

# Troubleshooting

## Connection Issues

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

