[← Part of the SO-101 module](../README.md)

# Model devrel:so101:teleop

A generic service that mirrors a configured **leader** SO-101 arm's joint positions (and gripper opening) onto a configured **follower** arm in real time, enabling hand-guided teleoperation. While the loop is running the leader's servo torque is disabled so the arm can be back-driven by hand; the follower continuously chases the latest leader pose.

Leader and follower must be separate SO-101 arm components, typically on separate serial ports (one USB cable per arm).

## Configuration

```json
{
  "leader_arm": "arm-leader",
  "follower_arm": "arm-follower",
  "leader_gripper": "gripper-leader",
  "follower_gripper": "gripper-follower"
}
```

## Attributes

| Name                     | Type   | Inclusion    | Description                                                                                                                                                                          |
| ------------------------ | ------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `leader_arm`             | string | **Required** | Name of the arm component to read joint positions from.                                                                                                                             |
| `follower_arm`           | string | **Required** | Name of the arm component to drive.                                                                                                                                                  |
| `leader_gripper`         | string | Optional     | Name of the leader gripper component. When both `leader_gripper` and `follower_gripper` are set, gripper opening is mirrored in addition to joint positions.                         |
| `follower_gripper`       | string | Optional     | Name of the follower gripper component.                                                                                                                                              |
| `rate_hz`                | number | Optional     | Mirror loop frequency in Hz. Default `20`. Must be > 0 and ≤ 1000.                                                                                                                  |
| `manage_leader_torque`   | bool   | Optional     | When `true` (default), disables leader arm torque on start so the arm can be back-driven by hand, and restores torque on stop. The torque command covers every servo on the leader's bus, so this also frees the leader gripper servo. |
| `auto_start`             | bool   | Optional     | When `true`, starts the mirror loop immediately when the service is configured. Default `false`.                                                                                     |
| `max_consecutive_errors` | int    | Optional     | Number of consecutive read/write failures after which the loop stops automatically and reports the error via `status`. Default `10`.                                                 |

## DoCommand

### Start

Start the mirror loop:

```json
{ "command": "start" }
```

Response:

```json
{ "running": true }
```

### Stop

Stop the mirror loop:

```json
{ "command": "stop" }
```

Response:

```json
{ "running": false }
```

### Status

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

## Notes

- Joint targets are sent to the follower with `wait: false` (non-blocking), so the follower continuously chases the most recent leader pose without queuing up moves.
- When `manage_leader_torque` is enabled (the default), torque is restored on the leader when the loop stops for any reason — including reaching the error threshold or calling `Close` on the service.
- To use without gripper mirroring, omit both `leader_gripper` and `follower_gripper`.

