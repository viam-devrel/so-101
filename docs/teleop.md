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
| `rate_hz`                | number | Optional     | Mirror loop frequency in Hz. Default `100`. Must be > 0 and ≤ 1000.                                                                                                                  |
| `manage_leader_torque`   | bool   | Optional     | When `true`, disables the leader arm's servo torque on start and restores it on stop, so the leader can be moved by hand. Default `true`. |
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
  "rate_hz": 100
}
```

Raising the rate is the single most effective smoothness lever — measured 30 → 100 Hz cut
chatter ~3.5× and tracking error ~28%, more than any servo gain change, and those figures
were measured at unlimited acceleration, which is how teleop now drives the follower. Lower
it only if the serial bus or host cannot keep up; the mirror loop's ticker drops ticks rather
than backing up, so a rate the bus cannot sustain degrades silently to the rate it can.

`streaming` reports whether the mirror has switched to streamed setpoints, which carry no speed cap and no acceleration ramp. Because that means a large position error is closed at full servo speed, the mirror uses the arm's configured speed and acceleration until the follower is within 5° of the leader, then switches and stays switched. Starting with the arms far apart is safe, but the follower will travel to meet the leader as soon as the loop starts.

`cycles` is the total number of successful mirror cycles since the last start. `consecutive_errors` resets to zero on any successful cycle. When `max_consecutive_errors` is reached the loop sets `running` to `false` and records the triggering error in `last_error`.

## Notes

- Joint targets are sent to the follower with `wait: false` (non-blocking), so the follower continuously chases the most recent leader pose without queuing up moves.
- When `manage_leader_torque` is enabled (the default), torque is restored on the leader when the loop stops for any reason — including reaching the error threshold or calling `Close` on the service.
- To use without gripper mirroring, omit both `leader_gripper` and `follower_gripper`.

