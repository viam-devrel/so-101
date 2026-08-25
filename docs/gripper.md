[← Part of the SO-101 module](../README.md)

# Model devrel:so101:gripper

The gripper component controls the 6th servo of the SO-101, which functions as a parallel gripper.

**This component requires an `arm`.** The SO-101 puts all 6 servos on a single serial bus, and the `devrel:so101:arm` component owns that connection. The gripper opens no serial port of its own — it drives its servo by sending [`servo_*` commands](arm.md#servo-commands) to the arm it depends on. So the gripper takes an `arm` attribute naming that component, and has no `port`, `baudrate`, `timeout`, or `calibration_file` of its own.

**Use the `devrel:so101:discovery` service to help with configuring this component automatically.**

Follow the [arm setup steps](../README.md#first-time-arm-setup) to learn how to set up the arm for the first time.

## Configuration

```json
{
  "arm": "arm-1"
}
```

Where `arm-1` is the name of your `devrel:so101:arm` component. Configure the arm first: viam-server builds it before the gripper and will report `dependency not ready` until it exists.

## Attributes

| Name           | Type   | Inclusion    | Description                                                                                                                                                                                                                                          |
| -------------- | ------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `arm`          | string | **Required** | Name of the `devrel:so101:arm` component that owns the serial connection this gripper's servo lives on.                                                                                                                                              |
| `servo_id`     | int    | Optional     | The servo ID for the gripper. Default is `6`.                                                                                                                      |
| `gripper_type` | string | Optional     | Which gripper meshes `Geometries()` serves for the 3D viewer: `follower` (moving jaw, default) or `leader` (thumb-loop handle + trigger). The moving part articulates with the live gripper opening.                                                  |
| `mesh_detail`  | string | Optional     | Gripper mesh resolution: `low` (decimated, the default) or `high` (full resolution). `Geometries()` is also the motion-planning collision geometry, so `low` keeps planning fast; `high` is mainly for visual comparison.                             |

## Reference frame (TCP)

Parent the gripper to the arm with a zero offset — the module supplies the rest:

```json
{
  "name": "gripper",
  "frame": { "parent": "arm" }
}
```

The gripper reports a kinematic model whose frame is its **tool center point**: the grasp point
between the tips of the two jaws, `(x: 6.835, y: 0, z: 99.9)` mm ahead of the arm's `tool` frame,
with the same axes (so `+Z` remains the approach axis). So `GetPose("gripper", "world")` and any
motion request targeting the gripper resolve to the jaw tips, not to the arm's wrist. Do **not** add
a compensating `translation` to the gripper's `frame` config — that would double the offset.

The `leader` gripper is a hand-held trigger rather than a pair of jaws, so it has no grasp point;
its frame stays at the mount.

The model also carries the gripper meshes, which is what makes the gripper a real obstacle for
motion planning: for `arm`/`gantry`/`gripper` components viam-server takes collision geometry from
the kinematic model and never calls `Geometries()`. The model is static, so its collision meshes are
frozen with the jaws **closed**; `Geometries()` still serves the live, articulating jaw to the 3D
viewer.

The 3D viewer reads the model's frames directly, so a `<gripper>:tcp` node appears in the scene tree
with its axes drawn at the grasp point — no extra configuration and no placeholder geometry needed.

## Communication

The gripper owns no serial connection of its own: it asks the arm for its servo's live `Moving` state over [`servo_moving`](arm.md#servo-moving), and if the arm cannot answer — including a remote arm running an older module build that does not know the command — it logs at Debug and falls back to whether a gripper command is in flight, rather than returning an error.

## When the arm fails to build

Because the arm is a required dependency, a gripper cannot start unless its arm starts. If the arm fails to configure, the gripper reports `dependency not ready` rather than coming up on its own. Worth knowing when diagnosing: the arm's startup check pings **all six** servos, so a dead *gripper* servo fails the arm, which in turn keeps the gripper down. Check the arm's logs first when a gripper will not configure.

## DoCommand

The gripper component provides several custom commands:

### Get Gripper Position

Get the current gripper position:

```json
{
  "command": "get_position"
}
```

Response:

```json
{
  "position_percentage": 42.5,
  "position_radians": -0.47,
  "open_position": 95,
  "closed_position": 0
}
```

`position_radians` is derived from the percentage and exists so single-key get/set round-trips keep working.

### Set Gripper Position

Set the gripper opening as a percentage of its calibrated range:

```json
{
  "command": "set_position",
  "percentage": 50
}
```

A raw servo position is also accepted, and is forwarded to the arm — which owns the calibration needed to interpret it — clamped to the servo's calibrated range:

```json
{
  "command": "set_position",
  "servo_position": 2000
}
```

Both percentage write forms — `set_position` above and the `set` shorthand below — accept an
optional `wait` key. It defaults to `true`, which blocks until the jaw stops moving (up to 2
seconds). Pass `false` to return as soon as the move is commanded:

```json
{
  "command": "set_position",
  "percentage": 50,
  "wait": false
}
```

`wait: false` is for callers issuing setpoints faster than the jaw can physically travel — a
control loop at 10 Hz, say — where waiting for a setpoint that is about to be superseded is
pure latency rather than safety. `IsMoving()` then falls back to the servo's own `Moving`
register (see [arm.md](arm.md)) — but against a remote arm running an older module build with
no `servo_moving` support it reports `false` while the jaw is still travelling.

The response's `position` echoes the clamped setpoint, not a measured position — under
`wait: false` the jaw has not arrived yet. Use `{"get": true}` or `get_position` for feedback.

Pacing is the caller's job. The module does not rate-limit writes, and the settle wait was
the only thing bounding them: sustained max-rate writes contend with arm motion on the shared
serial bus.

The flag has no effect on the raw `servo_position` form, which never waited.

`Open()` and `Grab()` always wait and offer no way to opt out: `Grab()` reads the position
back after closing to decide whether it caught anything, so sampling a jaw still in flight
would break grab detection.

The arm takes the same flag, but in the `extra` argument of `MoveToJointPositions`. No
`DoCommand` has an `extra` parameter — the arm's included — so on the gripper's high-rate
write path the command map is the only place to put it.

### Get and Set shorthand

A two-key shorthand covers the common read/write pair. It is the form intended for
high-rate external callers, and it is a supported part of this component's API:

```json
{ "get": true }
```

Response:

```json
{ "position": 42.5 }
```

```json
{ "set": 50.0, "wait": false }
```

Response:

```json
{ "position": 50.0 }
```

`set` clamps to 0–100 and takes the same `wait` key with the same `true` default as
`set_position`. Unlike `set_position` it returns the clamped `position` rather than a
`success` boolean, and it has no raw-tick form.

### Controller Status

Check the status of the serial controller, forwarded from the arm that owns it:

```json
{
  "command": "controller_status"
}
```

### Calibrate Positions

Override the open and closed set points the `Open`/`Grab` API calls drive to, as percentages of the calibrated jaw range. Both arguments are optional and each is ignored unless it is within `[0, 100]`; supply one to change only that end. This tunes the component in memory and is not persisted — use the [calibration sensor](calibration.md#model-devrelso101calibration) to write a calibration file.

```json
{
  "command": "calibrate_positions",
  "open_position": 90,
  "closed_position": 5
}
```

Returns the values in effect after the call:

```json
{
  "success": true,
  "open_position": 90,
  "closed_position": 5
}
```

### Set Motion Params

Set the speed and acceleration used for gripper moves. Both arguments are optional and each is ignored unless it is in range — `speed` in `(0, 180]` degrees per second, `acceleration` in `(0, 500]` degrees per second².

```json
{
  "command": "set_motion_params",
  "speed": 60,
  "acceleration": 300
}
```

Returns the values in effect after the call:

```json
{
  "success": true,
  "speed": 60,
  "acceleration": 300
}
```

### Get Motion Params

Read the current gripper speed and acceleration:

```json
{
  "command": "get_motion_params"
}
```

```json
{
  "speed": 60,
  "acceleration": 300
}
```
