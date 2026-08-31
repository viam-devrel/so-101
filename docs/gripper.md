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
| `articulated_jaw` | bool | Optional  | Gives the gripper a 1-DoF jaw joint and makes it `InputEnabled`, so the motion planner may drive the jaw during arm moves. Default `false`. See [Articulated jaw](#articulated-jaw) below.                                                          |

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

## Articulated jaw

`articulated_jaw` is off by default: the jaw does not affect the TCP, so turning it on only
gives the motion planner an extra degree of freedom that changes collision geometry, at the
cost of a `GoToInputs` call on every trajectory step of every plan (see the gotcha in the
repo's `CLAUDE.md`).

With it on, the gripper is `InputEnabled` and reports the jaw angle in **radians** from
`CurrentInputs`. That reading is cached and may be up to 50 ms stale; if the bus is down, it
instead serves the last known good reading, which can be arbitrarily stale.

The planner is free to move the jaw during an ordinary arm move — it is part of the same
trajectory as the arm joints, not a separate grasp action. A command that would actually move
the jaw **while the gripper is holding something** is refused with an error, to avoid dropping
whatever is held; a no-op command (the jaw's current angle, which is what most trajectory steps
carry) is never refused and never commands the servo, whether or not something is held — it may
still issue a `servo_position` read on a cache miss.

"Holding" is inferred by comparing the jaw's current reading against the last percent it was
*commanded* to reach, not the closed position: if the jaw failed to get there by more than the
grasp threshold, something must be blocking it, whichever direction it was moving. Comparing
against the closed position instead is wrong — any merely open jaw then looks held — and did
exactly that on real hardware until it was fixed. `IsHoldingSomething` follows the same rule and,
like the guard above, reports NOT holding whenever nothing has been commanded yet (at startup, or
right after a raw `set_position` in ticks), since there is no commanded target to compare against.

**How "holding" is determined.** The gripper latches a grasp when a command completes, rather than
inferring it from the jaw's position on demand. Two things measured on real hardware make the
live inference impossible: a thin part held in the jaw reads ~0.8% open, indistinguishable from an
empty closed jaw; and the servo's overload flag is not dependable -- a stock STS3215 trips
overload at 80% torque while a hard clamp draws only ~40%, so a normal grip never raises it, and
when it does fire it tracks the standing condition rather than marking a moment. So a close latches a grasp if the jaw stops short of
its target or the servo overloads clamping, and an open that reaches its target clears it. An
overload while closing is reported as success, not an error -- clamping is what a close is for.
A raw `set_position` (servo ticks) deliberately leaves the latch alone: an opaque target is no
evidence of release, and dropping the latch there would disable the grasp-retention guard while a
part is still held. Use `Open` to release.

**Servo conditions appear as data, not failures.** `get_position` and `get_load` may include a
`condition` field (for example `"overload"`) when the servo is reporting its own state — straining
against a held object, overheating. A condition does **not** mean the reading is invalid: the servo
answered correctly and is telling you why it is unhappy. It is also how a grasp on a thin object is
detected at all, since such an object can close the jaw to a position indistinguishable from empty.
The field is absent when there is nothing to report.

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

`Open()` and `Grab()` always wait and offer no way to opt out: both settle the grasp latch from
whether the jaw reached its target (see "How \"holding\" is determined" above), so sampling a jaw
still in flight would break grasp detection. For the same reason `wait: false` leaves the latch
untouched — the move has not arrived anywhere yet.

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
### Get Load

Read the servo's raw signed present-load register (sign-magnitude, not a percentage):

```json
{
  "command": "get_load"
}
```

Response:

```json
{
  "load": -412
}
```

This exists so load can be sampled (e.g. from the CLI) while investigating grasp behavior. It plays no part in the gripper's own holding decision -- see "Holding" above.

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
