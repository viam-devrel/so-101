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
