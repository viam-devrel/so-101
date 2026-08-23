[← Part of the SO-101 module](../README.md)

# Simulated Models

## Model devrel:so101:simulated

The simulated arm component emulates the SO-101 arm entirely in software, with **no hardware required**. It shares the same kinematics as the [`devrel:so101:arm`](arm.md#model-devrelso101arm) model, so it is useful for developing and testing configs, motion plans, and the 3D scene viewer without a physical robot.

When commanded to a joint configuration, the simulated arm interpolates its joints toward the target over time at a configurable speed, just like rdk's builtin `simulated` arm. It also serves 3D meshes for each link, so it renders as an SO-101 in the visualizer.

### Configuration

```json
{}
```

All attributes are optional, so the simulated arm works with an empty configuration.

### Attributes

The following attributes are available for the simulated arm component:

| Name                 | Type   | Inclusion | Description                                                                                                                                              |
| -------------------- | ------ | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `speed_degs_per_sec` | float  | Optional  | How fast each joint travels toward its target, in degrees per second. Default is `90`.                                                                   |
| `motion`             | string | Optional  | Name of the motion service used to plan `MoveToPosition` requests. Default is `"builtin"`.                                                               |
| `simulate_time`      | bool   | Optional  | Whether a background goroutine advances the arm's position in real time. Default is `true`. (Tests set it to `false` to drive the simulated clock.)        |
| `visualize_ee_frame` | bool   | Optional  | When `true`, serves a colored XYZ coordinate-frame marker at the end-effector for the 3D viewer. Default is `false`.                                       |
| `use_urdf`           | bool   | Optional  | When `true`, sources kinematics and collision geometry from the bundled `arm/so101.urdf` instead of the embedded `so101.json`. Requires the `VIAM_MODULE_ROOT` environment variable to be set (viam-server sets this automatically). This is a drop-in swap: the bundled URDF uses `so101.json`'s frame names and TCP exactly, so the frame names, TCP pose, and kinematics are identical — only the collision geometry upgrades from primitive shapes to per-link meshes. Default `false`. |
| `mesh_decimation_ratios` | []float | Optional | Per-collision-mesh simplification ratios, one per arm link in document order (base, shoulder, upper_arm, lower_arm, wrist), each in `[0, 1]`. Only used when `use_urdf` is `true`. Only values strictly in `(0, 1)` decimate (lower = more aggressive). Defaults to `0.9` for each of the 5 meshes when empty. |
| `orientation_tolerance_deg` | float | Optional | Half-angle, in degrees, of the cone of acceptable end-effector approach directions around the goal orientation used by `MoveToPosition` (see [Approach-axis orientation planning](arm.md#approach-axis-orientation-planning) below). **Roll about that axis is NOT constrained.** Valid range `[0, 180]`. `0`/unset means the default (`30`), **not** an exact match. The planner adds a small epsilon to the cone, so the angle actually enforced is `acos(cos(tol) - 0.001)` — always slightly wider than requested (`30` gives ~`30.11`) and never narrower than ~`2.56`, regardless of how small a value you configure. At or above `177.44`, every orientation is accepted. |
| `position_tolerance_mm` | float | Optional | Per-axis positional leeway, in millimeters, of the `MoveToPosition` goal cloud, applied as a box along the goal frame's axes (not a radius) — default `1.0`. Worst-case corner deviation is `sqrt(3)` times this value (~`1.73mm` at the default). Too small a value and the IK solver cannot realistically land inside the cloud, so planning reverts to strict six-DOF scoring and moves fail. |

Example configuration with attributes:

```json
{
  "speed_degs_per_sec": 60
}
```

### Behavior

- `MoveToJointPositions`, `MoveThroughJointPositions`, and `GoToInputs` interpolate the joints toward each target over time; the calls block until the arm arrives.
- `MoveToPosition` plans through the motion service. As with the `devrel:so101:arm` model, it attaches an approach-axis cone (`orientation_tolerance_deg`/`position_tolerance_mm`) to the goal by default instead of ignoring orientation — see [Approach-axis orientation planning](arm.md#approach-axis-orientation-planning) above for the config attributes, the `extra` escape hatches, and the viam-server version requirement.
- `JointPositions`, `EndPosition`, `Geometries`, and `IsMoving` report the simulated state.

### DoCommand

#### Get Motion Parameters

Retrieve the configured joint speed:

```json
{
  "command": "get_motion_params"
}
```

## Model devrel:so101:simulated-gripper

The simulated gripper emulates the SO-101 gripper entirely in software, with **no hardware required**. It serves the gripper meshes (a static body plus a moving jaw/trigger) to the 3D scene viewer, with the moving part posed by the gripper's opening — useful for developing and previewing without a physical robot.

`Open` opens the gripper and `Grab` closes it, interpolating the jaw open/closed over time so it animates in the 3D viewer (like the simulated arm). Both block until the motion finishes, and `IsMoving` reports `true` while the jaw is travelling. A simulated gripper never grasps an object, so `Grab` always reports `false`.

### Configuration

```json
{}
```

All attributes are optional, so the simulated gripper works with an empty configuration.

### Attributes

| Name           | Type   | Inclusion | Description                                                                                                |
| -------------- | ------ | --------- | ---------------------------------------------------------------------------------------------------------- |
| `gripper_type` | string | Optional  | Which gripper meshes to serve: `follower` (moving jaw, the default) or `leader` (thumb-loop handle + trigger). |
| `mesh_detail`  | string | Optional  | Gripper mesh resolution: `low` (decimated, the default) or `high` (full resolution). The mesh is also the motion-planning collision geometry, so `low` keeps planning fast; set `high` on a second gripper for a visual side-by-side. |

The simulated gripper reports the same TCP reference frame as the hardware gripper — see
[Reference frame (TCP)](gripper.md#reference-frame-tcp).

### DoCommand

#### Set Position

Set a target opening percentage (0 = closed, 100 = open); the jaw interpolates toward it:

```json
{
  "command": "set_position",
  "percentage": 50
}
```

#### Get Position

```json
{
  "command": "get_position"
}
```

