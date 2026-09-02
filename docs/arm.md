[← Part of the SO-101 module](../README.md)

# Model devrel:so101:arm

The arm component controls the first 5 joints of the SO-101: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, and wrist_roll.

**Use the `devrel:so101:discovery` service to help with configuring this component automatically.**

Follow the [arm setup steps](../README.md#first-time-arm-setup) to learn how to set up the arm for the first time.

## Configuration

```json
{
  "port": "/dev/ttyUSB0"
}
```

## Attributes

The following attributes are available for the arm component:

| Name               | Type     | Inclusion    | Description                                                                                                                                                            |
| ------------------ | -------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `port`             | string   | **Required** | The serial port for communication with the SO-101 (see Communication section below).                                                                                   |
| `calibration_file` | string   | Optional     | Path to the calibration file. If not provided, the module will attempt to read calibration from servo registers. If servo reads fail, uses default calibration values. |
| `baudrate`         | int      | Optional     | The baud rate for serial communication. Default is `1000000`.                                                                                                          |
| `servo_ids`        | []int    | Optional     | List of servo IDs for the arm joints. Default is `[1, 2, 3, 4, 5]`.                                                                                                    |
| `timeout`          | duration | Optional     | Communication timeout. Default is system default.                                                                                                                      |
| `visualize_ee_frame` | bool   | Optional     | When `true`, serves a colored XYZ coordinate-frame marker at the end-effector for the 3D viewer. Default is `false`.                                                    |
| `speed_degs_per_sec` | float  | Optional     | Real per-joint speed cap enforced on the servos, in degrees per second. Applies to the longest-travel joint in a move; other joints are scaled down so all joints arrive together. Default `50`, valid range 3–180. |
| `acceleration_degs_per_sec_per_sec` | float | Optional | Per-joint acceleration cap enforced on the servos, in degrees per second². Scaled alongside speed so joints arrive together; lower is smoother but slower. Default `500`, valid range 50–500 (the servo register cannot deliver acceleration outside it). |
| `waypoint_lookahead_deg` | float | Optional | How far ahead of the arm the commanded goal may run during a `MoveThroughJointPositions`/`GoToInputs` stream, in degrees: the next waypoint is not written until every joint is this close to the current one. **Omit it** — it is otherwise derived per move from the speed and acceleration in force. Set it only to override. Valid range 0.1–45. See [Waypoint streams](#waypoint-streams). |
| `use_urdf`         | bool     | Optional     | When `true`, sources kinematics and collision geometry from the bundled `assets/urdf/so101.urdf` instead of the embedded `so101.json`, and requires `VIAM_MODULE_ROOT` (viam-server sets this). A drop-in swap — frame names, TCP, and kinematics are identical; only the collision geometry upgrades from primitive shapes to per-link meshes. Default `false`. |
| `mesh_decimation_ratios` | []float | Optional | Per-mesh simplification ratios, one per arm link in document order (base, shoulder, upper_arm, lower_arm, wrist). Only values strictly inside `(0, 1)` decimate; lower is more aggressive. Used only with `use_urdf`. Defaults to `0.9` for all five. |
| `manual_mode`      | object   | Optional     | Tuning for hand-guided [manual mode](#manual-mode) (see below). Omit for the built-in defaults. |
| `disable_servo_gains` | bool  | Optional     | When `true`, leaves the servos' P/D/I registers untouched at init instead of writing the tuned defaults. Set it if you tuned gains externally (e.g. with `pidtune apply`). Default `false`. See [Servo gains](#servo-gains). |
| `orientation_tolerance_deg` | float | Optional | Half-angle, in degrees, of the cone of acceptable end-effector approach directions around the goal orientation, used by `MoveToPosition`. **Roll about that axis is NOT constrained.** Range `[0, 180]`; `0`/unset means `30`. See [Approach-axis orientation planning](#approach-axis-orientation-planning). |
| `position_tolerance_mm` | float | Optional | Per-axis positional leeway, in millimetres, of the `MoveToPosition` goal cloud. Default `1.0`. See [Approach-axis orientation planning](#approach-axis-orientation-planning). |
**If you're building and setting up an arm for the first time, please see the [calibration sensor component](calibration.md#model-devrelso101calibration) for setup instructions.**

This may also be necessary if you see inaccuracy issues while controlling the arm.

## DoCommand

The module provides several custom commands accessible through the `DoCommand` interface:

### Set Torque Control

Enable or disable joint torque:

```json
{
  "command": "set_torque",
  "enable": true
}
```

### Ping Servos

Test communication with all servos:

```json
{
  "command": "ping"
}
```

### Controller Status

Check the shared controller status for debugging:

```json
{
  "command": "controller_status"
}
```

### Connection Diagnostics

Run comprehensive connection diagnostics:

```json
{
  "command": "diagnose"
}
```

### Verify Configuration

Verify servo configuration and communication:

```json
{
  "command": "verify_config"
}
```

### Reinitialize Servos

Reinitialize servo communication with retry attempts:

```json
{
  "command": "reinitialize",
  "retries": 3
}
```

### Test Servo Communication

Test communication and read positions from arm servos:

```json
{
  "command": "test_servo_communication"
}
```

### Reload Calibration

Reload calibration from file:

```json
{
  "command": "reload_calibration"
}
```

### Get Calibration

Retrieve current calibration data:

```json
{
  "command": "get_calibration"
}
```

### Enter Manual Mode

Enter [manual (hand-guided) mode](#manual-mode). Accepts the same tuning keys as the `manual_mode` config object (`pos_deadband_deg`, `loop_hz`, `p_gain`, `torque_limit`) inline as optional overrides, which layer on top of the configured `manual_mode` values. Calling this while already in manual mode tears down the current session and starts a fresh one with the new parameters — no need to `exit_manual_mode` first.

```json
{ "command": "enter_manual_mode", "pos_deadband_deg": 3 }
```

Response:

```json
{ "mode": "manual", "servos": [1, 2, 3, 4, 5] }
```

### Exit Manual Mode

Leave manual mode, restore normal servo stiffness, and hold the current pose:

```json
{ "command": "exit_manual_mode" }
```

Response:

```json
{ "mode": "auto" }
```

### Set Speed

Update the per-joint speed cap at runtime without reconfiguring the component. `set_speed` is a key in the request map (not a `"command"` value). Valid range: 3–180 degrees/second.

```json
{
  "set_speed": 60.0
}
```

Response:

```json
{
  "speed_set": 60.0
}
```

### Set Acceleration

Update the acceleration parameter at runtime. `set_acceleration` is a key in the request map (not a `"command"` value). Valid range: 50–500 degrees/second², enforced on hardware.

```json
{
  "set_acceleration": 150.0
}
```

Response:

```json
{
  "acceleration_set": 150.0
}
```

### Get Motion Parameters

Retrieve the current speed and acceleration values in use. `get_motion_params` is a key set to `true` in the request map (not a `"command"` value). Both `set_speed` and `get_motion_params` can be combined in one request.

```json
{
  "get_motion_params": true
}
```

Response:

```json
{
  "current_speed_degs_per_sec": 50.0,
  "current_acceleration_degs_per_sec_per_sec": 500.0
}
```

## Servo commands

The arm owns the serial bus for all 6 servos, so a [`devrel:so101:gripper`](gripper.md#model-devrelso101gripper) on the same SO-101 drives its servo by sending these commands to the arm rather than opening its own connection. They are documented because they are a normal part of the arm's `DoCommand` surface — you can call them directly — but in ordinary use the gripper component issues them for you.

Every command takes a `servo_id`. The arm **rejects any `servo_id` in its own `servo_ids`** list, so these cannot be used to fight the arm for one of its own joints.

### Servo Capabilities

Report which servos on this bus are available to another component, and confirm this arm speaks the protocol. The gripper calls this once at startup so a misconfigured `arm` reference fails at configuration time rather than on the first move.

```json
{
  "command": "servo_capabilities"
}
```

Response on a standard 5-DOF arm:

```json
{
  "servo_commands": true,
  "servo_ids": [6]
}
```

### Move Servo

Move one servo to a percentage of its calibrated range:

```json
{
  "command": "servo_move",
  "servo_id": 6,
  "percent": 95
}
```

Or to a raw encoder value, clamped to the servo's calibrated range:

```json
{
  "command": "servo_move",
  "servo_id": 6,
  "raw": 2000
}
```

`percent` is converted using that servo's own calibration and normalization mode.

### Servo Position

Read one servo's position as both a percentage of its calibrated range and the underlying raw value:

```json
{
  "command": "servo_position",
  "servo_id": 6
}
```

Response:

```json
{
  "percent": 42.5,
  "raw": 1834
}
```

### Stop Servo

Halt a single servo, leaving every other servo on the bus untouched:

```json
{
  "command": "servo_stop",
  "servo_id": 6
}
```

### Servo Moving

Report whether a single servo is currently moving, read from its own `Moving` register — scoped to that servo, so it will not report `true` for arm motion:

```json
{
  "command": "servo_moving",
  "servo_id": 6
}
```

Response:

```json
{
  "moving": false
}
```

### Wait For Servo To Stop

Block until a servo reports it has stopped moving, or the timeout elapses. Scoped to the one servo, so it will not block on arm motion. `timeout_ms` is optional and defaults to `3000`.

```json
{
  "command": "servo_wait_stop",
  "servo_id": 6,
  "timeout_ms": 2000
}
```

## Calibration Priority

The module uses this calibration priority:

1. **File-based** - If `calibration_file` is configured and exists, load from file
2. **Servo registers** - If no file configured/found, read homing offset and range limits from servo hardware
3. **Hardcoded defaults** - If servo reads fail, use default values (offset=0, range=500-3500 for the arm joints; the gripper defaults to the conservative travel window described below)

The servo register fallback provides better out-of-box experience by using actual hardware settings instead of generic defaults. Each startup without a calibration file will re-read from servos to ensure fresh data.

**Note:** Calibration read from servos is used in-memory only and not automatically saved. To persist servo-read calibration, use the calibration sensor component's workflow.

### Gripper travel guard

A gripper whose position limits are still at the factory `0`-`4095` reads as calibrated, but that range is used as a linear scale factor — 0-100% would map across a full encoder revolution, several times the jaw's travel, driving it into its mechanical stops until the servo latches its overload protection.

So when servo 6's range is too wide to be real, the module substitutes a conservative window of ticks `2048`-`3248` and logs a warning naming the cause. The same window is the module's default, so a calibration file that omits its `gripper` entry is also safe. A range that could plausibly be real is never overridden.

**The jaw will not reach full open or closed in this mode.** It is a safe fallback, not a substitute for calibration — run the [calibration sensor's](calibration.md#model-devrelso101calibration) workflow to record the real range, after which the guard no longer applies.

**One case it does not cover.** The window assumes tick `2048` is the jaw's closed stop, which holds on kits homed with the vendor's half-turn convention. This module's own calibration workflow homes from the *middle* of the jaw's travel instead, so a run abandoned after `set_homing` leaves `2048` at mid-travel — and from there `Open()` can strain against the far stop. Finish the workflow rather than relying on the fallback.

## Motion and speed

The hardware arm scales each joint's speed and acceleration by its share of the move's travel, so all joints arrive together. `speed_degs_per_sec` and `acceleration_degs_per_sec_per_sec` (or their per-move overrides, [`set_speed`](#set-speed) and [`set_acceleration`](#set-acceleration)) apply to the longest-travel joint; the rest are scaled down to match it.

**Coordinated scaling has a floor.** A joint scaled down to a very low speed is not merely imprecise — below roughly 10.5 °/s the STS3215's goal-velocity register fails in *opposite* directions depending on load, so neither the commanded speed nor any single correction is recoverable. Measured on an SO-101 at `p_gain` 32, 3 reps per point, 25° sweeps:

| commanded | shoulder (heavy) | elbow (light) | wrist (unloaded) |
|---|---|---|---|
| 4 °/s | 0.80 (0.20×) | 1.57 (0.39×) | 7.60 (1.90×) |
| 8 °/s | 3.13 (0.39×) | 8.82 (1.10×) | 8.78 (1.10×) |
| 10 °/s | 8.18 (bimodal, 4.22–10.19 across reps) | | |
| 10.5 °/s | 10.52 (1.00×) | 10.50 (1.00×) | 10.44 (0.99×) |

The light joints floor hard at ~8.8 °/s — command less and you still get 8.8. The gravity-loaded shoulder does the reverse and under-executes to a fifth of its command. All three track within 1% from 10.5 up. So `MinExecutableSpeedDegsPerSec` is **12**, carrying margin over the worst joint's measured 10.5 and clear of the bimodal 10.0, and every scaled-down joint is floored there: a low-share joint arrives early instead of executing an unpredictable speed. A *stationary* joint is left at the 1-step floor — its goal is its own position, so the value is inert.

That 12 is a tuning value, not a hardware constant. It comes from one arm, three joints, one pose each, on 10–25° sweeps; a payload or a different pose moves the loaded joint's threshold, and short accel-limited segments were not measured. Note also that `speed_degs_per_sec`'s configured minimum is 3 °/s, *below* this floor — the configured range promises speeds the hardware does not honour, and raising it would be a breaking config change.

A per-joint `MaxVelRadsJoints` cap still wins over the floor: a cap slower than 12 °/s is under-executed rather than silently exceeded.

`MoveThroughJointPositions` accepts `MoveOptions.MaxVelRads` and `MaxAccRads` to override the configured defaults for one move, and the per-joint forms `MaxVelRadsJoints` and `MaxAccRadsJoints`. Setting a per-joint slice makes the matching scalar ignored, per `arm.proto`; a slice whose length doesn't match the arm's joint count is rejected. A per-joint limit slows the whole move rather than that joint, so the joints stay coordinated. `GoToInputs` routes through this same path.

### Streamed setpoints: `wait` and `unlimited_accel`

`MoveToJointPositions` accepts two optional booleans in its `extra` map:

```json
{ "wait": false, "unlimited_accel": true }
```

- **`wait`** (default `true`) — block until the move settles, or return as soon as the goal is written.
- **`unlimited_accel`** (default `false`) — command the servos' acceleration register to `0`, the servo's "unlimited" sentinel, instead of the configured `acceleration_degs_per_sec_per_sec` ramp, and skip the coordination read that a ramped move uses to give the other joints a travel reference for arriving together.

Both skips are for a caller **streaming setpoints** rather than making a one-off point-to-point move — the [teleop service](teleop.md) sets it on every follower command. An acceleration ramp is close to a free win for a single move, but it is measurably catastrophic for continuously-updated tracking: `acc=32` quadrupled RMS tracking error and took lag from 40ms to 160ms versus `acc=0`, because the servo re-ramps on every new goal instead of ever reaching speed. The coordination read is skipped for the same reason it's pointless here — coordinated arrival is meaningless for a goal about to be superseded in ~10ms — and skipping it is measured to halve the bus traffic per call, one packet (the goal write) instead of two (a position read, then the write). A plain point-to-point move should keep both the ramp and the coordinated read; do not set `unlimited_accel` there.

### Waypoint streams

`MoveThroughJointPositions` (and `GoToInputs`, which routes through it) paces its waypoints against the arm's real position: a waypoint is commanded, then held until every joint is within `waypoint_lookahead_deg` of it, and only then is the next one written. The arm waits for a full stop only after the **final** waypoint, so intermediate waypoints are still not stop-points — the arm flows through them.

**Why the pacing exists.** Each goal write supersedes the previous one, and an unpaced stream lands on the bus in a few tens of milliseconds, so the servos never act on anything but the last waypoint. A recorded trajectory replayed as a straight line from its first pose to its last (on a nodding motion recorded at 10 Hz, about 3% of the recorded path), and a planned path skipped the obstacle avoidance it was planned for.

**`Stop` ends a stream.** A paced stream runs for seconds, so `Stop` cancels the move before zeroing velocity and returns once the stream has stopped. Zeroing velocity alone would be overwritten by the next waypoint.

**The lookahead is derived per move, and it has two independent lower bounds.**

A servo decelerates as it approaches its goal, over `v²/2a`. Unless the next goal is written *before* that ramp begins, the arm decelerates and re-accelerates at every waypoint — on a 10 Hz recording, ten velocity dips a second, which reads as jerk. So the lookahead must exceed the deceleration distance.

It must *also* exceed the droop the arm parks with, or the dwell is never satisfied and every waypoint burns its full timeout. Measured on gravity-loaded joints: 4.6–5.2° at `p_gain` 16, **2.0–2.3° at 32**, 1.26–1.49° at 48.

The floor is `3.0`, which clears the `p_gain` 32 band with ~30% margin. It was 2.0 — inside that band — and the consequence was visible on hardware: recordings fast enough for the ramp term to lift the lookahead clear of the droop replayed smoothly, while slower ones that fell back to the floor were stuttery and ran roughly 8× their recorded duration.

The lookahead is therefore `max(1.2 · v²/2a, 3.0)`, computed from the speed and acceleration actually in force — which `arm.MoveOptions` can cap well below the configured default, so it is derived per move rather than once per component.

| move | speed | ramp `v²/2a` | lookahead |
|---|---|---|---|
| recorded gripper test | 21.0 °/s | 0.44° | 3.0° (droop floor governs) |
| recorded nod | 25.2 °/s | 0.64° | 3.0° (droop floor governs) |
| default profile | 50 °/s | 2.50° | 3.0° |
| recorded wave | 58.8 °/s | 3.45° | 4.2° |

This replaced a fixed `2.0`, which was correct only below about 45 °/s at the default acceleration. That is why a slow recording replayed smoothly and a fast one felt jerky — confirmed on hardware, where raising a wave replay's lookahead to 4 removed the jerk.

Note the derived value grows with the *square* of speed, so a move at the 180 °/s maximum derives a 38.9° lookahead: at that speed the goal necessarily runs far ahead and the pacing stops gating. Speed and path fidelity genuinely trade against each other here.

**Setting `waypoint_lookahead_deg` overrides all of that.** It is how far ahead of the arm the commanded goal is allowed to run:

- **Larger** — the goal leads by more, the arm never decelerates near a waypoint, motion is faster and cuts more corner between waypoints.
- **Smaller** — the path is tracked more exactly, down to a near-stop at every waypoint at very small values.
- A joint chasing its goal cruises at roughly `sqrt(accel × lookahead)`, so the derived value above replays a 10 Hz recording at close to its recorded duration.

**Most waypoints are never read at all.** The dwell returns where the arm actually was, so when the next goal is already inside the lookahead of that position the dwell would return on its first read — and is skipped outright instead. On a densified stream this is the common case, cutting reads by roughly 8× at a 25 °/s profile and 16× at 80 °/s. The estimate carries a `speed × elapsed` allowance for motion in any direction since the last read, so it can only overstate the distance remaining; a skip is taken only where a real dwell would also have returned immediately.

When a read is needed, position and moving state arrive in **one** bus transaction — the two registers are 11 bytes apart, and each transaction pays the bus's fixed command gap regardless of payload.

**A waypoint the arm has stopped short of ends immediately.** A servo settles where its position error times its P gain balances the load, then reports `Moving = 0` with the goal unreached. When that droop exceeds the lookahead — which the floor above cannot always prevent, since a `p_gain` 16 arm droops 4.8° — the dwell would otherwise have no exit but its deadline. Because it gates on the **worst** joint, one joint parked short makes every waypoint in the stream wait out its full timeout. Measured on hardware: a 1.5 s recorded trajectory took 31.3 s; with the escape, 6.5 s, and path deviation was unchanged (3.8° → 3.7° mean), so the escape removes dead time without cutting corners.

The moving state is consulted from the dwell's second poll onward, never the first — a servo takes about 2 ms to raise `Moving` after a goal write, so an immediate check could read the pre-write zero and abandon a waypoint before the arm started. A failure to read it is not fatal; the dwell falls back to its deadline.

Each waypoint's hold is bounded by that segment's own expected ramp duration, so a waypoint the arm cannot reach costs tens of milliseconds, not a full move timeout.

**One position read per waypoint.** The dwell polls at 100 Hz (one `SyncRead`, ~1 ms) and its read doubles as the next segment's travel reference. A position read failure fails the move, at the start of the stream or during it: without feedback there is no dwell, and an unpaced stream executes only its final waypoint.

`IsMoving()` reports the servos' own `Moving` register, not just whether a move call is in progress: it stays `true` while an interrupted or timed-out move is still settling, and it is the only signal available on the non-blocking (`wait: false`) path, where no call is left in flight to ask. While a move call **is** in flight, it short-circuits to `true` without touching the bus at all.

**It is not a hand-motion detector.** With torque disabled (`set_torque {"enable": false}`) the servos execute no goal command, so the register reads `0` and `IsMoving()` reports `false` while the arm is back-driven by hand. [Manual mode](#manual-mode) is the opposite case: it keeps torque enabled and chases the hand with real goal-position writes, so the servos are genuinely executing goals — `IsMoving()` now reports `true` intermittently throughout a hand-guided session, where it previously reported `false` the whole time.

Each query costs a serial transaction (~1.3ms measured); polling it in a tight loop competes with position reads and writes on the same bus.

`Stop()` does not currently interrupt an in-progress joint move: a move blocks until the servos settle (or the internal safety timeout elapses), so a `Stop()` issued mid-move takes effect only once that move returns. Interrupting in-flight motion is a planned follow-up.

Enforcing acceleration costs some speed, because the servos now ramp instead of jumping straight to full velocity. At the default of 500 °/s² a 20° move takes about 25% longer than before. Lowering the value costs more: at 100 °/s² the same move takes roughly twice as long again.

## Approach-axis orientation planning

The SO-101 is a 5-DOF arm, so most six-DOF pose targets are unreachable exactly. Rather than discard orientation wholesale, `MoveToPosition` attaches a `referenceframe.PoseCloud` to the goal: a cone that constrains where the tool points (the approach axis), while **roll about that axis is free**. `orientation_tolerance_deg` sets the cone's half-angle; `position_tolerance_mm` sets the per-axis positional leeway, applied as a **box along the goal frame's axes, not a radius** — worst-case corner deviation is `sqrt(3)` times the value (~`1.73mm` at the default `1.0`).

Both have floors you cannot configure away. The planner adds a `0.001` epsilon to the orientation leeway, so the angle actually enforced is `acos(cos(tol) - 0.001)`: always slightly wider than requested (`30` gives ~`30.11`), never narrower than ~`2.56`, and at or above `177.44` every orientation is accepted. Setting `0` therefore means the default `30`, **not** an exact match. Too small a `position_tolerance_mm` is worse than none — if the IK solver cannot realistically land inside the cloud, planning reverts to strict six-DOF scoring and moves fail.

**Behavior change for existing users:** this cone is on by default and is **stricter** than the previous behavior, which passed `goal_metric_type: position_only` and accepted any orientation at all. A goal that planned successfully before may now fail to plan. There is deliberately no fallback — a failed plan fails loudly rather than silently accepting an orientation you didn't intend. If you relied on the old orientation-agnostic behavior, pass `extra: {"goal_metric_type": "position_only"}` on the call (see below) to restore it.

**Requires viam-server >= 0.127.0.** Older servers accept a goal cloud but silently ignore it, which means planning falls back to strict six-degree-of-freedom scoring against the exact goal pose and fails.

`MoveToPosition` accepts two mutually exclusive `extra` keys to bypass the default cone:

- `extra: {"goal_metric_type": "position_only"}` — restores the previous orientation-agnostic behavior for that call; no cloud is sent.
- `extra: {"pose_cloud": {...}}` — supplies a raw `referenceframe.PoseCloud`, replacing the cone entirely (expert mode). Keys are **lowercase**: `x`, `y`, `z`, `ox`, `oy`, `oz`, `theta`. Note the Viam docs render these fields as `OX`/`OY` and the protobuf wire format spells them `o_x` — **neither of those is accepted here**; an unknown key is rejected with an error rather than silently ignored, so a typo fails loudly instead of quietly producing a near-zero-leeway cloud that fails every move. Any field you omit means ~zero leeway for that field.

Passing both `goal_metric_type` and `pose_cloud` in the same `extra` map is an error.

## Servo gains

`disable_servo_gains` (bool, default `false`) leaves the servos' `p_gain`/`d_gain`/`i_gain` registers untouched at init. Leave it `false` to get the tuned table below; set it if you have tuned gains of your own (`pidtune apply` writes the same registers this module does).

The tuned per-joint table, against a stock `32/32/0` (gripper `16/32/0`):

| servo id | joint | P | D | I |
|---|---|---|---|---|
| 1 | shoulder pan | 32 | 48 | 0 |
| 2 | shoulder lift | 48 | 64 | 2 |
| 3 | elbow | 48 | 64 | 4 |
| 4 | wrist flex | 48 | 64 | 2 |
| 5 | wrist roll | 48 | 48 | 0 |
| 6 | gripper | *untested — never written* | | |

These are EEPROM writes, applied during `doServoInitialization` (so on every `Reconfigure`, not just the first construction), and skipped register-by-register when the servo already holds the target value — both because EEPROM has finite write endurance and because `initializeServosWithRetry` can run this same path up to 3 times in one bring-up. They are never sent to a servo outside the arm's own `servo_ids`: a 4-DOF arm configured with `servo_ids: [1,2,3,4]` leaves servo 5 alone even though the table has an entry for it. A gain write failing does not fail arm construction — the arm comes up at whatever gains the servo already holds.

**Why this table.** Stock `P=32` is too weak to hold the gravity-loaded joints (shoulder lift, elbow) against load: at `I=0` they sit at a permanent, direction-independent steady-state error that scales as `1/P`. Adding integral gain removes it. Measured on joint 2: steady-state error drops roughly **4×, 0.90° → 0.22°** (10.2 → 2.5 encoder counts, 0.088°/count). The gravity-free joints (pan, wrist roll) need no integral term and get only the derivative bump.

**The write order is D, then I, then P, and a failed write is terminal for that servo.** A non-zero `I` is only stable against a raised `D` — joint 2 oscillated at `D=32, I=4` and was stable at `D=48, I=4` — so writing derivative first means every prefix of the sequence (`{D}`, `{D,I}`, `{D,I,P}`) is at least as stable as the state before it. If a write fails partway, the loop stops for that servo rather than continuing to the next register: continuing could land `{I,P}` without `D` — 48/32/4 on servo 3, the exact combination measured to oscillate — and strand it there in EEPROM with nothing to retry it (a gain-write failure never fails the arm, so nothing re-triggers this path outside the next `Reconfigure`).

**Peak current rises steeply with `D`.** Taking a joint's `D` from 32 to 64 is not free: at `P=32`, a 100-count step measured 388 current units at `D=16` versus 691 at `D=48`. Servos idled at 37–43°C and rose only 1–2°C over the measurement session, so this was not a thermal problem there — but it would matter under sustained high-duty motion.

**Measured on one arm, once.** 12.2–12.3 V bus, 2026-09-02. Unit-to-unit and supply-voltage variation are unquantified. If your arm needs different gains, set `disable_servo_gains: true` and run `pidtune apply` — it writes the same registers this module does.

## Manual Mode

Manual mode is a live jog-by-hand mode using position-deviation admittance. While active, a background control loop keeps each arm servo in position mode holding a goal — so the arm holds against gravity and never goes limp. When you backdrive a joint past a small angular deadband, the goal follows your hand, trailing by the deadband (which leaves a small restoring force so the joint still holds against gravity); when released, the joint holds its new pose. Torque is never fully disabled. The servo's `present_load` register is not used for control — it saturates on the gentlest push and doesn't encode push direction — and is reported only as telemetry.

Manual mode is entered and exited with the [`enter_manual_mode`](#enter-manual-mode) / [`exit_manual_mode`](#exit-manual-mode) DoCommands and is tuned with the optional `manual_mode` config object. Calling `enter_manual_mode` again while already active re-applies with the new parameters (no need to `exit_manual_mode` first) — useful for live tuning.

| Name               | Type    | Inclusion | Description                                                                                                                                                   |
| ------------------ | ------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `pos_deadband_deg` | float   | Optional  | Degrees a joint must be backdriven from its goal before the goal follows your hand. Higher = stiffer/less sensitive. Valid range 0-45. Default `0.5`. |
| `loop_hz`          | float   | Optional  | Control loop rate, in Hz. Valid range 0-200 (`0` uses the default). Default `50`.                                                                               |
| `p_gain`           | int     | Optional  | Reduced servo position P-gain applied during manual mode, restored on exit. Valid range 0-255. Default `8`. Lower = easier to backdrive/lighter feel, but must stay high enough to hold gravity. |
| `torque_limit`     | int     | Optional  | Servo torque cap (STS3215 `torque_limit` register) applied during manual mode and restored on exit. Range 0-1000, default `50`. Lower is easier to backdrive, but it **must stay above the gravity-hold load or the joint will sag**. Unlike `p_gain`, this is what actually makes the servo compliant to a hand — see the tuning notes below. |
All fields are optional; a zero/omitted value falls back to the built-in default, so manual mode is usable with no `manual_mode` config at all.

**The defaults are bench-validated** on an unloaded SO-101: `p_gain: 8`, `torque_limit: 50`, `pos_deadband_deg: 0.5` — easy to guide by hand, and holds a fully extended arm without drooping. Compliance is arm- and payload-dependent, so an arm carrying a payload (or with different servo tuning) may need a higher `torque_limit` to hold its pose; verify on your arm that `dev_deg` at rest stays close to `0`. To leave the servos' configured `p_gain`/`torque_limit` untouched entirely, pass an explicit `0` as an inline override to `enter_manual_mode` (a config `0` is indistinguishable from omitted and uses the default).

```json
{
  "manual_mode": {
    "pos_deadband_deg": 0.5,
    "loop_hz": 50,
    "p_gain": 8,
    "torque_limit": 50
  }
}
```

**Safety:** any motion command auto-exits manual mode first and re-stiffens the arm at its current pose — this includes `MoveToPosition`, `MoveToJointPositions`, `MoveThroughJointPositions`, motion-service planning (`GoToInputs`), and `Stop`. A hand may still be on the arm when this happens, so expect the arm to become rigid and then move. Manual mode is also torn down on `Close`/reconfigure. It has no idle timeout — it stays active until explicitly exited (`exit_manual_mode`) or auto-exited by one of the events above.

The arm's `GetStatus`/`Status` reports the current mode. Normally:

```json
{ "mode": "auto" }
```

While manual mode is active, it also reports live per-joint telemetry (`actual_deg`, `goal_deg`, `dev_deg`, and `load`), which is useful for tuning `pos_deadband_deg`. `load` is telemetry only and is not used for control:

```json
{
  "mode": "manual",
  "manual_mode": {
    "joints": [{ "servo": 1, "actual_deg": 12.3, "goal_deg": 10.3, "dev_deg": 2.0, "load": 0 }]
  }
}
```

**Tuning diagnostic:** the `set_torque_limit` DoCommand writes the `torque_limit` register directly on every arm servo, so you can find the lowest value that still holds the arm before dialing it into `manual_mode`. It requires manual mode to be off and does **not** auto-restore — set it back to `1000` (or reinitialize) when done.

```json
{
  "command": "set_torque_limit",
  "value": 120
}
```

```json
{
  "set_torque_limit": 120,
  "servos": [
    { "servo": 1, "ok": true },
    { "servo": 2, "ok": true }
  ],
  "note": "diagnostic: restore full torque with value 1000 (or reinitialize) when done"
}
```

## Communication

The SO-101 uses serial communication over USB with Feetech STS3215 servos. All 6 servos share one bus, and the arm component owns that connection: it drives servos 1-5 itself and exposes the remaining servo to a gripper component through its [`servo_*` commands](#servo-commands). The gripper therefore configures an `arm` rather than a `port`, and never opens the port itself.

Components that do configure a `port` — an arm, and the [calibration sensor](calibration.md#model-devrelso101calibration) — share one connection when they name the same port, rather than opening it twice. The port stays open until the last of them is removed.

If the port is not present when the machine starts, the component keeps retrying and connects on its own once the device appears. No restart is needed.

You can use the included [discovery service](discovery.md#model-devrelso101discovery) or find the available serial port options from your machine's command line.

On MacOS, look for `usbmodem` or `usbserial` in the name:

```
you@machine: ls /dev/tty.*
/dev/tty.Bluetooth-Incoming-Port
/dev/tty.debug-console
/dev/tty.usbmodem58CD1767051

you@machine: ls /dev/cu.*
/dev/cu.Bluetooth-Incoming-Port
/dev/cu.debug-console
/dev/cu.usbmodem58CD1767051
```

On Linux, look for `ACM` or `USB` in the name:

```
you@machine: ls /dev/tty*
/dev/ttyACM0
/dev/ttyUSB0
```

On Windows, look for `COM` in the name:

```
you@machine: mode
COM0
COM1
```
