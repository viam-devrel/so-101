# Non-blocking gripper writes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a DoCommand caller write a gripper setpoint without blocking up to 2 s for the jaw to settle, via a `"wait": false` key that defaults to `true` so nothing existing changes.

**Architecture:** The arm component already solved this: `parseWaitExtra` reads `extra["wait"]` (default `true`) and `moveJoints` early-returns before its settle wait. This plan lifts that 7-line parser into `internal/servocmd` — which both components already import, and which already hosts the sibling arg-extractors `NumArg`/`FloatArg`/`ClampPercent` — then threads a `wait bool` through the gripper's `moveToPercent`. `Open` and `Grab` pass `true` explicitly: `Grab` reads the position back to decide its boolean return, so its wait is load-bearing, not latency.

**Tech Stack:** Go 1.25, `go.viam.com/rdk v1.0.0`, `testify`. Build/test with `go test ./...`; format with `gofmt -s -w .`; vet with `go vet ./...`.

**Source spec:** `../viam-vla-inference-service/docs/superpowers/specs/2026-08-24-so101-nonblocking-set-handoff.md` (verified accurate against this repo at commit `f1cbbef`).

---

## Background an engineer with no context needs

- `devrel:so101:gripper` drives **servo 6** only. It owns no serial port — it reaches the
  bus by sending `servo_*` DoCommands to the **arm** component (`g.servoDo`), which may be a
  *remote* arm across gRPC. That is why the flag has to travel in the command map.
- `Gripper.DoCommand` has **no `extra` parameter** in the RDK API (unlike
  `Arm.MoveToJointPositions`). So the wire form is `{"set": 42.0, "wait": false}` — the flag
  rides in the command map itself. This asymmetry with the arm is intentional and must be
  documented, not "fixed".
- There are **two** gripper write paths in `DoCommand`, and they are unrelated code:
  1. the `{"set": <float>}` shorthand (`gripper.go:374`)
  2. `case "set_position":` (`gripper.go:405`), which has *two* sub-branches — a raw
     `servo_position` tick write that **never waited** (leave it alone) and a percentage
     write at `:424` that does.
- `gripperSettleTimeoutMs` is `2000` (`gripper.go:246`). At a caller's 10 fps that is 20
  ticks of latency for a setpoint about to be superseded.
- **Do not add locking changes.** No read path takes `g.mu` — `{"get": true}` (`:366`),
  `get_position` (`:390`), `IsMoving` (`:307`), `Geometries`/`jawAngle` (`:330`, `:341`) are
  all lock-free. Removing the wait is sufficient.
- **Do not "fix" `IsMoving` under `wait: false`.** `defer g.isMoving.Store(false)` fires as
  soon as the command is issued, so `IsMoving` falls through to the hardware `servo_moving`
  query. On a remote arm too old to answer that, the fallback (`:315-319`) reports `false`
  while the jaw still travels. This is identical to the arm's existing `wait: false`
  semantics — consistent, not a new wart.

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `internal/servocmd/servo_commands.go` | Modify (add `WaitArg`) | New shared home for the wait-flag parser, alongside `NumArg`/`FloatArg`/`ClampPercent`. No new import edges — `servocmd` still imports nothing else in the module. |
| `internal/servocmd/wait_test.go` | Create | The parser's table test, moved from `components/arm/wait_test.go`. |
| `components/arm/motion.go` | Modify `:239-249`, `:287`, `:290` | Delete local `parseWaitExtra`; call `servocmd.WaitArg`. Pure refactor, zero behavior change. |
| `components/arm/wait_test.go` | Delete | Superseded by `internal/servocmd/wait_test.go`. |
| `components/gripper/gripper.go` | Modify `:220-229`, `:257`, `:274`, `:383`, `:424` | Thread `wait bool` through `moveToPercent`; both DoCommand write paths read the flag. |
| `components/gripper/gripper_test.go` | Modify (append tests) | Six new assertions, all on *emitted commands*. |
| `docs/gripper.md` | Modify | Document `wait` on both write commands **and** document the previously-undocumented `{"get": true}` / `{"set": v}` shorthand. |

**Design note on Task 1:** the spec proposed a private `gripperWaitExtra` duplicating the
arm's parser. Moving it to `servocmd` instead avoids shipping the duplicate at all, and both
packages already import `servocmd`. The spec's "anything in the arm component is out of
scope" was about *behavior*; this is a mechanical move covered by an existing table test.
If Task 1's review objects, fall back to a private `gripperWaitExtra` in
`components/gripper/gripper.go` with identical semantics and skip Task 1 entirely — every
later task still applies, substituting `gripperWaitExtra` for `servocmd.WaitArg`.

**Amendment (post-Task-1 review):** the helper is named `WaitArg`, not `ParseWaitExtra`.
Its siblings in `servocmd` are `NumArg`/`FloatArg`, and "Extra" is the arm's word — the
gripper passes a *command map*, not an `extra`. The doc comment was also trimmed: an earlier
draft said the gripper reads the flag from the command map "because `Gripper.DoCommand` has
no `extra` parameter", which reads as though the Gripper API is uniquely extra-less. It is
not — `Gripper.Open`/`Grab` both take `extra`, and no `DoCommand` takes one, the arm's
included. The real split is typed method vs `DoCommand`, and the misleading version invited
wiring the flag into `Grab`, whose wait is load-bearing.

---

### Task 1: Move the wait-flag parser into `internal/servocmd`

Pure refactor. No behavior changes anywhere. The existing arm table test moves with it, so
this task starts from a *passing* test rather than a failing one — the guard is that the
test keeps passing after the move and the arm still compiles.

**Files:**
- Modify: `internal/servocmd/servo_commands.go` (add after `ClampPercent`, which ends at `:85`)
- Create: `internal/servocmd/wait_test.go`
- Modify: `components/arm/motion.go:239-249`, `:287`, `:290`
- Delete: `components/arm/wait_test.go`

- [ ] **Step 1: Confirm the current state is green**

Run: `go test ./components/arm/... ./internal/servocmd/... 2>&1 | tail -5`
Expected: `ok` for both packages.

- [ ] **Step 2: Add `WaitArg` to `internal/servocmd/servo_commands.go`**

Insert immediately after the `ClampPercent` function:

```go
// WaitArg reads the optional "wait" bool from a command or extra map. Absent or
// non-bool values default to true, so every existing caller keeps blocking.
func WaitArg(cmd map[string]any) bool {
	if cmd != nil {
		if w, ok := cmd["wait"].(bool); ok {
			return w
		}
	}
	return true
}
```

Note `map[string]any` — that is what the rest of `servocmd` uses, and it is the identical
type to the arm's `map[string]interface{}`, so no call site needs a conversion.

- [ ] **Step 3: Move the table test to `internal/servocmd/wait_test.go`**

Create the file with exactly this content:

```go
package servocmd

import "testing"

func TestWaitArg(t *testing.T) {
	cases := []struct {
		name  string
		cmd   map[string]any
		want  bool
	}{
		{"nil defaults to true", nil, true},
		{"missing key defaults to true", map[string]any{"speed": 1.0}, true},
		{"explicit false", map[string]any{"wait": false}, false},
		{"explicit true", map[string]any{"wait": true}, true},
		{"non-bool ignored", map[string]any{"wait": "no"}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			if got := WaitArg(tc.cmd); got != tc.want {
				t.Fatalf("WaitArg(%v) = %v, want %v", tc.cmd, got, tc.want)
			}
		})
	}
}
```

Then delete the old one:

```bash
rm components/arm/wait_test.go
```

- [ ] **Step 4: Run the moved test**

Run: `go test ./internal/servocmd/ -run TestWaitArg -v 2>&1 | tail -15`
Expected: PASS, five subtests.

- [ ] **Step 5: Point the arm at the shared parser**

In `components/arm/motion.go`, delete the local function and its doc comment (`:239-249`):

```go
// parseWaitExtra reads the optional "wait" bool from a DoCommand/extra map.
// Absent or non-bool values default to true so the motion planner and existing
// callers keep the blocking behavior; teleop passes wait=false to stream setpoints.
func parseWaitExtra(extra map[string]interface{}) bool {
	if extra != nil {
		if w, ok := extra["wait"].(bool); ok {
			return w
		}
	}
	return true
}
```

Then replace both call sites (`:287` and `:290`) — `parseWaitExtra(extra)` becomes
`servocmd.WaitArg(extra)`. Add `"so_arm/internal/servocmd"` to `motion.go`'s import
block if it is not already there (`components/arm/status.go` imports it, but each file needs
its own import).

- [ ] **Step 6: Verify the whole suite is still green**

Run: `go build ./... && go vet ./... && go test ./... 2>&1 | grep -v "^ok" | head -20`
Expected: no output beyond `no test files` lines. If `go build ./...` leaves a stray
`module` binary at the repo root, that is expected and gitignored.

- [ ] **Step 7: Commit**

```bash
gofmt -s -w internal/servocmd/servo_commands.go internal/servocmd/wait_test.go components/arm/motion.go
git add internal/servocmd/servo_commands.go internal/servocmd/wait_test.go components/arm/motion.go components/arm/wait_test.go
git commit -m "refactor: move parseWaitExtra into internal/servocmd as WaitArg"
```

Note: stage explicit paths. Do **not** use `git commit -a`.

---

### Task 2: `moveToPercent` takes a `wait` parameter

The behavior change lands here, but no caller can reach it yet — every call site passes
`true`. Task 2 is therefore a compile-and-stay-green task; Task 3 is where the flag becomes
observable.

**Files:**
- Modify: `components/gripper/gripper.go:220-229` (`moveToPercent`), `:257` (`Open`), `:274` (`Grab`), `:383` (`set` path), `:424` (`set_position` path)
- Test: `components/gripper/gripper_test.go`

- [ ] **Step 1: Write the failing test — `Open` and `Grab` still wait**

These two are the regression guard for the rest of the plan. Append to
`components/gripper/gripper_test.go`:

```go
// Open and Grab are one-shot typed API calls whose contract is "the jaw is now open" /
// "did I grab something". Grab in particular reads the position back to decide its
// boolean return, so without the settle wait it samples a jaw still in flight. Assert on
// the emitted command: the grab-classification tests cannot catch a missing wait, because
// fakeServoArm returns a static f.percent that CmdServoMove never changes.
func TestOpenAndGrabAlwaysWaitForTheJawToSettle(t *testing.T) {
	t.Run("Open", func(t *testing.T) {
		fa := newFakeServoArm()
		g := newTestGripper(t, fa)

		require.NoError(t, g.Open(context.Background(), nil))

		assert.NotNil(t, fa.lastCommand(servocmd.CmdServoWaitStop),
			"Open must wait for the jaw to settle")
	})

	t.Run("Grab", func(t *testing.T) {
		fa := newFakeServoArm()
		g := newTestGripper(t, fa)

		_, err := g.Grab(context.Background(), nil)
		require.NoError(t, err)

		assert.NotNil(t, fa.lastCommand(servocmd.CmdServoWaitStop),
			"Grab reads the position back, so its wait is load-bearing")
	})
}
```

- [ ] **Step 2: Run it — it should PASS already**

Run: `go test ./components/gripper/ -run TestOpenAndGrabAlwaysWaitForTheJawToSettle -v 2>&1 | tail -10`
Expected: PASS. This is deliberate: it pins behavior that already exists so the rest of the
plan cannot silently break it. (`Open`'s wait is also covered by the ordering assertion in
`TestGripperOpenCommandsServoAndWaits` at `gripper_test.go:223`; `Grab`'s was covered by
nothing.)

- [ ] **Step 3: Add the `wait` parameter to `moveToPercent`**

Replace `components/gripper/gripper.go:220-229` entirely:

```go
// moveToPercent commands the servo. When wait is true it blocks until the servo settles;
// a false wait returns as soon as the move is commanded, for callers issuing setpoints
// faster than the jaw can travel (a VLA control loop at 10 fps, for instance), where
// waiting for a setpoint that is about to be superseded is pure latency.
func (g *so101Gripper) moveToPercent(ctx context.Context, percent float64, wait bool) error {
	if _, err := g.servoDo(ctx, servocmd.CmdServoMove,
		map[string]interface{}{"percent": servocmd.ClampPercent(percent)}); err != nil {
		return err
	}
	if !wait {
		return nil
	}
	_, err := g.servoDo(ctx, servocmd.CmdServoWaitStop,
		map[string]interface{}{"timeout_ms": gripperSettleTimeoutMs})
	return err
}
```

- [ ] **Step 4: Pass `true` at all four existing call sites**

The compiler will name them. Each gets a literal `true` — no flag plumbing yet:

- `:257` → `g.moveToPercent(ctx, g.openPosition, true)`
- `:274` → `g.moveToPercent(ctx, g.closedPosition, true)`
- `:383` → `g.moveToPercent(ctx, percentPos, true)`
- `:424` → `g.moveToPercent(ctx, targetPercent, true)`

- [ ] **Step 5: Verify nothing changed**

Run: `go build ./... && go test ./components/gripper/ 2>&1 | tail -5`
Expected: `ok  	so_arm/components/gripper`. Every existing test passes unchanged — that is
the point of this step.

- [ ] **Step 6: Commit**

```bash
gofmt -s -w components/gripper/gripper.go components/gripper/gripper_test.go
git add components/gripper/gripper.go components/gripper/gripper_test.go
git commit -m "refactor(gripper): thread an explicit wait flag through moveToPercent"
```

---

### Task 3: Both DoCommand write paths honor `wait`

**Files:**
- Modify: `components/gripper/gripper.go:374-386` (the `set` shorthand), `:424` (the `set_position` percentage branch)
- Test: `components/gripper/gripper_test.go`

- [ ] **Step 1: Write the failing tests**

Append to `components/gripper/gripper_test.go`. Note every case asserts on the **emitted
command list**, never on a timing or a classification:

```go
// A caller streaming setpoints faster than the jaw travels passes "wait": false, and gets
// the move commanded with no settle wait behind it. The flag rides in the command map
// rather than an extra argument because Gripper.DoCommand has no extra parameter.
func TestGripperWriteCommandsHonorTheWaitFlag(t *testing.T) {
	cases := []struct {
		name     string
		cmd      map[string]any
		wantWait bool
	}{
		{
			name:     "set shorthand with wait false skips the settle wait",
			cmd:      map[string]any{"set": 42.0, "wait": false},
			wantWait: false,
		},
		{
			name:     "set shorthand without the flag still waits",
			cmd:      map[string]any{"set": 42.0},
			wantWait: true,
		},
		{
			name:     "set_position with wait false skips the settle wait",
			cmd:      map[string]any{"command": "set_position", "percentage": 42.0, "wait": false},
			wantWait: false,
		},
		{
			name:     "set_position without the flag still waits",
			cmd:      map[string]any{"command": "set_position", "percentage": 42.0},
			wantWait: true,
		},
		{
			// A JSON string sneaking through must not silently disable the wait on the
			// paths that need it, so a non-bool falls back to waiting.
			name:     "a non-bool wait falls back to waiting",
			cmd:      map[string]any{"set": 42.0, "wait": "false"},
			wantWait: true,
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			fa := newFakeServoArm()
			g := newTestGripper(t, fa)

			_, err := g.DoCommand(context.Background(), tc.cmd)
			require.NoError(t, err)

			move := fa.lastCommand(servocmd.CmdServoMove)
			require.NotNil(t, move, "the move must be commanded either way")
			assert.Equal(t, 42.0, move["percent"])

			waited := fa.lastCommand(servocmd.CmdServoWaitStop) != nil
			assert.Equal(t, tc.wantWait, waited,
				"issued commands: %v", fa.issued())
		})
	}
}
```

- [ ] **Step 2: Run it to watch it fail**

Run: `go test ./components/gripper/ -run TestGripperWriteCommandsHonorTheWaitFlag -v 2>&1 | tail -25`
Expected: FAIL — the two `wait: false` subtests report `Equal: false != true` because every
path still hard-codes `true` from Task 2. The three `wantWait: true` subtests already pass.

- [ ] **Step 3: Honor the flag on the `set` shorthand**

`components/gripper/gripper.go:383` — change the hard-coded `true`:

```go
if err := g.moveToPercent(ctx, percentPos, servocmd.WaitArg(cmd)); err != nil {
	return nil, err
}
```

Leave the surrounding lock and `isMoving` bookkeeping (`:376-381`) untouched.

- [ ] **Step 4: Honor the flag on `set_position`**

`components/gripper/gripper.go:424`:

```go
err := g.moveToPercent(ctx, targetPercent, servocmd.WaitArg(cmd))
return map[string]interface{}{"success": err == nil}, err
```

**Leave the `servo_position` raw-tick branch above it (`:414-417`) alone** — it never issued
a wait in the first place, so there is nothing for the flag to gate.

- [ ] **Step 5: Run the new tests**

Run: `go test ./components/gripper/ -run TestGripperWriteCommandsHonorTheWaitFlag -v 2>&1 | tail -20`
Expected: PASS, all five subtests.

- [ ] **Step 6: Run the full suite**

Run: `go vet ./... && go test ./... 2>&1 | grep -v "^ok" | head -20`
Expected: no failures. In particular `TestGripperOpenCommandsServoAndWaits`,
`TestGripperSetPositionClampsAndCommands`, `TestOpenAndGrabAlwaysWaitForTheJawToSettle`, and
`TestGripperDrivesRealDispatcherEndToEnd` must all still pass.

- [ ] **Step 7: Commit**

```bash
gofmt -s -w components/gripper/gripper.go components/gripper/gripper_test.go
git add components/gripper/gripper.go components/gripper/gripper_test.go
git commit -m "feat(gripper): honor a wait flag on both DoCommand write paths"
```

---

### Task 4: Document `wait` and the undocumented shorthand

`docs/gripper.md` currently documents only the `command:`-style calls. The
`{"get": true}` / `{"set": v}` shorthand — the exact form the requesting module uses — is
documented **nowhere** in the repo (verified: zero hits across `docs/` and `README.md`). An
undocumented shorthand that an external module depends on is the likeliest thing to get
refactored away by accident, so documenting it is part of this change, not a nicety.

**Files:**
- Modify: `docs/gripper.md` — after the `### Set Gripper Position` block that ends at `:114`, before `### Controller Status` at `:116`

- [ ] **Step 1: Add the `wait` key to the Set Gripper Position section**

Insert after the `servo_position` example (i.e. after `docs/gripper.md:114`) and before
`### Controller Status`:

````markdown
Both percentage write forms accept an optional `wait` key. It defaults to `true`, which
blocks until the jaw stops moving (up to 2 seconds). Pass `false` to return as soon as the
move is commanded:

```json
{
  "command": "set_position",
  "percentage": 50,
  "wait": false
}
```

`wait: false` is for callers issuing setpoints faster than the jaw can physically travel — a
control loop at 10 Hz, say — where waiting for a setpoint that is about to be superseded is
pure latency rather than safety. On that path `IsMoving()` reports the servo's own `Moving`
register, since no call is left in flight to ask; against a remote arm running an older
module build with no `servo_moving` support, it reports `false` while the jaw is still
travelling.

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
````

- [ ] **Step 2: Verify the doc matches the code**

Read back `components/gripper/gripper.go:366-372` (the `get` branch returns
`{"position": …}`) and `:374-386` (the `set` branch returns `{"position": percentPos}` after
`ClampPercent`). Confirm the documented response keys and the clamping claim are exact.

Run: `grep -n '"position"' components/gripper/gripper.go`
Expected: hits inside both the `get` and `set` branches.

- [ ] **Step 3: Commit**

```bash
git add docs/gripper.md
git commit -m "docs(gripper): document the wait flag and the get/set shorthand"
```

---

### Task 5: Final verification

- [ ] **Step 1: Format, vet, and test everything**

Run:
```bash
gofmt -s -l . && go vet ./... && go test ./... 2>&1 | grep -v "^ok"
```
Expected: `gofmt -s -l .` prints nothing (any file listed is unformatted); `go vet` prints
nothing; `go test` output shows only `no test files` lines.

- [ ] **Step 2: Confirm the diff is scoped**

Run: `git diff main --stat`
Expected exactly these files: `internal/servocmd/servo_commands.go`,
`internal/servocmd/wait_test.go` (new), `components/arm/motion.go`,
`components/arm/wait_test.go` (deleted), `components/gripper/gripper.go`,
`components/gripper/gripper_test.go`, `docs/gripper.md`, and this plan document. Nothing
under `components/simulated/`, `services/teleop/`, `internal/geometry/`, or `setup-app/`.

- [ ] **Step 3: Confirm no in-repo caller changed behavior**

Run: `grep -rn '"wait"' --include='*.go' components/ services/ internal/`
Expected: `services/teleop/teleop.go:183` (the arm's `{"wait": false}`, pre-existing),
`services/teleop/teleop_test.go:50`, the new `WaitArg` body, and the new tests. Note
`components/arm/motion.go` will **not** match — after Task 1 its call sites read
`servocmd.WaitArg(extra)` with no `"wait"` literal. That absence is correct, not a
missed edit. `services/teleop/teleop.go:189` must still send `set_position` with **no**
`wait` key — the teleop service keeps blocking, and changing that is a separate decision.

---

## Explicitly out of scope

- **Making the gripper 1-DOF so `CurrentInputs`/`GoToInputs` work.** Considered and rejected
  as far too large for the need. `BuildGripperModel` (`internal/geometry/gripper.go:140-181`)
  emits links only — zero joints — with the jaw mesh baked at `GripperJointMin`, so the
  zero-DOF model correctly reports an empty input list and `ErrUnsupported`
  (`gripper.go:497-502`) is not wrong today. Making the pair meaningful would need a revolute
  joint, posing the mesh from the joint value instead of baking it, mirroring all of it in
  `simulated-gripper`, and accepting the jaw as a planning DOF the motion planner can drive.
- **Any behavior change in the arm component.** Task 1 moves a helper; it changes no
  behavior. The arm's `wait` handling is already correct.
- **Switching teleop to `wait: false`.** It would likely benefit at its 20 Hz default
  (`services/teleop/teleop.go:21`), but that is a separate decision.
- **Teaching `simulated-gripper` the shorthand.** Worth knowing, and *not* fixed here: its
  `DoCommand` (`components/simulated/simulated_gripper.go:248`) switches only on
  `cmd["command"]` and accepts only `percentage` — it supports neither `{"get": true}` nor
  `{"set": v}` nor `position_percentage`. So a caller using the shorthand cannot dry-run
  against the simulated model. Back-compat is unaffected either way; if sim-first testing
  matters to the requesting module, raise it as its own change.
