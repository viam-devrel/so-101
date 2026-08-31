# ConditionStatus Adoption Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Adopt `feetech` v0.7.0's `ConditionStatus` so a servo condition (overload, overheat, voltage) stops being an *error* that discards data, and becomes *data* that crosses the `servocmd` boundary intact.

**Architecture:** `internal/controller` keeps the reading it already has instead of throwing it away, and reports the condition flags alongside it. `ServoOps` carries those flags, the `servo_position`/`servo_load` DoCommand responses carry them as a field, and `components/gripper` reads that field instead of pattern-matching an error string.

**Tech Stack:** Go 1.25, `github.com/hipsterbrown/feetech-servo v0.7.0`, `go.viam.com/rdk v1.0.0`, testify.

**Branch:** `gripper-grasp-latch` (PR #42).

---

## Why this is not a mechanical swap

The gripper currently learns that it is holding something **from a read failing**. `positionPercentCached` sets `jawConditionSeen` when `positionPercent` returns an overload error, and that flag drives the retroactive grasp latch — the mechanism verified on hardware.

Adopting v0.7.0 correctly means `ServoPositionPercent` **succeeds** during overload. That silently destroys the latch's only evidence unless the condition is carried forward deliberately. **Any step that makes reads succeed without first carrying the flags forward will break grasp detection and the tests will not necessarily catch it** — the latch tests use a fake that fails reads, so they would keep passing while hardware regressed.

Order matters: carry the flags first, switch the consumer, then let reads succeed.

## Facts this rests on

- `ConditionStatus(err) (StatusError, bool)` returns `ok=true` only when the data alongside `err` is safe to use — false for transport failures and for request-rejection flags (checksum/instruction/range).
- `Servo.Position`/`Servo.Load` already return the value *and* the error: `if len(data) == 0 { return 0, err }`.
- `Ping` in v0.7.0 tolerates condition flags, which fixed the arm-init cascade. No work needed here.
- Typed errors do **not** survive the `servocmd` DoCommand boundary to a remote arm — they arrive as gRPC status text. This is the entire reason flags must travel as a response field rather than as an error.

---

## File Structure

| File | Change |
|---|---|
| `internal/controller/manager.go` | `ServoPositionPercent`/`ServoLoad` keep the reading, return flags |
| `internal/controller/wait_motion.go` | `isConditionError` → `ConditionStatus`; `tolerateConditions` unchanged in shape |
| `internal/servocmd/servo_commands.go` | `ServoOps` signatures; `condition` in the two responses |
| `internal/testfake/fake_servo_ops.go` | fake gains condition fields |
| `components/gripper/gripper.go` | read `condition` from the response; delete `isOverload` string matching |
| `components/gripper/gripper_test.go` | fake arm emits `condition`; latch tests use it |
| `docs/gripper.md`, `CLAUDE.md` | document conditions-as-data |

---

## Task 1: Controller stops discarding readings

**Files:** `internal/controller/manager.go`, `internal/controller/wait_motion.go`, plus a test.

- [ ] **Step 1: Write the failing test**

In `internal/controller`, a unit test that a condition error no longer costs the reading. If a bus-level fake is impractical, test the decision helper instead — extract if needed, but do not skip the assertion.

- [ ] **Step 2: Change the two readers**

`ServoPositionPercent` and `ServoLoad` currently do `if err != nil { return error }`, discarding a value the library handed back. Use:

```go
raw, err := servo.Position(ctx)
if flags, ok := feetech.ConditionStatus(err); ok {
    condition = flags   // raw is valid; the servo is just unhappy
} else if err != nil {
    return fmt.Errorf("failed to read position for servo %d: %w", id, err)
}
```

Both gain a condition return. Suggested shape, matching the library's own idiom:

```go
ServoPositionPercent(ctx, id) (percent float64, raw int, condition feetech.StatusError, err error)
ServoLoad(ctx, id) (load int, condition feetech.StatusError, err error)
```

- [ ] **Step 3: Replace `isConditionError` with `ConditionStatus`**

`wait_motion.go`'s `isConditionError` is a string-matching stopgap written before v0.7.0. Delete it; `tolerateConditions` uses `ConditionStatus`. Keep `tolerateConditions` a named wrapper — its test exercises production code precisely because it is not inlined.

**Careful:** `ConditionStatus` returns `ok=false` for request-rejection flags where the old helper also returned false, and `ok=false` for transport failures where it also returned false. Confirm the existing `TestIsConditionErrorDistinguishesConditionsFromFailures` table still passes, adapting names only.

- [ ] **Step 4: Verify + commit** — `go test ./...`, `-race`, `gofmt -s -l .`, `go vet ./...`.

Commit: `refactor(controller): keep the reading when a servo reports a condition`

---

## Task 2: Carry conditions across the servocmd boundary

**Files:** `internal/servocmd/servo_commands.go`, `internal/testfake/fake_servo_ops.go`, plus tests.

- [ ] **Step 1: Write the failing test**

`HandleServoCommand` with a fake reporting a condition must return the reading **and** a `condition` field; with no condition the field must be absent (not empty-string), so existing consumers are unaffected.

- [ ] **Step 2: Update `ServoOps` and the handler**

```go
case CmdServoPosition:
    percent, raw, condition, err := ops.ServoPositionPercent(ctx, id)
    if err != nil {
        return nil, err
    }
    res := map[string]any{"percent": percent, "raw": raw}
    if condition != 0 {
        res["condition"] = condition.Error()   // rendered string: survives gRPC
    }
    return res, nil
```

Same shape for `CmdServoLoad`.

**Why a string and not an int:** the flags cross gRPC as a protobuf `Struct`, where every number becomes a float64 and a bitfield would be lossy and opaque. `StatusError.Error()` renders human-readable text (`"overload"`) that is both loggable and matchable. Note this repo's `NumArg` exists precisely because of that float64 coercion — do not add a numeric flags field without accounting for it.

- [ ] **Step 3: Update `FakeServoOps`** with `Condition feetech.StatusError` and `LoadCondition`, honoured by both methods.

- [ ] **Step 4: Verify + commit** — the whole repo must build; `ServoOps` is a structural interface that `*ControllerHandle` satisfies implicitly, so a mismatch shows up as a compile error in `components/arm`, not here.

Commit: `feat(servocmd): carry servo condition flags in position and load responses`

---

## Task 3: Gripper reads conditions as data

**Files:** `components/gripper/gripper.go`, `components/gripper/gripper_test.go`.

This is the step that can silently regress grasp detection. Do it carefully.

- [ ] **Step 1: Write the failing test**

The fake arm currently signals a condition by **failing** the read (`readErr`). Add the new path: a `condition` field in a *successful* `servo_position` response must set `jawConditionSeen` and drive the retroactive latch exactly as the failure path does today.

Keep at least one test on the failure path too — a genuine transport failure must still be an error, and last-known-good must still serve.

- [ ] **Step 2: Read the field**

`positionPercent` returns the condition alongside the percent. `positionPercentCached` sets `jawConditionSeen` from that field rather than from `isOverload(err)`.

- [ ] **Step 3: Delete `isOverload`**

It is a string-matching stopgap. Its remote-arm justification disappears once flags travel as data — that was the whole point. Confirm no caller remains; `updateHoldingLatch`'s overload branches now consult the condition field.

- [ ] **Step 4: Mutation-prove the latch still works**

Non-negotiable, because this is where a silent regression lands. For each: break it, record the actual failure, revert.

1. Stop setting `jawConditionSeen` from the condition field → `TestGraspLatchUpgradesOnLateOverload` must fail.
2. Stop clearing the latch on an unstrained open → `TestGraspLatchClearsAfterStrainedOpen` must fail.

**Assert the mutation actually applied** before trusting a pass — a replacement that silently does not match reports a false pass, which happened during the latch work.

- [ ] **Step 5: Verify + commit** — full suite, `-race` on `components/gripper`.

Commit: `refactor(gripper): read servo conditions as data instead of parsing errors`

---

## Task 4: Docs

- [ ] `docs/gripper.md` — `get_position`/`get_load` may include a `condition` field; a condition does not mean the reading is invalid.
- [ ] `CLAUDE.md` — replace the stopgap gotcha with the real contract: conditions are data, `ConditionStatus` gates whether the payload is usable, and typed errors do not survive the DoCommand boundary but flags-as-fields do.

Commit: `docs: document servo conditions as data`

---

## Verification for every task

`go test -count=1 ./...`, `go test -race ./components/gripper/ ./internal/controller/`, `gofmt -s -l .` empty, `go vet ./...`.

## Hardware re-verification (after Task 3)

The latch's evidence path changes here, so the local suite is not sufficient. Re-run on the machine: `Grab` on the overloading object → expect `success: true` and `isHolding: true`; planner jaw command → **refused**; `Open` → latch clears; planner jaw command → accepted.

Leave the gripper **open and unclamped** — a clamped gripper previously blocked arm initialisation, and while v0.7.0's ping fix should prevent that, do not rely on it untested.
