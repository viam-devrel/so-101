#!/usr/bin/env bash
# Drive an articulated simulated gripper's jaw over GoToInputs, by percentage.
#
# The gripper API passes raw float64 RADIANS with no degree conversion (unlike the arm API,
# which round-trips through Frame.ProtobufFromInput), so the percentage is converted here using
# internal/geometry's GripperJointMin/GripperJointMax.
#
# Requires the gripper to be configured with "articulated_jaw": true -- without it the model is
# 0-DoF and GoToInputs returns "unsupported operation".
#
# Usage:
#   tools/jaw.sh 50                 # open the jaw halfway
#   tools/jaw.sh 0                  # closed
#   tools/jaw.sh 100                # fully open
#   tools/jaw.sh status             # current position, angle, DoF
#   tools/jaw.sh trajectory         # the last GoToInputs batches the gripper recorded
#
#   PART=<part-id> COMPONENT=<name> tools/jaw.sh 50
set -euo pipefail

PART="${PART:-07b7fe1d-3cd1-49b4-bb10-c8a41df37c48}"
COMPONENT="${COMPONENT:-gripper-sim}"

# internal/geometry/gripper.go
JAW_MIN=-0.174533
JAW_MAX=1.74533

run() { viam machines part run --part "$PART" -c "$COMPONENT" --method "$1" --data "$2"; }
status() { run DoCommand '{"command":{"command":"get_position"}}'; }

case "${1:-status}" in
  status)     status ;;
  trajectory) run DoCommand '{"command":{"command":"get_jaw_trajectory"}}' ;;
  *)
    pct="$1"
    if ! awk -v p="$pct" 'BEGIN{exit !(p==p+0 && p>=0 && p<=100)}'; then
      echo "usage: $0 <0-100|status|trajectory>" >&2
      exit 1
    fi
    rad=$(awk -v p="$pct" -v lo="$JAW_MIN" -v hi="$JAW_MAX" 'BEGIN{printf "%.7f", lo + (p/100)*(hi-lo)}')
    echo "--- before ---"; status
    echo "--- GoToInputs [$rad] (${pct}%) ---"
    run viam.component.gripper.v1.GripperService.GoToInputs "{\"values\":[$rad]}" >/dev/null
    echo "--- after ---"; status
    ;;
esac
