#!/usr/bin/env bash
# Build smoke test for av-prop firmware targets.
# Run from repo root: bash .claude/skills/run-av-prop/smoke.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
VENV="$REPO_ROOT/.venv/bin/activate"

if [[ ! -f "$VENV" ]]; then
  echo "ERROR: venv not found at $VENV" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$VENV"

PASS=0
FAIL=0
declare -A RESULTS

build_target() {
  local label="$1"
  local dir="$2"
  local env_flag="$3"

  echo ""
  echo "=== Building $label ==="
  if (cd "$REPO_ROOT/$dir" && pio run -e "$env_flag" 2>&1); then
    RESULTS[$label]="PASS"
    ((PASS++)) || true
  else
    RESULTS[$label]="FAIL"
    ((FAIL++)) || true
  fi
}

build_target "Upper_Control (ESP32-S3)" "Firmware/Upper_Control" "Upper_Control"
build_target "Lower_Control_test (STM32F103)" "Firmware/Lower_Control_test" "lower_test"

echo ""
echo "=============================="
echo "Build summary:"
for label in "${!RESULTS[@]}"; do
  echo "  ${RESULTS[$label]}  $label"
done
echo "=============================="

if [[ $FAIL -gt 0 ]]; then
  echo "RESULT: $FAIL target(s) failed"
  exit 1
else
  echo "RESULT: All $PASS target(s) passed"
fi
