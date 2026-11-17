#!/usr/bin/env bash
# Choose a Python with the policy dependencies; ROS launch itself keeps its system interpreter.
JAGUAR_RUNTIME_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
JAGUAR_RUNTIME_IMPORTS="torch"
if [[ "${1:-}" == sim2sim ]]; then
    JAGUAR_RUNTIME_IMPORTS="torch, mujoco, viser"
fi
JAGUAR_RUNTIME_SELECTED=""
if [[ -n "${JAGUAR_PYTHON:-}" ]]; then
    JAGUAR_RUNTIME_CANDIDATES=("$JAGUAR_PYTHON")
else
    JAGUAR_RUNTIME_CANDIDATES=(
        "$(command -v python3)"
        "$JAGUAR_RUNTIME_ROOT/.venv/bin/python3"
        "$HOME/IsaacLab/.venv/bin/python3"
        "$HOME/IsaacLab-upgrade/.venv/bin/python3"
    )
fi
for JAGUAR_RUNTIME_CANDIDATE in "${JAGUAR_RUNTIME_CANDIDATES[@]}"; do
    if [[ -x "$JAGUAR_RUNTIME_CANDIDATE" ]] && "$JAGUAR_RUNTIME_CANDIDATE" -c "import sys; assert sys.version_info[:2] == (3, 12); import $JAGUAR_RUNTIME_IMPORTS" >/dev/null 2>&1; then
        JAGUAR_RUNTIME_SELECTED="$JAGUAR_RUNTIME_CANDIDATE"
        break
    fi
done
if [[ -z "$JAGUAR_RUNTIME_SELECTED" ]]; then
    echo "Python3.12 dependencies missing ($JAGUAR_RUNTIME_IMPORTS). Set JAGUAR_PYTHON to the Python in your runtime environment." >&2
    return 1
fi
export PATH="$(dirname "$JAGUAR_RUNTIME_SELECTED"):$PATH"
echo "Policy Python: $JAGUAR_RUNTIME_SELECTED" >&2
unset JAGUAR_RUNTIME_ROOT JAGUAR_RUNTIME_IMPORTS JAGUAR_RUNTIME_SELECTED JAGUAR_RUNTIME_CANDIDATES JAGUAR_RUNTIME_CANDIDATE
