#!/bin/bash
# Walks through all blocks of the 9-condition test plan (camtestplan/9condition.md),
# launching camtestv6.py with the right --condition/--position each time so you
# don't have to retype the command per block. Depth is fixed at 30cm throughout
# (matching the calibrated rig) -- still the discrete/manual 3x-SPACE-per-block
# trial workflow, just single-depth instead of the plan's original 40cm/80cm.
#
# Tier 1 (condition 1 only): 5 positions x 30cm = 5 blocks.
# Tier 2 (conditions 2-9): center + plusx x 30cm = 16 blocks.
# 21 blocks total, 3x SPACE per block (inside camtestv6.py) = 63 trials.
#
# Turbidity NTU is prompted once per CONDITION (not per block), since the water
# only changes between conditions, not between positions within one.
#
# USAGE:
#   ./run_9cond.sh results/9cond.csv [extra camtestv6.py flags...]
#   ./run_9cond.sh results/9cond.csv --record --no-preview
#
# RESUMING: to skip everything already done and start at a given condition
# (e.g. you finished conditions 1-3 already), set START_CONDITION:
#   START_CONDITION=4 ./run_9cond.sh results/9cond.csv
# START_CONDITION=1 (default) runs Tier 1 + all of Tier 2. Any value >=2
# skips Tier 1 entirely (it's condition-1-only) and resumes Tier 2 from
# that condition number.

set -e

CSV="${1:-results/9cond.csv}"
shift || true
EXTRA_ARGS=("$@")

START_CONDITION="${START_CONDITION:-1}"

DEPTH_CM=30

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

prompt_turbidity() {
    local cond=$1 ntu
    read -r -p "Condition $cond -- set up the water, then enter turbidity NTU: " ntu
    echo "$ntu"
}

run_block() {
    local condition=$1 position=$2 turbidity=$3
    echo
    echo "=== Condition $condition | Position $position | Depth ${DEPTH_CM}cm | Turbidity ${turbidity}NTU ==="
    read -r -p "Reposition marker, then press Enter to launch (q inside camtestv6.py when this block's reps are done)... "
    python3 "$SCRIPT_DIR/camtestv6.py" \
        --condition "$condition" --position "$position" --distance-cm "$DEPTH_CM" \
        --turbidity-ntu "$turbidity" --csv "$CSV" "${EXTRA_ARGS[@]}"
}

echo "Logging all trials to $CSV (fixed depth ${DEPTH_CM}cm)"
if [ "$START_CONDITION" -gt 1 ]; then
    echo "Resuming from condition $START_CONDITION -- skipping Tier 1 and earlier Tier 2 conditions."
fi

# --- Tier 1: condition 1, full position sweep at 30cm ---
if [ "$START_CONDITION" -le 1 ]; then
    T1_NTU=$(prompt_turbidity 1)
    for pos in center plusx minusx plusy minusy; do
        run_block 1 "$pos" "$T1_NTU"
    done
fi

# --- Tier 2: conditions 2-9, center + plusx at 30cm ---
for cond in 2 3 4 5 6 7 8 9; do
    if [ "$cond" -lt "$START_CONDITION" ]; then
        continue
    fi
    NTU=$(prompt_turbidity "$cond")
    for pos in center plusx; do
        run_block "$cond" "$pos" "$NTU"
    done
done

echo
echo "All 21 blocks complete. Trials logged to $CSV"
