#!/bin/bash
# Run all SVO pipelines on all EuRoC sequences and evaluate with evo.
#
# Prerequisites:
#   - SVO built with examples: cmake --build build -j$(nproc)
#   - EuRoC sequences downloaded: bash examples/download_euroc.sh /tmp/euroc ALL
#   - Python evo tool installed:  pip install evo
#
# Usage:
#   ./run_all_benchmarks.sh [build_dir] [data_dir] [results_dir]
#
# Example:
#   ./run_all_benchmarks.sh ../build /tmp/euroc /tmp/euroc_results

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${1:-${SCRIPT_DIR}/../build}"
DATA_DIR="${2:-/tmp/euroc}"
RESULTS_DIR="${3:-/tmp/euroc_results}"

SEQUENCES=(
  MH_01_easy
  MH_02_easy
  MH_03_medium
  MH_04_difficult
  MH_05_difficult
  V1_01_easy
  V1_02_medium
  V1_03_difficult
  V2_01_easy
  V2_02_medium
  V2_03_difficult
)

PIPELINES=(euroc_mono_vo euroc_mono_vio euroc_stereo_vio)
PIPE_LABELS=("Mono VO" "Mono VIO" "Stereo VIO")

# Timeout per pipeline run (seconds). Stereo VIO is slower.
TIMEOUT=300

mkdir -p "$RESULTS_DIR"

# --- Check prerequisites ---
for PIPE in "${PIPELINES[@]}"; do
  if [ ! -x "${BUILD_DIR}/examples/${PIPE}" ]; then
    echo "ERROR: ${BUILD_DIR}/examples/${PIPE} not found. Build with -DSVO_BUILD_EXAMPLES=ON first."
    exit 1
  fi
done

if ! command -v evo_ape &>/dev/null; then
  echo "WARNING: evo_ape not found. Install with: pip install evo"
  echo "         Trajectories will be generated but not evaluated."
  HAS_EVO=false
else
  HAS_EVO=true
fi

MISSING_SEQS=()
for SEQ in "${SEQUENCES[@]}"; do
  if [ ! -d "${DATA_DIR}/${SEQ}/mav0/cam0" ]; then
    MISSING_SEQS+=("$SEQ")
  fi
done
if [ ${#MISSING_SEQS[@]} -gt 0 ]; then
  echo "ERROR: Missing sequences: ${MISSING_SEQS[*]}"
  echo "       Download with: bash examples/download_euroc.sh ${DATA_DIR} ALL"
  exit 1
fi

# --- Convert ground truth to TUM format ---
convert_gt() {
  local gt_csv="$1" gt_tum="$2"
  python3 -c "
import csv
with open('${gt_csv}') as f:
    reader = csv.reader(f)
    next(reader)
    with open('${gt_tum}', 'w') as out:
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            ts = float(row[0]) * 1e-9
            px, py, pz = row[1], row[2], row[3]
            qw, qx, qy, qz = row[4], row[5], row[6], row[7]
            out.write(f'{ts:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n')
"
}

echo "=== Step 1: Converting ground truth to TUM format ==="
for SEQ in "${SEQUENCES[@]}"; do
  GT_CSV="${DATA_DIR}/${SEQ}/mav0/state_groundtruth_estimate0/data.csv"
  GT_TUM="${DATA_DIR}/${SEQ}/mav0/groundtruth_tum.txt"
  if [ -f "$GT_TUM" ]; then
    continue
  fi
  echo "  Converting ${SEQ}..."
  convert_gt "$GT_CSV" "$GT_TUM"
done
echo "  Done."
echo ""

# --- Run pipelines ---
echo "=== Step 2: Running pipelines ==="
for SEQ in "${SEQUENCES[@]}"; do
  DATASET="${DATA_DIR}/${SEQ}/mav0"
  for PIPE in "${PIPELINES[@]}"; do
    OUTFILE="${RESULTS_DIR}/${SEQ}_${PIPE}.txt"
    if [ -f "$OUTFILE" ] && [ -s "$OUTFILE" ]; then
      echo "  [skip] ${SEQ} / ${PIPE} (already exists)"
      continue
    fi
    echo -n "  [run]  ${SEQ} / ${PIPE}... "
    if timeout "$TIMEOUT" "${BUILD_DIR}/examples/${PIPE}" \
        --dataset_dir "$DATASET" \
        --output "$OUTFILE" \
        2>&1 | grep -oP "Frames tracked:\s+\d+ \([0-9.]+%\)" | tail -1; then
      :
    else
      echo "FAILED or TIMEOUT"
      # Remove partial output
      rm -f "$OUTFILE"
    fi
  done
done
echo ""

# --- Evaluate ---
eval_one() {
  local gt="$1" est="$2"
  if [ ! -f "$est" ] || [ ! -s "$est" ]; then
    echo "--"
    return
  fi
  local nlines
  nlines=$(grep -cv "^#" "$est" 2>/dev/null || echo 0)
  if [ "$nlines" -lt 10 ]; then
    echo "--"
    return
  fi
  if [ "$HAS_EVO" = false ]; then
    echo "N/A"
    return
  fi
  local rmse
  rmse=$(evo_ape tum "$gt" "$est" --align --correct_scale 2>&1 | grep "rmse" | awk '{print $2}')
  if [ -z "$rmse" ]; then
    echo "ERR"
  else
    # Truncate to 3 decimal places
    printf "%.3f" "$rmse"
  fi
}

echo "=== Step 3: Evaluation (ATE RMSE in meters, Sim(3) alignment) ==="
echo ""
printf "| %-18s | %-10s | %-10s | %-10s |\n" "Sequence" "Mono VO" "Mono VIO" "Stereo VIO"
printf "|%-20s|%-12s|%-12s|%-12s|\n" "--------------------" "------------" "------------" "------------"

for SEQ in "${SEQUENCES[@]}"; do
  GT="${DATA_DIR}/${SEQ}/mav0/groundtruth_tum.txt"
  VO=$(eval_one  "$GT" "${RESULTS_DIR}/${SEQ}_euroc_mono_vo.txt")
  VIO=$(eval_one "$GT" "${RESULTS_DIR}/${SEQ}_euroc_mono_vio.txt")
  SVIO=$(eval_one "$GT" "${RESULTS_DIR}/${SEQ}_euroc_stereo_vio.txt")
  printf "| %-18s | %-10s | %-10s | %-10s |\n" "$SEQ" "$VO" "$VIO" "$SVIO"
done

echo ""
echo "Results saved in: ${RESULTS_DIR}/"
echo "To re-evaluate a single trajectory:"
echo "  evo_ape tum <groundtruth_tum.txt> <trajectory.txt> --align --correct_scale"
