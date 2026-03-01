#!/bin/bash
# Download EuRoC MAV dataset sequences.
#
# Usage:
#   ./download_euroc.sh [output_dir] [sequence...]
#
# Examples:
#   ./download_euroc.sh /tmp/euroc                          # Downloads V2_01_easy
#   ./download_euroc.sh /tmp/euroc MH_01_easy V1_01_easy    # Downloads specific sequences
#   ./download_euroc.sh /tmp/euroc ALL                      # Downloads all 11 sequences
#
# Available sequences:
#   MH_01_easy  MH_02_easy  MH_03_medium  MH_04_difficult  MH_05_difficult
#   V1_01_easy  V1_02_medium  V1_03_difficult
#   V2_01_easy  V2_02_medium  V2_03_difficult
#
# Data source: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

set -euo pipefail

BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"

ALL_SEQUENCES=(
  MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult
  V1_01_easy V1_02_medium V1_03_difficult
  V2_01_easy V2_02_medium V2_03_difficult
)

OUTPUT_DIR="${1:-/tmp/euroc}"
shift 2>/dev/null || true

# Parse sequence arguments
if [ $# -eq 0 ]; then
  SEQUENCES=(V2_01_easy)
elif [ "$1" = "ALL" ] || [ "$1" = "all" ]; then
  SEQUENCES=("${ALL_SEQUENCES[@]}")
else
  SEQUENCES=("$@")
fi

mkdir -p "$OUTPUT_DIR"

# Validate sequence names
for SEQ in "${SEQUENCES[@]}"; do
  VALID=false
  for ALL_SEQ in "${ALL_SEQUENCES[@]}"; do
    if [ "$SEQ" = "$ALL_SEQ" ]; then
      VALID=true
      break
    fi
  done
  if [ "$VALID" = false ]; then
    echo "ERROR: Unknown sequence '${SEQ}'"
    echo "Available: ${ALL_SEQUENCES[*]}"
    exit 1
  fi
done

TOTAL=${#SEQUENCES[@]}
CURRENT=0

for SEQ in "${SEQUENCES[@]}"; do
  CURRENT=$((CURRENT + 1))

  if [ -d "${OUTPUT_DIR}/${SEQ}/mav0/cam0" ]; then
    echo "[${CURRENT}/${TOTAL}] ${SEQ} already exists, skipping."
    continue
  fi

  ZIP_URL="${BASE_URL}/${SEQ}/${SEQ}.zip"
  ZIP_FILE="${OUTPUT_DIR}/${SEQ}.zip"

  echo "[${CURRENT}/${TOTAL}] Downloading ${SEQ}..."
  if ! curl -L --progress-bar --retry 3 -o "$ZIP_FILE" "$ZIP_URL"; then
    echo "  ERROR: Download failed for ${SEQ}. Skipping."
    rm -f "$ZIP_FILE"
    continue
  fi

  echo "  Extracting..."
  if ! unzip -q -o "$ZIP_FILE" -d "$OUTPUT_DIR"; then
    echo "  ERROR: Extraction failed for ${SEQ}. The zip may be corrupted."
    rm -f "$ZIP_FILE"
    continue
  fi
  rm -f "$ZIP_FILE"

  # Verify extraction
  if [ -d "${OUTPUT_DIR}/${SEQ}/mav0/cam0" ]; then
    N_IMGS=$(ls "${OUTPUT_DIR}/${SEQ}/mav0/cam0/data/" 2>/dev/null | wc -l)
    echo "  Done: ${OUTPUT_DIR}/${SEQ} (${N_IMGS} images)"
  else
    echo "  WARNING: Extracted but mav0/cam0 not found — directory structure may differ."
  fi
done

echo ""
echo "All done. Dataset directory: ${OUTPUT_DIR}"
echo ""
echo "Run examples:"
echo "  ./build/examples/euroc_mono_vo      --dataset_dir ${OUTPUT_DIR}/${SEQUENCES[0]}/mav0"
echo "  ./build/examples/euroc_mono_vio     --dataset_dir ${OUTPUT_DIR}/${SEQUENCES[0]}/mav0"
echo "  ./build/examples/euroc_stereo_vio   --dataset_dir ${OUTPUT_DIR}/${SEQUENCES[0]}/mav0"
echo ""
echo "Run full benchmark:"
echo "  bash examples/run_all_benchmarks.sh build ${OUTPUT_DIR} /tmp/euroc_results"
