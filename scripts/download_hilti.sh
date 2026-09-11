#!/usr/bin/env bash
# Download the Hilti-Oxford 2022 files ranger-lio uses into the host data directory.
# Usage: scripts/download_hilti.sh [exp14|exp18|exp16|all]   (default: exp14)
# Uses curl with resume (-C -) against Hugging Face; safe to re-run. Bags are 6 to 16 GB each.
set -euo pipefail

DATA_ROOT="${DATA_ROOT:-/home/dexter/data/hilti22}"
BASE="https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main"
which="${1:-exp14}"

fetch() {  # fetch <relative path>
  local rel="$1" dst="$DATA_ROOT/$1"
  mkdir -p "$(dirname "$dst")"
  echo ">> $rel"
  curl -L --fail --retry 5 --retry-delay 10 -C - -o "$dst" "$BASE/$rel"
}

# Ground truth and calibration are small; always fetch them.
for f in exp14_basement_2_imu.txt exp14_basement_2_imu_3dof.txt \
         exp16_attic_to_upper_gallery_2_imu.txt exp16_attic_to_upper_gallery_2_imu_3dof.txt \
         exp18_corridor_lower_gallery_2_imu.txt exp18_corridor_lower_gallery_2_imu_3dof.txt; do
  fetch "ground_truth/$f"
done
fetch "calibration/calibration_files/lidar_calibration.yaml"
fetch "calibration/calibration_files/calib_3_cam0-1-camchain-imucam.yaml"

case "$which" in
  exp14) fetch "rosbags/exp14_basement_2.bag" ;;
  exp18) fetch "rosbags/exp18_corridor_lower_gallery_2.bag" ;;
  exp16) fetch "rosbags/exp16_attic_to_upper_gallery_2.bag" ;;
  all)   fetch "rosbags/exp14_basement_2.bag"
         fetch "rosbags/exp18_corridor_lower_gallery_2.bag"
         fetch "rosbags/exp16_attic_to_upper_gallery_2.bag" ;;
  *) echo "unknown selection: $which" >&2; exit 2 ;;
esac
echo "done: $DATA_ROOT"
