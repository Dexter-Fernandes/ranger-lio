#!/usr/bin/env bash
# Convert a Hilti ROS 1 bag to rosbag2 (mcap), keeping only the LiDAR and IMU topics.
# Run inside the devcontainer (rosbags is installed in /opt/venv).
# Usage: scripts/convert_bag.sh <sequence>   e.g. exp14_basement_2
set -euo pipefail

DATA_ROOT="${DATA_ROOT:-/data/hilti22}"
CONFIG="$(dirname "$0")/../configs/hilti22.yaml"
seq="${1:?sequence name, e.g. exp14_basement_2}"
src="$DATA_ROOT/rosbags/$seq.bag"
dst="$DATA_ROOT/rosbag2/$seq"

lidar_topic=$(python3 -c "import yaml,sys; print(yaml.safe_load(open('$CONFIG'))['topics']['lidar'])")
imu_topic=$(python3 -c "import yaml,sys; print(yaml.safe_load(open('$CONFIG'))['topics']['imu'])")

[ -f "$src" ] || { echo "missing $src" >&2; exit 1; }
[ -e "$dst" ] && { echo "exists: $dst (delete to reconvert)"; exit 0; }
mkdir -p "$(dirname "$dst")"

echo ">> $src -> $dst  (topics: $lidar_topic $imu_topic)"
rosbags-convert --src "$src" --dst "$dst" --dst-storage mcap \
  --include-topic "$lidar_topic" --include-topic "$imu_topic"
ros2 bag info "$dst" || true
