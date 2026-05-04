#!/usr/bin/env bash
# End-to-end PerceptionBOT demo: launches Gazebo + MoveIt + perception +
# manipulation, waits for everything to come up, spawns objects, fires the
# SortObjects action, records a ros2 bag for evidence.
#
# Usage:
#   bash scripts/run_demo.sh [--headless]
#
# --headless skips RViz (and ideally the Gazebo client, but Gazebo Harmonic's
# launcher is GUI-by-default; pass --headless-rendering via gz_args if needed).
set -euo pipefail

HEADLESS=0
for arg in "$@"; do
  case "${arg}" in
    --headless) HEADLESS=1 ;;
    *) echo "unknown arg: ${arg}"; exit 2 ;;
  esac
done

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT}"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS2 Jazzy not found at /opt/ros/jazzy"; exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

if [[ ! -f install/setup.bash ]]; then
  echo "Workspace not built — run: colcon build --symlink-install"; exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

if [[ -d /opt/onnxruntime ]]; then
  export LD_LIBRARY_PATH="/opt/onnxruntime/lib:${LD_LIBRARY_PATH:-}"
fi

LOG="${ROOT}/.demo.log"
BAG="${ROOT}/.demo_bag"
rm -rf "${BAG}"
mkdir -p "${BAG}"

echo "[demo] starting demo.launch.py (logs → ${LOG})"
RVIZ_ARG="use_rviz:=true"
if [[ ${HEADLESS} -eq 1 ]]; then
  RVIZ_ARG="use_rviz:=false"
fi
ros2 launch perception_bot_bringup demo.launch.py "${RVIZ_ARG}" > "${LOG}" 2>&1 &
LAUNCH_PID=$!
trap 'echo "[demo] cleaning up"; kill -INT ${LAUNCH_PID} 2>/dev/null || true; sleep 2; pkill -f gz || true' EXIT

echo "[demo] waiting for /joint_states to flow..."
for _ in $(seq 1 60); do
  if ros2 topic echo /joint_states sensor_msgs/msg/JointState --once \
      --timeout 1 >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

echo "[demo] waiting for move_group action server..."
for _ in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q "/move_action"; then
    break
  fi
  sleep 1
done

echo "[demo] waiting for /sort_objects action server..."
for _ in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q "/sort_objects"; then
    break
  fi
  sleep 1
done

echo "[demo] spawning objects..."
python3 "${ROOT}/scripts/spawn_objects.py" --count 3 --seed 7 || true

# Give Gazebo physics + perception a moment to settle and detect.
sleep 4

echo "[demo] starting rosbag record..."
ros2 bag record -o "${BAG}/run" \
  /perception/detections \
  /perception/objects_world \
  /tf /tf_static /joint_states \
  /camera/image /camera/depth_image \
  > "${BAG}/record.log" 2>&1 &
BAG_PID=$!

echo "[demo] sending SortObjects goal..."
ros2 action send_goal /sort_objects perception_bot_msgs/action/SortObjects \
  '{use_estimated_depth: false, max_objects: 0}' || true

echo "[demo] stopping rosbag..."
kill -INT "${BAG_PID}" 2>/dev/null || true
sleep 2

echo "[demo] done — bag stored at ${BAG}/run, logs at ${LOG}"
