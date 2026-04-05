# PerceptionBOT — CLAUDE.md

## Project
ROS2 C++ vision-guided multi-object sorting. Franka FR3 in Gazebo Harmonic.
YOLOv11n + Depth Anything V2 ViT-S → 6DoF pose → MoveIt 2 Task Constructor pick-and-place.

## Stack
- ROS2 Jazzy + Gazebo Harmonic on Ubuntu 24.04
- MoveIt 2.12.4 + Task Constructor (binary install)
- ONNX Runtime C++ with CUDA EP (pre-built tarball at /opt/onnxruntime)
- OpenCV 4 (system)
- C++17 throughout

## Workspace
```
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install
colcon test
```

## Package Map
- `perception_bot_msgs`       — custom msg/srv/action interfaces
- `perception_bot_perception` — YOLOv11 + Depth Anything + pose estimation nodes
- `perception_bot_manipulation`— MoveIt MTC pick-and-place + sort planner
- `perception_bot_description` — URDF, SDF world, meshes
- `perception_bot_bringup`    — launch files, RViz config

## Conventions
- All ROS2 nodes in C++ (rclcpp). Python only for launch files and utility scripts.
- Topic names: snake_case. Package names: snake_case with `perception_bot_` prefix.
- ONNX models NOT committed. Download via: `python3 scripts/download_models.py`
- No hardcoded paths. All file paths via ROS2 parameters or `ament_index`.

## Build from scratch
```bash
bash scripts/install_deps.sh        # installs ROS2, Gazebo, MoveIt
python3 scripts/download_models.py  # downloads ONNX models
colcon build --symlink-install
```

## Implementation Phases
- [x] Phase 0: Foundation — ROS2 + Gazebo + workspace
- [ ] Phase 1: Gazebo world + Franka FR3 + camera
- [ ] Phase 2: YOLOv11 detection node (C++ ONNX)
- [ ] Phase 3: Depth Anything V2 depth node (C++ ONNX)
- [ ] Phase 4: Pose estimation node (depth → 3D)
- [ ] Phase 5: MoveIt 2 + pick-and-place + sort planner
- [ ] Phase 6: Integration + demo recording
- [ ] Phase 7: Docker + CI + README polish
