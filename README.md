# Learning- and Rule-based Hybrid Path Planning

This repository is provided for portfolio and research demonstration purposes.
No separate open-source reuse license is granted; third-party dependencies keep
their own terms.

A cleaned portfolio release of a two-person, co-developed CARLA autonomous
driving project. The system separates learned normal-path steering from
explicit planning and safety responses for obstacles and traffic situations.
This repository was organized by Taeyun Kim from the final local development
snapshot; the original development repository is
[MyungJin04/hybrid-path-planning](https://github.com/MyungJin04/hybrid-path-planning).

## Overview

The project combines Imitation Learning (IL) for smooth normal driving with
rule-based control where the situation has an explicit operational response.
Rather than asking one learned policy to cover every condition, the master
controller arbitrates between a Behavioral Cloning steering policy, a Lattice
Planner, stop behavior, traffic rules, and a Pure Pursuit fallback.

## Problem

Keyboard driving was initially considered for collecting training data, but
the steering signal was too noisy and inconsistent for the final learning
set. A Pure Pursuit controller following the CARLA reference route was used as
the expert policy instead. Obstacle-avoidance data was not available in enough
variety to train it reliably into IL, so the final system assigns normal route
following to IL and static-obstacle avoidance to the Lattice Planner.

## System Architecture

```text
Camera / route / perception topics
               |
               v
  Situation recognition and obstacle tracking
               |
               v
      Master Control / Arbitration
        |- Normal driving        -> Behavioral Cloning steering
        |- Static obstacle       -> Lattice Planner
        |- Pedestrian / cut-in   -> Stop
        |- Traffic / stop line   -> Rule-based control
        `- Missing IL command    -> Pure Pursuit fallback
               |
               v
       CARLA vehicle-control command
```

The Behavioral Cloning controller publishes steering during normal driving.
Speed is handled by its rule-based speed-control path. The master controller
selects or overrides the command when traffic or obstacle conditions require
an explicit response.

## Imitation Learning

- Pure Pursuit route following provided the expert steering used for the final
  data-collection pipeline.
- The Behavioral Cloning runtime consumes a front-camera image, a route
  command and future route points, then predicts steering for normal path
  following.
- Keyboard-driving data was explored before the final pipeline, but was not
  used for final training because of steering inconsistency.
- Learned weights and training data are intentionally not included in this
  release. See [`src/carla_autonomy/models/README.md`](src/carla_autonomy/models/README.md).

## Rule-based Planning

- `master_control_node` performs hybrid arbitration and publishes the final
  CARLA control command.
- `lattice_planner.hpp` generates the offset path used for static-obstacle
  avoidance and return behavior.
- `traffic_manager.hpp`, the traffic-light detector and stop-line detector
  provide rule-based traffic intervention.
- Perception and obstacle scripts publish the inputs used by the master
  controller.
- Pure Pursuit remains available when an IL command is not received within the
  configured timeout.

## Team and Contribution

This was a **two-person team project**. System design and implementation were
co-developed. This repository is a cleaned portfolio version organized by
Taeyun Kim; it does not assign file-level sole authorship where the original
Git history does not establish it. The original development repository remains
the source of record for the team project.

## Result

The project implemented CARLA closed-loop hybrid driving with control transfer
between IL normal driving, lattice obstacle handling, traffic/stop behavior,
and a Pure Pursuit fallback.

## Limitation

When the same hybrid structure was applied to another CARLA map late in the
project, choosing an avoidance direction from obstacle position alone did not
account for lane boundaries, curbs, or the drivable area. In some conditions,
the generated path pointed toward a curb or outside the drivable area. Some
obstacle-classification conditions also conflicted, preventing the intended
static-obstacle avoidance from running.

These issues were found during final validation and were not completely fixed
within the project period. Future work should generate and compare left/right
avoidance candidates while evaluating their drivable area and surrounding
vehicle safety.

## Repository Structure

```text
.
├── README.md
├── PUBLIC_RELEASE_NOTES.md
└── src/
    └── carla_autonomy/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── route.example.csv
        ├── launch/
        │   ├── rule_based_carla.launch.py
        │   └── manual_town04_obstacles.launch.py
        ├── models/
        │   └── README.md
        ├── scripts/
        │   ├── control/
        │   ├── obstacles/
        │   ├── perception/
        │   └── utils/
        └── src/control/
            ├── master_control_node.cpp
            ├── lattice_planner.hpp
            ├── pure_pursuit.hpp
            └── traffic_manager.hpp
```

## Environment

The source is a ROS 2 `ament_cmake` package and requires a CARLA server plus
the separately installed `carla_ros_bridge`, `carla_spawn_objects`, and
`carla_msgs` packages. The Python nodes additionally use CARLA's Python API,
PyTorch, OpenCV, Ultralytics, NumPy, Pillow, and `cv_bridge`.

The original source does not document exact ROS 2 distribution, CARLA, CUDA,
or model versions. This release therefore does not claim a specific version;
use compatible versions for the CARLA ROS bridge being installed.

## How to Run

This repository does not include CARLA, ROS 2, learned weights, a competition
route, datasets, or rosbag recordings. Install those dependencies separately
and provide paths to artifacts that are cleared for your own use.

```bash
# Put this package in a ROS 2 workspace.
mkdir -p ~/hybrid_ws/src
cp -a /path/to/hybrid-path-planning/src/carla_autonomy ~/hybrid_ws/src/
cd ~/hybrid_ws
source /opt/ros/<your-distro>/setup.bash
colcon build --packages-select carla_autonomy
source install/setup.bash

# Start a compatible CARLA server, then provide local artifacts explicitly.
ros2 launch carla_autonomy rule_based_carla.launch.py \\
  route_file:=/absolute/path/to/route.csv \\
  bc_model_path:=/absolute/path/to/bc_checkpoint.pth \\
  traffic_light_model_path:=/absolute/path/to/traffic_light_model.pt \\
  yolo_model_path:=/absolute/path/to/object_detector.pt
```

`route_file` must use the `x,y,z,yaw` CSV header shown in
[`route.example.csv`](src/carla_autonomy/config/route.example.csv). The launch
file exposes additional CARLA, perception, obstacle, speed, and IL parameters;
inspect `rule_based_carla.launch.py` before applying it to a different map.

## Project History and Credits

- Original team repository:
  [MyungJin04/hybrid-path-planning](https://github.com/MyungJin04/hybrid-path-planning)
- Development model: two-person co-development
- Portfolio release organization: Taeyun Kim

Publication checks were completed before this public release. The
[`PUBLIC_RELEASE_NOTES.md`](PUBLIC_RELEASE_NOTES.md) file records the release
scope and provenance.
