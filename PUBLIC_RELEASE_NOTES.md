# Public Release Notes

## Purpose and Scope

This directory is the clean public release for the portfolio repository. The
original working tree was not modified during this extraction.

## Source Origin

- Original development repository:
  `https://github.com/MyungJin04/hybrid-path-planning`
- Original local snapshot: private local development workspace (path omitted)
- Committed baseline at extraction: `e64a954` on `main`, matching
  `origin/main`
- Source used here: the local `main` checkout **including its uncommitted
  overlay**, because that overlay contains the final hybrid arbitration and
  runtime additions described below.

The source snapshot is a preservation copy, not a claim that all retained
files were authored by one person.

## Difference Between Committed Main and the Local Overlay

At extraction, the local overlay contained nine modified tracked files and
three untracked files. The relevant additions or changes are:

- `master_control_node.cpp`: BC IL command input, hybrid-mode publication,
  configurable route/control inputs, and IL timeout fallback behavior.
- `traffic_manager.hpp`: final traffic-control adjustments.
- `stop_line_detector.py`, `traffic_light_detector.py`, and
  `yolo_perception.py`: detector tuning and configurable model loading.
- `cross_obs.py` and `dynamic_slow_obs.py`: obstacle-scenario adjustments.
- `rule_based_carla.launch.py` and `manual_town04_obstacles.launch.py`:
  launch orchestration for the final scenario.
- `bc_future_il_controller.py`: Behavioral Cloning inference using camera,
  command, and future-route inputs.
- `CMakeLists.txt` and `package.xml`: installation and launch dependencies for
  the added runtime files.

## Included Files

The public release retains only the `carla_autonomy` package portions needed to
understand the final hybrid stack:

- C++ master control, Lattice Planner, Pure Pursuit, and traffic-manager code
- BC inference controller
- traffic-light, stop-line, LiDAR, and YOLO perception scripts
- CARLA obstacle-scenario scripts and small visualization/control utilities
- the two final launch files
- package metadata, build instructions, and a route CSV format template

## Excluded Files

The following were deliberately not copied:

- ROS `build/`, `install/`, `log/`, Python cache, and editor workspace files
- CARLA installation, ROS installation, `carla_ros_bridge`, and other external
  packages
- learned weights: `models/best.pt`, `scripts/perception/yolov8n.pt`, and the
  external BC checkpoint
- the original private local route file
- datasets, rosbag recordings, model outputs, unrelated experiments, and
  vendor code

Model paths are launch arguments rather than repository contents. The public
release must continue to exclude weights and data unless their redistribution
rights are confirmed separately.

## Public-Safety Changes

The following changes were made **only in this public release**:

| File | Original local setting | Public-release change |
|---|---|---|
| `scripts/control/bc_future_il_controller.py` | Default BC model and route paths under a private local workspace | Defaults are empty and both parameters are required at runtime. |
| `src/control/master_control_node.cpp` | Default route under a private local path | Default is empty; a clear error is logged unless `route_file` is supplied. |
| `launch/rule_based_carla.launch.py` | Default route and BC-model paths under a private local workspace | Both defaults are empty; traffic-light and object-model paths are exposed as launch arguments. |

No token, API key, password, `.env` file, private key, phone number, student
identifier, or non-loopback IP address was found in the copied source scan.
`localhost`, `127.0.0.1`, and CARLA's default port remain because they are
generic local simulator settings. Hardware-free CARLA coordinates remain as
scenario configuration, not personal location data.

## Ownership and Provenance

This is a **two-person co-developed project**. The upstream repository belongs
to `MyungJin04`; Git history alone does not establish sole ownership of every
retained file, especially for the uncommitted overlay. The README therefore
uses `co-developed` language and identifies the original repository.

Team consent to publish the retained co-developed source is confirmed. The
README keeps the project co-developed and preserves the original-repository
credit. Model/data/media rights remain separate: those artifacts are excluded
from this tree and require a separate review before any later addition.

`package.xml` uses `UNLICENSED` to record that this portfolio repository grants
no separate open-source reuse license. Third-party dependency terms remain in
force.

## Release Follow-up

1. Build in a compatible ROS 2 + CARLA ROS bridge environment and record the
   tested dependency versions.
2. Test the documented launch command with publication-cleared local artifacts
   and a non-sensitive route file.

## Validation Performed

- Python syntax validation for all 17 copied `.py` files
- `package.xml` XML validation, package discovery, and local README-link checks
- static scan for absolute paths, credentials, artifact extensions, IP
  addresses, and machine-specific files
- package/launch file reference review
- an isolated ROS 2 Humble `colcon build` configure attempt

The isolated `colcon build` stopped before C++ compilation because
`carla_msgs` was not installed in the current ROS environment. `carla_msgs`,
`carla_ros_bridge`, and the CARLA runtime are external dependencies that this
release deliberately does not copy. A full build and simulation run remain
follow-up validation tasks in an environment with the compatible bridge
installed.
