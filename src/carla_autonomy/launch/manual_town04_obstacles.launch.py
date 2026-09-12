import json
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


OBJECTS_DEFINITION = {
    "objects": [
        {"type": "sensor.pseudo.traffic_lights", "id": "traffic_lights"},
        {"type": "sensor.pseudo.objects", "id": "objects"},
        {"type": "sensor.pseudo.actor_list", "id": "actor_list"},
        {"type": "sensor.pseudo.markers", "id": "markers"},
        {"type": "sensor.pseudo.opendrive_map", "id": "map"},
        {
            "type": "vehicle.tesla.model3",
            "id": "ego_vehicle",
            "spawn_point": {
                "x": 14.73,
                "y": 179.41,
                "z": 1.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 92.0,
            },
            "autopilot": False,
            "sensors": [
                {
                    "type": "sensor.camera.rgb",
                    "id": "rgb_view",
                    "spawn_point": {
                        "x": -4.5,
                        "y": 0.0,
                        "z": 2.8,
                        "roll": 0.0,
                        "pitch": 20.0,
                        "yaw": 0.0,
                    },
                    "image_size_x": 800,
                    "image_size_y": 600,
                    "fov": 90.0,
                    "attached_objects": [
                        {"type": "actor.pseudo.control", "id": "control"}
                    ],
                },
                {
                    "type": "sensor.other.gnss",
                    "id": "gnss",
                    "spawn_point": {"x": 1.0, "y": 0.0, "z": 2.0},
                    "noise_alt_stddev": 0.0,
                    "noise_lat_stddev": 0.0,
                    "noise_lon_stddev": 0.0,
                    "noise_alt_bias": 0.0,
                    "noise_lat_bias": 0.0,
                    "noise_lon_bias": 0.0,
                },
                {
                    "type": "sensor.other.collision",
                    "id": "collision",
                    "spawn_point": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
                {
                    "type": "sensor.other.lane_invasion",
                    "id": "lane_invasion",
                    "spawn_point": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
                {"type": "sensor.pseudo.tf", "id": "tf"},
                {"type": "sensor.pseudo.objects", "id": "objects"},
                {"type": "sensor.pseudo.odom", "id": "odometry"},
                {"type": "sensor.pseudo.speedometer", "id": "speedometer"},
                {"type": "actor.pseudo.control", "id": "control"},
            ],
        },
    ]
}


def _write_objects_definition():
    path = Path(tempfile.gettempdir()) / "carla_manual_town04_objects.json"
    path.write_text(json.dumps(OBJECTS_DEFINITION, indent=4) + "\n", encoding="utf-8")
    return str(path)


def _python_script(*parts):
    return PathJoinSubstitution([FindPackageShare("carla_autonomy"), "scripts", *parts])


def _obstacle_process(script_name):
    return ExecuteProcess(
        cmd=["python3", _python_script("obstacles", script_name)],
        name=f"obstacle_{script_name.removesuffix('.py').replace('.', '_')}",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_obstacles")),
    )


def generate_launch_description():
    objects_definition_file = _write_objects_definition()

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("carla_ros_bridge"), "carla_ros_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "timeout": LaunchConfiguration("timeout"),
            "town": LaunchConfiguration("town"),
            "passive": LaunchConfiguration("passive"),
            "synchronous_mode": LaunchConfiguration("synchronous_mode"),
            "fixed_delta_seconds": LaunchConfiguration("fixed_delta_seconds"),
            "register_all_sensors": "True",
            "ego_vehicle_role_name": "ego_vehicle",
        }.items(),
    )

    spawn_ego = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("carla_spawn_objects"),
                            "carla_spawn_objects.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "objects_definition_file": objects_definition_file,
                    "spawn_point_ego_vehicle": "None",
                    "spawn_sensors_only": "False",
                }.items(),
            )
        ],
    )

    manual_control = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("carla_manual_control"),
                            "carla_manual_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "role_name": "ego_vehicle",
                    "max_throttle": LaunchConfiguration("manual_max_throttle"),
                    "max_speed_kmh": LaunchConfiguration("manual_max_speed_kmh"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_manual_control")),
            )
        ],
    )

    obstacle_nodes = TimerAction(
        period=9.0,
        actions=[
            _obstacle_process("static_obs.py"),
            _obstacle_process("walker1.py"),
            _obstacle_process("walker2.py"),
            _obstacle_process("highway_obs.py"),
            _obstacle_process("cross_obs.py"),
            _obstacle_process("highway_hybrid_obstacle.py"),
            _obstacle_process("dynamic_slow_obs.py"),
        ],
    )

    camera_follow = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=["python3", _python_script("utils", "camera_follow.py")],
                name="camera_follow",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_camera_follow")),
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
            DeclareLaunchArgument("host", default_value="localhost"),
            DeclareLaunchArgument("port", default_value="2000"),
            DeclareLaunchArgument("timeout", default_value="10"),
            DeclareLaunchArgument("town", default_value="Town04"),
            DeclareLaunchArgument("passive", default_value="False"),
            DeclareLaunchArgument("synchronous_mode", default_value="True"),
            DeclareLaunchArgument("fixed_delta_seconds", default_value="0.05"),
            DeclareLaunchArgument("manual_max_throttle", default_value="0.30"),
            DeclareLaunchArgument("manual_max_speed_kmh", default_value="15.0"),
            DeclareLaunchArgument("start_manual_control", default_value="True"),
            DeclareLaunchArgument("start_obstacles", default_value="True"),
            DeclareLaunchArgument("start_camera_follow", default_value="True"),
            LogInfo(msg=f"Using CARLA objects definition: {objects_definition_file}"),
            bridge_launch,
            spawn_ego,
            manual_control,
            obstacle_nodes,
            camera_follow,
        ]
    )
