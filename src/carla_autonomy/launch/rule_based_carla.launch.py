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
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


OBJECTS_DEFINITION = {
    "objects": [
        {
            "type": "sensor.pseudo.traffic_lights",
            "id": "traffic_lights",
        },
        {
            "type": "sensor.pseudo.objects",
            "id": "objects",
        },
        {
            "type": "sensor.pseudo.actor_list",
            "id": "actor_list",
        },
        {
            "type": "sensor.pseudo.markers",
            "id": "markers",
        },
        {
            "type": "sensor.pseudo.opendrive_map",
            "id": "map",
        },
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
                    "id": "rgb_traffic_light",
                    "spawn_point": {
                        "x": 1.5,
                        "y": 0.0,
                        "z": 2.4,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "image_size_x": 640,
                    "image_size_y": 480,
                    "fov": 60.0,
                },
                {
                    "type": "sensor.camera.rgb",
                    "id": "rgb_stop_line",
                    "spawn_point": {
                        "x": 2.0,
                        "y": 0.0,
                        "z": 1.2,
                        "roll": 0.0,
                        "pitch": 30.0,
                        "yaw": 0.0,
                    },
                    "image_size_x": 480,
                    "image_size_y": 320,
                    "fov": 90.0,
                },
                {
                    "type": "sensor.camera.rgb",
                    "id": "rgb_bc",
                    "spawn_point": {
                        "x": 1.5,
                        "y": 0.0,
                        "z": 2.4,
                        "roll": 0.0,
                        "pitch": -5.0,
                        "yaw": 0.0,
                    },
                    "image_size_x": 640,
                    "image_size_y": 360,
                    "fov": 90.0,
                },
                {
                    "type": "sensor.lidar.ray_cast",
                    "id": "lidar",
                    "spawn_point": {
                        "x": 1.5,
                        "y": 0.0,
                        "z": 2.4,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "range": 50,
                    "channels": 32,
                    "points_per_second": 320000,
                    "upper_fov": 2.0,
                    "lower_fov": -26.8,
                    "rotation_frequency": 20,
                    "noise_stddev": 0.0,
                },
                {
                    "type": "sensor.pseudo.tf",
                    "id": "tf",
                },
                {
                    "type": "sensor.pseudo.objects",
                    "id": "objects",
                },
                {
                    "type": "sensor.pseudo.odom",
                    "id": "odometry",
                },
                {
                    "type": "sensor.pseudo.speedometer",
                    "id": "speedometer",
                },
                {
                    "type": "actor.pseudo.control",
                    "id": "control",
                },
            ],
        },
    ]
}


def _write_objects_definition():
    path = Path(tempfile.gettempdir()) / "carla_rule_based_objects.json"
    path.write_text(json.dumps(OBJECTS_DEFINITION, indent=4) + "\n", encoding="utf-8")
    return str(path)


def _python_script(*parts):
    return PathJoinSubstitution([FindPackageShare("carla_autonomy"), "scripts", *parts])


def _obstacle_process(script_name):
    return ExecuteProcess(
        cmd=[
            "python3",
            _python_script("obstacles", script_name),
        ],
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
        condition=IfCondition(LaunchConfiguration("start_bridge")),
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
                    "spawn_sensors_only": LaunchConfiguration("spawn_sensors_only"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_spawn")),
            )
        ],
    )

    perception_nodes = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    _python_script("perception", "traffic_light_detector.py"),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                    "-p",
                    ["model_path:=", LaunchConfiguration("traffic_light_model_path")],
                ],
                name="traffic_light_detector",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_perception")),
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    _python_script("perception", "stop_line_detector.py"),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                    "-p",
                    ["model_path:=", LaunchConfiguration("yolo_model_path")],
                ],
                name="stop_line_detector",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_perception")),
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    _python_script("perception", "yolo_perception.py"),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                name="yolo_perception",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_perception")),
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    _python_script("perception", "lidar_perception.py"),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                name="lidar_perception",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_perception")),
            ),
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

    bc_il_node = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    _python_script("control", "bc_future_il_controller.py"),
                    "--ros-args",
                    "-p",
                    ["host:=", LaunchConfiguration("host")],
                    "-p",
                    ["port:=", LaunchConfiguration("port")],
                    "-p",
                    "role_name:=ego_vehicle",
                    "-p",
                    ["model_path:=", LaunchConfiguration("bc_model_path")],
                    "-p",
                    ["route_file:=", LaunchConfiguration("route_file")],
                    "-p",
                    ["route_csv_frame:=", LaunchConfiguration("route_csv_frame")],
                    "-p",
                    ["control_topic:=", LaunchConfiguration("bc_control_topic")],
                    "-p",
                    ["target_speed_kmh:=", LaunchConfiguration("bc_target_speed_kmh")],
                    "-p",
                    ["future_distances:=", LaunchConfiguration("bc_future_distances")],
                    "-p",
                    ["command_mode:=", LaunchConfiguration("bc_command_mode")],
                    "-p",
                    ["rule_red_light_stop:=", LaunchConfiguration("bc_enable_traffic_light_stop")],
                    "-p",
                    ["waypoint_blend_base:=", LaunchConfiguration("bc_waypoint_blend_base")],
                    "-p",
                    ["waypoint_blend_max:=", LaunchConfiguration("bc_waypoint_blend_max")],
                    "-p",
                    ["waypoint_blend_cte_gain:=", LaunchConfiguration("bc_waypoint_blend_cte_gain")],
                    "-p",
                    ["max_model_steer_delta:=", LaunchConfiguration("bc_max_model_steer_delta")],
                    "-p",
                    ["waypoint_correction_enabled:=", LaunchConfiguration("bc_waypoint_correction_enabled")],
                    "-p",
                    ["flip_future_y_for_model:=", LaunchConfiguration("bc_flip_future_y_for_model")],
                    "-p",
                    ["cpu:=", LaunchConfiguration("bc_cpu")],
                ],
                name="bc_future_il_controller",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_bc_il")),
            )
        ],
    )

    camera_follow = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    _python_script("utils", "camera_follow.py"),
                ],
                name="camera_follow",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_camera_follow")),
            )
        ],
    )

    control_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="carla_autonomy",
                executable="master_control_node",
                name="master_control_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": True,
                        "route_file": LaunchConfiguration("route_file"),
                        "route_csv_frame": LaunchConfiguration("route_csv_frame"),
                        "enable_unsigned_stopline_stop": False,
                        "use_il_normal_control": ParameterValue(
                            LaunchConfiguration("use_il_normal_control"),
                            value_type=bool,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_control")),
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
            SetEnvironmentVariable("MPLCONFIGDIR", "/tmp/matplotlib"),
            SetEnvironmentVariable("YOLO_CONFIG_DIR", "/tmp/Ultralytics"),
            DeclareLaunchArgument("host", default_value="localhost"),
            DeclareLaunchArgument("port", default_value="2000"),
            DeclareLaunchArgument("timeout", default_value="10"),
            DeclareLaunchArgument("town", default_value="Town04"),
            DeclareLaunchArgument("passive", default_value="False"),
            DeclareLaunchArgument("synchronous_mode", default_value="True"),
            DeclareLaunchArgument("fixed_delta_seconds", default_value="0.05"),
            DeclareLaunchArgument("spawn_sensors_only", default_value="False"),
            DeclareLaunchArgument("route_file", default_value=""),
            DeclareLaunchArgument("route_csv_frame", default_value="ros"),
            DeclareLaunchArgument("bc_model_path", default_value=""),
            DeclareLaunchArgument("traffic_light_model_path", default_value=""),
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("bc_control_topic", default_value="/carla/ego_vehicle/vehicle_control_cmd_il"),
            DeclareLaunchArgument("bc_target_speed_kmh", default_value="30.0"),
            DeclareLaunchArgument("bc_future_distances", default_value="4,8,12"),
            DeclareLaunchArgument("bc_command_mode", default_value="route"),
            DeclareLaunchArgument("bc_enable_traffic_light_stop", default_value="False"),
            DeclareLaunchArgument("bc_waypoint_blend_base", default_value="0.55"),
            DeclareLaunchArgument("bc_waypoint_blend_max", default_value="0.95"),
            DeclareLaunchArgument("bc_waypoint_blend_cte_gain", default_value="0.22"),
            DeclareLaunchArgument("bc_max_model_steer_delta", default_value="0.20"),
            DeclareLaunchArgument("bc_waypoint_correction_enabled", default_value="False"),
            DeclareLaunchArgument("bc_flip_future_y_for_model", default_value="False"),
            DeclareLaunchArgument("bc_cpu", default_value="False"),
            DeclareLaunchArgument("use_il_normal_control", default_value="True"),
            DeclareLaunchArgument("start_bridge", default_value="True"),
            DeclareLaunchArgument("start_spawn", default_value="True"),
            DeclareLaunchArgument("start_perception", default_value="True"),
            DeclareLaunchArgument("start_bc_il", default_value="True"),
            DeclareLaunchArgument("start_obstacles", default_value="True"),
            DeclareLaunchArgument("start_camera_follow", default_value="True"),
            DeclareLaunchArgument("start_control", default_value="True"),
            LogInfo(msg=f"Using CARLA objects definition: {objects_definition_file}"),
            bridge_launch,
            spawn_ego,
            perception_nodes,
            bc_il_node,
            obstacle_nodes,
            camera_follow,
            control_node,
        ]
    )
