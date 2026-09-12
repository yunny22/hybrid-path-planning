#!/usr/bin/env python3
import csv
import math
import re
from pathlib import Path

import cv2
import numpy as np
from PIL import Image as PilImage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from carla_msgs.msg import CarlaEgoVehicleControl

import torch
import torch.nn as nn

import carla


class FutureWaypointSteerOnly(nn.Module):
    def __init__(self, feature_dim):
        super().__init__()
        self.cnn = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.ELU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ELU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ELU(),
            nn.Conv2d(48, 64, kernel_size=3, stride=1),
            nn.ELU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ELU(),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
        )
        self.head = nn.Sequential(
            nn.Linear(64 + feature_dim, 128),
            nn.ELU(),
            nn.Dropout(0.1),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, 1),
        )

    def forward(self, image, feature):
        x = torch.cat([self.cnn(image), feature], dim=1)
        return torch.tanh(self.head(x))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def normalize_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def normalize_name(value):
    return str(value).split(".")[-1].upper()


def get_speed_kmh(vehicle):
    velocity = vehicle.get_velocity()
    return 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)


def get_traffic_light_info(vehicle):
    try:
        if vehicle.is_at_traffic_light():
            traffic_light = vehicle.get_traffic_light()
            if traffic_light is not None:
                return 1, normalize_name(traffic_light.get_state())
            return 1, "UNKNOWN"
    except Exception:
        return 0, "UNKNOWN"
    return 0, "NONE"


def make_speed_control(
    speed_kmh,
    target_speed_kmh,
    throttle_kp,
    brake_kp,
    max_throttle,
    max_brake,
    red_stop=False,
    stop_brake=0.75,
):
    if red_stop:
        return 0.0, stop_brake

    error = target_speed_kmh - speed_kmh
    if error >= 0.0:
        return clamp(throttle_kp * error, 0.0, max_throttle), 0.0
    return 0.0, clamp(brake_kp * (-error), 0.0, max_brake)


def convert_route_point(x, y, z, yaw, csv_frame):
    if csv_frame == "ros":
        return x, -y, z, -yaw
    if csv_frame == "carla":
        return x, y, z, yaw
    raise ValueError(csv_frame)


class BCFutureILController(Node):
    def __init__(self):
        super().__init__("bc_future_il_controller")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 2000)
        self.declare_parameter("role_name", "ego_vehicle")
        # The portfolio release intentionally ships without private route data
        # or learned weights.  Supply both paths as ROS parameters at runtime.
        self.declare_parameter("model_path", "")
        self.declare_parameter("route_file", "")
        self.declare_parameter("route_csv_frame", "ros")
        self.declare_parameter("route_z_offset", 0.3)
        self.declare_parameter("min_point_gap", 0.3)
        self.declare_parameter("route_search_window", 180)
        self.declare_parameter("future_distances", "4,8,12")
        self.declare_parameter("command_mode", "route")
        self.declare_parameter("fixed_command", "LANEFOLLOW")
        self.declare_parameter("command_probe_distance", 15.0)
        self.declare_parameter("turn_threshold", 12.0)
        self.declare_parameter("image_topic", "/carla/ego_vehicle/rgb_bc/image")
        self.declare_parameter("control_topic", "/carla/ego_vehicle/vehicle_control_cmd_il")
        self.declare_parameter("status_topic", "/hybrid_control/bc_il_status")
        self.declare_parameter("target_speed_kmh", 30.0)
        self.declare_parameter("throttle_kp", 0.08)
        self.declare_parameter("brake_kp", 0.08)
        self.declare_parameter("max_throttle", 0.40)
        self.declare_parameter("max_brake", 0.30)
        self.declare_parameter("max_steer", 0.90)
        self.declare_parameter("steer_gain", 1.0)
        self.declare_parameter("steer_smoothing", 0.10)
        self.declare_parameter("rule_red_light_stop", False)
        self.declare_parameter("tl_stop_brake", 0.75)
        self.declare_parameter("start_assist_seconds", 0.0)
        self.declare_parameter("start_assist_throttle", 0.30)
        self.declare_parameter("waypoint_correction_enabled", False)
        self.declare_parameter("waypoint_blend_base", 0.55)
        self.declare_parameter("waypoint_blend_max", 0.95)
        self.declare_parameter("waypoint_blend_cte_gain", 0.22)
        self.declare_parameter("max_model_steer_delta", 0.20)
        self.declare_parameter("pure_pursuit_wheelbase", 2.88)
        self.declare_parameter("flip_future_y_for_model", False)
        self.declare_parameter("cpu", False)
        self.declare_parameter("route_end_stop_distance", 4.5)

        self.role_name = self.get_parameter("role_name").value
        self.route_csv_frame = str(self.get_parameter("route_csv_frame").value)
        self.route_z_offset = float(self.get_parameter("route_z_offset").value)
        self.min_point_gap = float(self.get_parameter("min_point_gap").value)
        self.route_search_window = int(self.get_parameter("route_search_window").value)
        self.future_distances_arg = str(self.get_parameter("future_distances").value)
        self.command_mode = str(self.get_parameter("command_mode").value)
        self.fixed_command = normalize_name(self.get_parameter("fixed_command").value)
        self.command_probe_distance = float(self.get_parameter("command_probe_distance").value)
        self.turn_threshold = float(self.get_parameter("turn_threshold").value)
        self.target_speed_kmh = float(self.get_parameter("target_speed_kmh").value)
        self.throttle_kp = float(self.get_parameter("throttle_kp").value)
        self.brake_kp = float(self.get_parameter("brake_kp").value)
        self.max_throttle = float(self.get_parameter("max_throttle").value)
        self.max_brake = float(self.get_parameter("max_brake").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.steer_gain = float(self.get_parameter("steer_gain").value)
        self.steer_smoothing = float(self.get_parameter("steer_smoothing").value)
        self.rule_red_light_stop = bool(self.get_parameter("rule_red_light_stop").value)
        self.tl_stop_brake = float(self.get_parameter("tl_stop_brake").value)
        self.start_assist_seconds = float(self.get_parameter("start_assist_seconds").value)
        self.start_assist_throttle = float(self.get_parameter("start_assist_throttle").value)
        self.waypoint_correction_enabled = bool(self.get_parameter("waypoint_correction_enabled").value)
        self.waypoint_blend_base = float(self.get_parameter("waypoint_blend_base").value)
        self.waypoint_blend_max = float(self.get_parameter("waypoint_blend_max").value)
        self.waypoint_blend_cte_gain = float(self.get_parameter("waypoint_blend_cte_gain").value)
        self.max_model_steer_delta = float(self.get_parameter("max_model_steer_delta").value)
        self.pure_pursuit_wheelbase = float(self.get_parameter("pure_pursuit_wheelbase").value)
        self.flip_future_y_for_model = bool(self.get_parameter("flip_future_y_for_model").value)
        self.route_end_stop_distance = float(self.get_parameter("route_end_stop_distance").value)

        model_path_value = str(self.get_parameter("model_path").value).strip()
        route_file_value = str(self.get_parameter("route_file").value).strip()
        if not model_path_value:
            raise ValueError("model_path is required; this repository does not include BC weights")
        if not route_file_value:
            raise ValueError("route_file is required; pass a CSV with x,y,z,yaw columns")

        self.bridge = CvBridge()
        self.latest_image = None
        self.vehicle = None
        self.route_progress_idx = 0
        self.prev_steer = 0.0
        self.route = self.load_route(Path(route_file_value))
        self.route_s = self.compute_route_s(self.route)
        self.start_time = self.get_clock().now()

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() and not bool(self.get_parameter("cpu").value) else "cpu"
        )
        self.model, self.config, self.nav_cols, self.command_to_idx = self.load_model(
            Path(model_path_value)
        )
        self.future_distances = self.parse_future_distances(self.nav_cols)
        if self.future_distances_arg:
            self.future_distances = [
                float(value.strip()) for value in self.future_distances_arg.split(",") if value.strip()
            ]
        self.nav_x_scale = float(self.config.get("nav_x_scale", 20.0))
        self.nav_y_scale = float(self.config.get("nav_y_scale", 10.0))
        self.crop_top = int(self.config.get("crop_top", 120))
        self.resize_width = int(self.config.get("resize_width", 200))
        self.resize_height = int(self.config.get("resize_height", 88))

        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.image_callback,
            10,
        )
        self.control_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            self.get_parameter("control_topic").value,
            10,
        )
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)

        self.client = carla.Client(
            self.get_parameter("host").value,
            int(self.get_parameter("port").value),
        )
        self.client.set_timeout(2.0)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            f"BC IL controller ready: model={self.get_parameter('model_path').value}, "
            f"route_points={len(self.route)}, csv_frame={self.route_csv_frame}, "
            f"distances={self.future_distances}, device={self.device}"
        )

    def load_route(self, route_file):
        route = []
        if not route_file.exists():
            self.get_logger().warn(f"Route file not found: {route_file}. Falling back to CARLA lane waypoints.")
            return route

        with route_file.open("r", newline="") as f:
            reader = csv.DictReader(f)
            last = None
            for row in reader:
                try:
                    x, y, z, yaw = convert_route_point(
                        float(row["x"]),
                        float(row["y"]),
                        float(row.get("z", 0.0)),
                        float(row.get("yaw", 0.0)),
                        self.route_csv_frame,
                    )
                    point = {
                        "x": x,
                        "y": y,
                        "z": z + self.route_z_offset,
                        "yaw": yaw,
                    }
                except (KeyError, TypeError, ValueError):
                    continue
                if last is not None and math.hypot(point["x"] - last["x"], point["y"] - last["y"]) < self.min_point_gap:
                    continue
                route.append(point)
                last = point
        return route

    @staticmethod
    def compute_route_s(route):
        if not route:
            return []

        route_s = [0.0]
        for i in range(1, len(route)):
            prev = route[i - 1]
            cur = route[i]
            route_s.append(route_s[-1] + math.hypot(cur["x"] - prev["x"], cur["y"] - prev["y"]))
        return route_s

    def load_model(self, model_path):
        if not model_path.exists():
            raise FileNotFoundError(model_path)

        checkpoint = torch.load(str(model_path), map_location=self.device)
        config = checkpoint.get("config", {})
        nav_cols = checkpoint.get(
            "nav_cols",
            ["future_x_4m", "future_y_4m", "future_x_8m", "future_y_8m", "future_x_12m", "future_y_12m"],
        )
        command_to_idx = checkpoint.get(
            "command_to_idx",
            {
                "LANEFOLLOW": 0,
                "STRAIGHT": 1,
                "LEFT": 2,
                "RIGHT": 3,
                "CHANGELANELEFT": 4,
                "CHANGELANERIGHT": 5,
                "UNKNOWN": 6,
            },
        )
        feature_dim = len(nav_cols) + len(command_to_idx)
        model = FutureWaypointSteerOnly(feature_dim).to(self.device)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.eval()
        return model, config, nav_cols, command_to_idx

    @staticmethod
    def parse_future_distances(nav_cols):
        distances = []
        for col in nav_cols:
            if not col.startswith("future_x_"):
                continue
            match = re.search(r"_(\d+(?:\.\d+)?)m$", col)
            if match:
                distances.append(float(match.group(1)))
        return distances or [4.0, 8.0, 12.0]

    def image_callback(self, msg):
        self.latest_image = msg

    def connect_vehicle(self):
        try:
            world = self.client.get_world()
            for actor in world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") == self.role_name:
                    self.vehicle = actor
                    self.get_logger().info(f"Connected to ego vehicle id={actor.id}")
                    return True
        except Exception as exc:
            self.get_logger().warn(f"Waiting for CARLA ego vehicle: {exc}")
        return False

    def ros_image_to_pil(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        return PilImage.fromarray(rgb)

    def preprocess_image(self, image):
        image = image.convert("RGB")
        if self.crop_top > 0 and self.crop_top < image.height:
            image = image.crop((0, self.crop_top, image.width, image.height))
        image = image.resize((self.resize_width, self.resize_height))

        arr = np.asarray(image).astype(np.float32) / 255.0
        arr = (arr - 0.5) / 0.5
        arr = np.transpose(arr, (2, 0, 1))
        return torch.from_numpy(arr)

    def find_reference_index(self, cur_x, cur_y):
        if not self.route:
            return 0

        start = max(0, self.route_progress_idx - 30)
        end = min(len(self.route), self.route_progress_idx + self.route_search_window)
        best_dist = 1e18
        best_idx = self.route_progress_idx

        for i in range(start, end):
            dx = self.route[i]["x"] - cur_x
            dy = self.route[i]["y"] - cur_y
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        if best_idx > self.route_progress_idx:
            self.route_progress_idx = best_idx
        return best_idx

    def find_index_after_s(self, start_idx, distance):
        if not self.route_s:
            return 0

        target_s = self.route_s[start_idx] + distance
        lo = start_idx
        hi = len(self.route_s)
        while lo < hi:
            mid = (lo + hi) // 2
            if self.route_s[mid] < target_s:
                lo = mid + 1
            else:
                hi = mid
        return min(lo, len(self.route_s) - 1)

    def route_future_points(self, ref_idx):
        if not self.route:
            return []

        return [self.route[self.find_index_after_s(ref_idx, distance)] for distance in self.future_distances]

    def lane_future_points(self):
        try:
            world_map = self.vehicle.get_world().get_map()
            waypoint = world_map.get_waypoint(
                self.vehicle.get_location(),
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
            points = []
            for distance in self.future_distances:
                next_wps = waypoint.next(distance)
                wp = next_wps[0] if next_wps else waypoint
                loc = wp.transform.location
                points.append({"x": loc.x, "y": loc.y, "z": loc.z})
            return points
        except Exception:
            loc = self.vehicle.get_location()
            return [{"x": loc.x, "y": loc.y, "z": loc.z} for _ in self.future_distances]

    def route_command(self, ref_idx):
        if self.command_mode == "lanefollow":
            return "LANEFOLLOW"
        if self.command_mode == "fixed":
            return self.fixed_command
        if not self.route or ref_idx >= len(self.route) - 2:
            return "LANEFOLLOW"

        yaw_now = self.route[ref_idx]["yaw"]
        look_idx = self.find_index_after_s(ref_idx, self.command_probe_distance)
        yaw_ahead = self.route[look_idx]["yaw"]
        yaw_delta = normalize_angle_deg(yaw_ahead - yaw_now)

        if abs(yaw_delta) < self.turn_threshold:
            return "LANEFOLLOW"
        return "RIGHT" if yaw_delta > 0.0 else "LEFT"

    def build_local_route_path(self, ref_idx, loc, yaw):
        path = [(0.0, 0.0)]
        if not self.route:
            return path

        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        last_x = -1e9
        last_y = 0.0

        for point in self.route[ref_idx:min(len(self.route), ref_idx + 120)]:
            dx = point["x"] - loc.x
            dy = point["y"] - loc.y
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw

            if local_x < 0.25:
                continue
            if abs(math.atan2(local_y, max(local_x, 0.1))) > 1.25:
                continue
            if path and local_x < last_x - 0.15:
                continue
            if path and math.hypot(local_x - last_x, local_y - last_y) < 0.15:
                continue

            path.append((local_x, local_y))
            last_x = local_x
            last_y = local_y
            if local_x > 45.0:
                break

        return path

    @staticmethod
    def path_heading_at(path, idx):
        if len(path) < 2:
            return 0.0
        i0 = max(0, idx - 1)
        i1 = min(len(path) - 1, idx + 1)
        return math.atan2(path[i1][1] - path[i0][1], max(path[i1][0] - path[i0][0], 1e-3))

    @staticmethod
    def lateral_at_x(path, xq):
        if not path:
            return 0.0
        if xq <= path[0][0]:
            return path[0][1]
        for i in range(1, len(path)):
            if xq <= path[i][0]:
                x0, y0 = path[i - 1]
                x1, y1 = path[i]
                t = clamp((xq - x0) / max(x1 - x0, 1e-3), 0.0, 1.0)
                return y0 + (y1 - y0) * t
        return path[-1][1]

    def pure_pursuit_steer(self, path, speed_kmh):
        if len(path) < 3:
            return 0.0

        speed_mps = speed_kmh / 3.6
        lookahead = clamp(3.8 + 0.24 * speed_mps, 3.0, 7.0)
        accum = 0.0
        prev_x, prev_y = 0.0, 0.0
        target = path[-1]
        target_idx = len(path) - 1

        for idx, (x, y) in enumerate(path):
            accum += math.hypot(x - prev_x, y - prev_y)
            prev_x, prev_y = x, y
            if accum >= lookahead:
                target = (x, y)
                target_idx = idx
                break

        alpha = math.atan2(target[1], max(target[0], 0.5))
        ld = max(math.hypot(target[0], target[1]), 1.8)
        pp_term = math.atan2(2.0 * self.pure_pursuit_wheelbase * math.sin(alpha), ld)
        heading_term = self.path_heading_at(path, target_idx)
        cte_term = math.atan2(0.80 * self.lateral_at_x(path, min(lookahead * 0.65, 4.0)), speed_mps + 4.0)
        raw = 0.85 * pp_term + 0.20 * heading_term + 0.25 * cte_term

        # In CARLA vehicle-local coordinates, +y is to the vehicle's right,
        # and +steer also turns right.
        return clamp(raw, -self.max_steer, self.max_steer)

    def blend_waypoint_steer(self, model_steer, route_steer, cte):
        if not self.waypoint_correction_enabled:
            return model_steer, 0.0

        weight = clamp(
            self.waypoint_blend_base + self.waypoint_blend_cte_gain * abs(cte),
            0.0,
            self.waypoint_blend_max,
        )
        bounded_model = clamp(
            model_steer,
            route_steer - self.max_model_steer_delta,
            route_steer + self.max_model_steer_delta,
        )
        return (1.0 - weight) * bounded_model + weight * route_steer, weight

    def feature_vector(self):
        transform = self.vehicle.get_transform()
        loc = transform.location
        yaw = math.radians(transform.rotation.yaw)
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)

        ref_idx = self.find_reference_index(loc.x, loc.y)
        local_path = self.build_local_route_path(ref_idx, loc, yaw)
        future_points = self.route_future_points(ref_idx)
        if not future_points:
            future_points = self.lane_future_points()

        nav_features = []
        for point in future_points:
            dx = point["x"] - loc.x
            dy = point["y"] - loc.y
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw
            model_y = -local_y if self.flip_future_y_for_model else local_y
            nav_features.extend(
                [
                    local_x / self.nav_x_scale,
                    model_y / self.nav_y_scale,
                ]
            )

        command = self.route_command(ref_idx)
        if command not in self.command_to_idx:
            command = "UNKNOWN"
        onehot = [0.0] * len(self.command_to_idx)
        onehot[int(self.command_to_idx[command])] = 1.0
        return onehot + nav_features, command, local_path

    def timer_callback(self):
        if self.vehicle is None:
            self.connect_vehicle()
            return
        if self.latest_image is None:
            return

        try:
            speed_kmh = get_speed_kmh(self.vehicle)
            is_at_tl, tl_state = get_traffic_light_info(self.vehicle)
            red_stop = self.rule_red_light_stop and is_at_tl == 1 and tl_state in ("RED", "YELLOW")
            route_end_stop = False

            if self.route:
                end_point = self.route[-1]
                end_distance = math.hypot(end_point["x"] - self.vehicle.get_location().x, end_point["y"] - self.vehicle.get_location().y)
                route_end_stop = end_distance <= self.route_end_stop_distance

            image = self.preprocess_image(self.ros_image_to_pil(self.latest_image)).unsqueeze(0).to(self.device)
            features, command, local_path = self.feature_vector()
            feature_tensor = torch.tensor(features, dtype=torch.float32, device=self.device).unsqueeze(0)

            with torch.no_grad():
                model_steer = float(self.model(image, feature_tensor)[0, 0].detach().cpu().item())

            route_steer = self.pure_pursuit_steer(local_path, speed_kmh)
            cte = self.lateral_at_x(local_path, 4.0)
            steer, blend_weight = self.blend_waypoint_steer(model_steer, route_steer, cte)
            steer_raw = clamp(steer * self.steer_gain, -self.max_steer, self.max_steer)
            steer = self.steer_smoothing * self.prev_steer + (1.0 - self.steer_smoothing) * steer_raw
            steer = clamp(steer, -self.max_steer, self.max_steer)
            self.prev_steer = steer

            if route_end_stop:
                throttle = 0.0
                brake = 0.85 if speed_kmh > 1.0 else 1.0
            else:
                throttle, brake = make_speed_control(
                    speed_kmh,
                    self.target_speed_kmh,
                    self.throttle_kp,
                    self.brake_kp,
                    self.max_throttle,
                    self.max_brake,
                    red_stop=red_stop,
                    stop_brake=self.tl_stop_brake,
                )

            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed < self.start_assist_seconds and speed_kmh < 2.0 and not red_stop:
                throttle = max(throttle, self.start_assist_throttle)
                brake = 0.0

            control = CarlaEgoVehicleControl()
            control.steer = float(steer)
            control.throttle = float(clamp(throttle, 0.0, 1.0))
            control.brake = float(clamp(brake, 0.0, 1.0))
            control.hand_brake = False
            control.reverse = False
            control.manual_gear_shift = False
            self.control_pub.publish(control)

            status = String()
            status.data = (
                f"speed={speed_kmh:.2f}, command={command}, tl={tl_state}, "
                f"model={model_steer:.3f}, route={route_steer:.3f}, "
                f"final={control.steer:.3f}, cte={cte:.2f}, blend={blend_weight:.2f}, "
                f"csv_frame={self.route_csv_frame}, flip_y={int(self.flip_future_y_for_model)}, "
                f"throttle={control.throttle:.3f}, brake={control.brake:.3f}, route_end_stop={int(route_end_stop)}"
            )
            self.status_pub.publish(status)

        except Exception as exc:
            self.get_logger().error(f"BC IL control step failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = BCFutureILController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
