#!/usr/bin/env python3
import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN


class LidarPerception(Node):
    """
    detected_obstacles publish format (10 floats per obstacle):
      [track_id, gx, gy, size_x, size_y, size_z, type_id, est_speed, camera_confirmed, motion_state]

    type_id:
      1 = pedestrian
      2 = vehicle

    motion_state:
      0 = STATIC
      1 = SLOW_MOVING_IN_LANE
      2 = CUT_IN / CROSSING
      3 = ONCOMING / OTHER_LANE_MOVING
      4 = UNKNOWN
    """

    STATIC = 0
    SLOW_MOVING_IN_LANE = 1
    CUT_IN = 2
    OTHER_LANE_MOVING = 3
    UNKNOWN = 4

    def __init__(self):
        super().__init__('lidar_perception')

        self.declare_parameter('role_name', 'ego_vehicle')
        role_name = self.get_parameter('role_name').value

        self.declare_parameter('lidar_topic', f'/carla/{role_name}/lidar')
        self.declare_parameter('odom_topic', f'/carla/{role_name}/odometry')
        self.declare_parameter('obstacle_topic', '/detected_obstacles')
        self.declare_parameter('marker_topic', '/detected_obstacles_marker')
        self.declare_parameter('frame_id', f'{role_name}/lidar')

        self.declare_parameter('max_forward_range', 55.0)
        self.declare_parameter('max_lateral_range', 14.0)
        self.declare_parameter('min_cluster_points', 4)
        self.declare_parameter('use_yolo_fusion', True)

        self.declare_parameter('dbscan_eps', 0.85)
        self.declare_parameter('track_match_distance', 3.0)
        self.declare_parameter('max_missed_frames', 5)
        self.declare_parameter('history_size', 10)

        self.declare_parameter('static_speed_enter_thresh', 0.28)
        self.declare_parameter('static_speed_exit_thresh', 0.95)
        self.declare_parameter('static_hold_frames', 4)
        self.declare_parameter('moving_hold_frames', 3)

        # motion thresholds (non-sticky classification)
        self.declare_parameter('static_speed_thresh', 0.80)              # < 2.9 km/h
        self.declare_parameter('slow_vehicle_min_speed_thresh', 0.80)    # >= 2.9 km/h
        self.declare_parameter('slow_vehicle_speed_thresh', 3.20)        # <= 11.5 km/h
        self.declare_parameter('cutin_min_speed_thresh', 2.80)           # >= 10.1 km/h
        self.declare_parameter('crossing_lateral_speed_thresh', 1.20)    # significant lateral motion
        self.declare_parameter('cutin_min_lateral_offset', 1.20)
        self.declare_parameter('cutin_approach_rate_thresh', 0.45)
        self.declare_parameter('in_lane_half_width', 2.2)
        self.declare_parameter('path_half_width_margin', 0.8)
        self.declare_parameter('yolo_match_margin_px', 120.0)

        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.max_forward_range = float(self.get_parameter('max_forward_range').value)
        self.max_lateral_range = float(self.get_parameter('max_lateral_range').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.use_yolo_fusion = bool(self.get_parameter('use_yolo_fusion').value)

        self.dbscan_eps = float(self.get_parameter('dbscan_eps').value)
        self.track_match_distance = float(self.get_parameter('track_match_distance').value)
        self.max_missed_frames = int(self.get_parameter('max_missed_frames').value)
        self.history_size = int(self.get_parameter('history_size').value)

        self.static_speed_enter_thresh = float(self.get_parameter('static_speed_enter_thresh').value)
        self.static_speed_exit_thresh = float(self.get_parameter('static_speed_exit_thresh').value)
        self.static_hold_frames = int(self.get_parameter('static_hold_frames').value)
        self.moving_hold_frames = int(self.get_parameter('moving_hold_frames').value)

        self.static_speed_thresh = float(self.get_parameter('static_speed_thresh').value)
        self.slow_vehicle_min_speed_thresh = float(self.get_parameter('slow_vehicle_min_speed_thresh').value)
        self.slow_vehicle_speed_thresh = float(self.get_parameter('slow_vehicle_speed_thresh').value)
        self.cutin_min_speed_thresh = float(self.get_parameter('cutin_min_speed_thresh').value)
        self.crossing_lateral_speed_thresh = float(self.get_parameter('crossing_lateral_speed_thresh').value)
        self.cutin_min_lateral_offset = float(self.get_parameter('cutin_min_lateral_offset').value)
        self.cutin_approach_rate_thresh = float(self.get_parameter('cutin_approach_rate_thresh').value)
        self.in_lane_half_width = float(self.get_parameter('in_lane_half_width').value)
        self.path_half_width_margin = float(self.get_parameter('path_half_width_margin').value)
        self.yolo_match_margin_px = float(self.get_parameter('yolo_match_margin_px').value)

        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0

        self.latest_bboxes: List[Tuple[int, float, float, float, float]] = []
        self.yolo_received = False

        self.tracks: Dict[int, dict] = {}
        self.track_id_counter = 0

        self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(Float32MultiArray, '/yolo_detections', self.yolo_cb, 10)

        self.obstacle_pub = self.create_publisher(Float32MultiArray, self.obstacle_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.get_logger().info(
            'LidarPerception started | stable track-based motion classification + YOLO-confirmed tracking'
        )

    def odom_cb(self, msg: Odometry):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w

        siny = 2.0 * (ow * oz + ox * oy)
        cosy = 1.0 - 2.0 * (oy * oy + oz * oz)
        self.ego_yaw = math.atan2(siny, cosy)

    def yolo_cb(self, msg: Float32MultiArray):
        self.latest_bboxes.clear()
        data = list(msg.data)

        if len(data) < 5 or len(data) % 5 != 0:
            self.yolo_received = False
            return

        for i in range(0, len(data), 5):
            cls_id = int(data[i + 0])
            x1 = float(data[i + 1])
            y1 = float(data[i + 2])
            x2 = float(data[i + 3])
            y2 = float(data[i + 4])
            self.latest_bboxes.append((cls_id, x1, y1, x2, y2))

        self.yolo_received = True

    def lidar_cb(self, msg: PointCloud2):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        points = np.array(
            [[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True)],
            dtype=np.float32
        )

        if points.size == 0:
            self._age_tracks_without_measurement()
            self._publish_from_tracks(msg)
            return

        roi_mask = (
            (points[:, 0] > 0.6) &
            (points[:, 0] < self.max_forward_range) &
            (np.abs(points[:, 1]) < self.max_lateral_range) &
            (points[:, 2] > -1.8) &
            (points[:, 2] < 2.8)
        )
        roi_points = points[roi_mask]

        if roi_points.shape[0] < self.min_cluster_points:
            self._age_tracks_without_measurement()
            self._publish_from_tracks(msg)
            return

        if self.use_yolo_fusion and self.yolo_received and len(self.latest_bboxes) > 0:
            roi_points, point_camera_confirmed, point_yolo_cls = self._fuse_yolo(roi_points)
        else:
            point_camera_confirmed = np.zeros((roi_points.shape[0],), dtype=np.int32)
            point_yolo_cls = np.full((roi_points.shape[0],), -1, dtype=np.int32)

        if roi_points.shape[0] < self.min_cluster_points:
            self._age_tracks_without_measurement()
            self._publish_from_tracks(msg)
            return

        labels = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.min_cluster_points
        ).fit(roi_points[:, :2]).labels_

        used_track_ids = set()
        measurements = []

        for label in sorted(set(labels)):
            if label == -1:
                continue

            cluster_mask = (labels == label)
            cluster = roi_points[cluster_mask]
            cam_flags = point_camera_confirmed[cluster_mask]
            yolo_cls_ids = point_yolo_cls[cluster_mask]

            if cluster.shape[0] < self.min_cluster_points:
                continue

            min_xyz = cluster.min(axis=0)
            max_xyz = cluster.max(axis=0)
            size_x, size_y, size_z = map(float, max_xyz - min_xyz)

            if size_x > 8.0 or size_y > 8.0:
                continue
            if size_x < 0.18 and size_y < 0.18 and size_z < 0.15:
                continue

            cx_local = float(cluster[:, 0].mean())
            cy_local = float(cluster[:, 1].mean())
            cz_local = float(cluster[:, 2].mean())

            gx, gy = self._local_to_global(cx_local, cy_local)

            matched_count = int(np.sum(cam_flags > 0))
            camera_confirmed_now = matched_count >= max(3, int(0.15 * cluster.shape[0]))

            dominant_cls = self._majority_valid_class(yolo_cls_ids)
            semantic_type = self._type_id_from_yolo_class(dominant_cls)

            measurements.append({
                'gx': gx,
                'gy': gy,
                'cx_local': cx_local,
                'cy_local': cy_local,
                'cz_local': cz_local,
                'size_x': size_x,
                'size_y': size_y,
                'size_z': size_z,
                'camera_confirmed_now': camera_confirmed_now,
                'dominant_cls': dominant_cls,
                'semantic_type': semantic_type,
                'matched_count': matched_count,
                'point_count': int(cluster.shape[0]),
            })

        for meas in measurements:
            best_tid = self._find_best_track(meas['gx'], meas['gy'], now_sec, used_track_ids)

            if best_tid is not None:
                self._update_track_from_measurement(best_tid, meas, now_sec)
                used_track_ids.add(best_tid)
                continue

            if meas['camera_confirmed_now']:
                new_tid = self.track_id_counter
                self.track_id_counter += 1
                self.tracks[new_tid] = self._make_new_track(meas, now_sec)
                used_track_ids.add(new_tid)

        to_delete = []
        for tid, trk in self.tracks.items():
            if tid not in used_track_ids:
                trk['missed'] += 1
                if trk['missed'] > self.max_missed_frames:
                    to_delete.append(tid)

        for tid in to_delete:
            del self.tracks[tid]

        self._publish_from_tracks(msg)

    def _make_new_track(self, meas: dict, now_sec: float) -> dict:
        world_history: Deque[Tuple[float, float, float]] = deque(maxlen=self.history_size)
        local_history: Deque[Tuple[float, float, float]] = deque(maxlen=self.history_size)

        world_history.append((now_sec, meas['gx'], meas['gy']))
        local_history.append((now_sec, meas['cx_local'], meas['cy_local']))

        type_id = meas['semantic_type']
        if type_id == 0:
            type_id = self._type_id_from_size(meas['size_x'], meas['size_y'], meas['size_z'])

        label, code, _ = self._classify_motion(
            type_id=type_id,
            local_x=meas['cx_local'],
            local_y=meas['cy_local'],
            vx_world=0.0,
            vy_world=0.0,
            speed=0.0,
            age_frames=1,
            static_consistency=1,
            moving_consistency=0,
            lateral_approach_rate=0.0,
            prev_motion_state=self.UNKNOWN,
        )

        return {
            'pos': (meas['gx'], meas['gy']),
            'local_pos': (meas['cx_local'], meas['cy_local'], meas['cz_local']),
            'size': (meas['size_x'], meas['size_y'], meas['size_z']),
            'stamp': now_sec,
            'last_seen': now_sec,
            'missed': 0,

            'camera_confirmed': 1,
            'yolo_cls_id': meas['dominant_cls'],
            'type_id': type_id,

            'world_history': world_history,
            'local_history': local_history,

            'vx_world': 0.0,
            'vy_world': 0.0,
            'speed_ema': 0.0,

            'static_count': 1,
            'moving_count': 0,
            'prev_abs_local_y': abs(meas['cy_local']),
            'lateral_approach_rate': 0.0,

            'motion_label': label,
            'motion_state': code,
        }

    def _update_track_from_measurement(self, tid: int, meas: dict, now_sec: float):
        trk = self.tracks[tid]

        prev_last_seen = float(trk.get('last_seen', now_sec))
        prev_type = trk.get('type_id', 0)
        new_type = meas['semantic_type']
        if new_type == 0:
            new_type = prev_type if prev_type != 0 else self._type_id_from_size(
                meas['size_x'], meas['size_y'], meas['size_z']
            )

        trk['type_id'] = new_type
        if meas['dominant_cls'] >= 0:
            trk['yolo_cls_id'] = meas['dominant_cls']

        trk['camera_confirmed'] = 1 if (trk.get('camera_confirmed', 0) or meas['camera_confirmed_now']) else 0
        trk['pos'] = (meas['gx'], meas['gy'])
        trk['local_pos'] = (meas['cx_local'], meas['cy_local'], meas['cz_local'])
        trk['size'] = (meas['size_x'], meas['size_y'], meas['size_z'])
        trk['stamp'] = now_sec
        trk['last_seen'] = now_sec
        trk['missed'] = 0

        trk['world_history'].append((now_sec, meas['gx'], meas['gy']))
        trk['local_history'].append((now_sec, meas['cx_local'], meas['cy_local']))

        prev_abs_local_y = float(trk.get('prev_abs_local_y', abs(meas['cy_local'])))
        dt_local = max(now_sec - prev_last_seen, 1e-3)
        curr_abs_local_y = abs(meas['cy_local'])
        lateral_approach_rate_raw = max(0.0, (prev_abs_local_y - curr_abs_local_y) / dt_local)
        trk['prev_abs_local_y'] = curr_abs_local_y
        trk['lateral_approach_rate'] = (
            0.35 * lateral_approach_rate_raw + 0.65 * float(trk.get('lateral_approach_rate', 0.0))
        )

        vx_world, vy_world, speed = self._estimate_velocity_from_history(trk['world_history'])

        alpha = 0.30
        trk['vx_world'] = alpha * vx_world + (1.0 - alpha) * float(trk.get('vx_world', 0.0))
        trk['vy_world'] = alpha * vy_world + (1.0 - alpha) * float(trk.get('vy_world', 0.0))
        trk['speed_ema'] = alpha * speed + (1.0 - alpha) * float(trk.get('speed_ema', 0.0))

        if trk['speed_ema'] < self.static_speed_thresh:
            trk['static_count'] = trk.get('static_count', 0) + 1
        else:
            trk['static_count'] = 0

        if trk['speed_ema'] >= self.slow_vehicle_min_speed_thresh:
            trk['moving_count'] = trk.get('moving_count', 0) + 1
        else:
            trk['moving_count'] = 0

        label, code, _ = self._classify_motion(
            type_id=trk['type_id'],
            local_x=meas['cx_local'],
            local_y=meas['cy_local'],
            vx_world=trk['vx_world'],
            vy_world=trk['vy_world'],
            speed=trk['speed_ema'],
            age_frames=len(trk['world_history']),
            static_consistency=int(trk.get('static_count', 0)),
            moving_consistency=int(trk.get('moving_count', 0)),
            lateral_approach_rate=float(trk.get('lateral_approach_rate', 0.0)),
            prev_motion_state=int(trk.get('motion_state', self.UNKNOWN)),
        )
        trk['motion_label'] = label
        trk['motion_state'] = code

    def _find_best_track(self, gx: float, gy: float, now_sec: float, used_track_ids: set) -> Optional[int]:
        best_tid = None
        best_dist = float('inf')

        for tid, trk in self.tracks.items():
            if tid in used_track_ids:
                continue

            pred_x, pred_y = self._predict_track_position(trk, now_sec)
            d = math.hypot(gx - pred_x, gy - pred_y)

            dynamic_gate = self.track_match_distance + min(2.0, 0.5 * float(trk.get('speed_ema', 0.0)))
            if d < dynamic_gate and d < best_dist:
                best_dist = d
                best_tid = tid

        return best_tid

    def _predict_track_position(self, trk: dict, now_sec: float) -> Tuple[float, float]:
        dt = max(0.0, now_sec - float(trk.get('last_seen', now_sec)))
        px = trk['pos'][0] + float(trk.get('vx_world', 0.0)) * dt
        py = trk['pos'][1] + float(trk.get('vy_world', 0.0)) * dt
        return px, py

    def _age_tracks_without_measurement(self):
        to_delete = []
        for tid, trk in self.tracks.items():
            trk['missed'] += 1
            if trk['missed'] > self.max_missed_frames:
                to_delete.append(tid)

        for tid in to_delete:
            del self.tracks[tid]

    def _estimate_velocity_from_history(
        self,
        history: Deque[Tuple[float, float, float]]
    ) -> Tuple[float, float, float]:
        if len(history) < 2:
            return 0.0, 0.0, 0.0

        t0, x0, y0 = history[0]
        t1, x1, y1 = history[-1]
        dt = max(t1 - t0, 1e-3)

        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        speed = math.hypot(vx, vy)
        return vx, vy, speed

    def _world_velocity_to_ego(self, vx_world: float, vy_world: float) -> Tuple[float, float]:
        cos_y = math.cos(self.ego_yaw)
        sin_y = math.sin(self.ego_yaw)

        vx_ego = cos_y * vx_world + sin_y * vy_world
        vy_ego = -sin_y * vx_world + cos_y * vy_world
        return vx_ego, vy_ego

    def _classify_motion(
        self,
        type_id: int,
        local_x: float,
        local_y: float,
        vx_world: float,
        vy_world: float,
        speed: float,
        age_frames: int,
        static_consistency: int,
        moving_consistency: int,
        lateral_approach_rate: float,
        prev_motion_state: int,
    ) -> Tuple[str, int, float]:
        """
        Non-sticky classification.
        Every frame, classify from current smoothed kinematics only.

        motion_state:
          0 = STATIC
          1 = SLOW_MOVING_IN_LANE
          2 = CUT_IN / CROSSING
          3 = ONCOMING / OTHER_LANE_MOVING
          4 = UNKNOWN
        """

        vx_ego, vy_ego = self._world_velocity_to_ego(vx_world, vy_world)
        abs_vy = abs(vy_ego)
        abs_y = abs(local_y)

        if age_frames < 3:
            return 'UNKNOWN', 4, speed

        # 1) STATIC = stopped or near-stopped object
        if speed < self.static_speed_thresh:
            return 'STATIC', 0, 0.0

        # Pedestrian
        if type_id == 1:
            if speed < 0.6:
                return 'STATIC', 0, 0.0

            if abs_vy >= 0.9 and moving_consistency >= 2:
                return 'CUT_IN/CROSSING', 2, speed

            if moving_consistency >= 2:
                return 'ONCOMING/OTHER_LANE_MOVING', 3, speed

            return 'UNKNOWN', 4, speed

        # Vehicle
        if type_id == 2:
            in_lane_zone = (local_x > 0.0 and abs_y < self.in_lane_half_width)
            cutin_zone = abs_y >= self.cutin_min_lateral_offset

            # 2) CUT_IN/CROSSING:
            # must be actually moving fast enough + meaningful lateral motion + approaching our lane
            if (
                speed >= self.cutin_min_speed_thresh and
                abs_vy >= self.crossing_lateral_speed_thresh and
                cutin_zone and
                lateral_approach_rate >= self.cutin_approach_rate_thresh and
                moving_consistency >= 2
            ):
                return 'CUT_IN/CROSSING', 2, speed

            # 3) SLOW_MOVING_IN_LANE:
            # in our lane / path vicinity, moderate speed, mainly longitudinal
            if (
                in_lane_zone and
                self.slow_vehicle_min_speed_thresh <= speed <= self.slow_vehicle_speed_thresh and
                vx_ego > -1.0 and
                abs_vy < 0.8 and
                moving_consistency >= 2
            ):
                return 'SLOW_MOVING_IN_LANE', 1, speed

            # 4) OTHER_LANE_MOVING:
            # moving object that is not static, not slow lead, not cut-in
            if moving_consistency >= 2:
                return 'ONCOMING/OTHER_LANE_MOVING', 3, speed

            return 'UNKNOWN', 4, speed

        return 'UNKNOWN', 4, speed

    def _fuse_yolo(self, roi_points: np.ndarray):
        fused_points = []
        confirmed_flags = []
        matched_cls_ids = []

        max_coord = max([max(b[1], b[3]) for b in self.latest_bboxes]) if self.latest_bboxes else 800.0
        if max_coord > 1280:
            width, height = 1920.0, 1080.0
        elif max_coord > 800:
            width, height = 1280.0, 720.0
        else:
            width, height = 800.0, 600.0

        fx = width / 2.0
        fy = width / 2.0
        cx = width / 2.0
        cy = height / 2.0
        margin = self.yolo_match_margin_px

        for pt in roi_points:
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
            if x <= 0.2:
                continue

            u = fx * (-y / x) + cx
            v = fy * (-z / x) + cy

            matched = False
            matched_cls = -1

            for cls_id, x1, y1, x2, y2 in self.latest_bboxes:
                if (x1 - margin) <= u <= (x2 + margin) and (y1 - margin) <= v <= (y2 + margin):
                    matched = True
                    matched_cls = int(cls_id)
                    break

            fused_points.append(pt)
            confirmed_flags.append(1 if matched else 0)
            matched_cls_ids.append(matched_cls)

        if len(fused_points) == 0:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0,), dtype=np.int32),
                np.empty((0,), dtype=np.int32),
            )

        return (
            np.array(fused_points, dtype=np.float32),
            np.array(confirmed_flags, dtype=np.int32),
            np.array(matched_cls_ids, dtype=np.int32),
        )

    def _publish_from_tracks(self, msg: PointCloud2):
        detections: List[float] = []
        markers: List[Marker] = []
        marker_id = 0

        for tid in sorted(self.tracks.keys()):
            trk = self.tracks[tid]

            if trk.get('camera_confirmed', 0) == 0:
                continue

            cx_local, cy_local, cz_local = trk['local_pos']
            size_x, size_y, size_z = trk['size']
            type_id = int(trk.get('type_id', 2))
            speed = float(trk.get('speed_ema', 0.0))
            motion_state = int(trk.get('motion_state', self.UNKNOWN))
            motion_label = trk.get('motion_label', 'UNKNOWN')

            if type_id == 1:
                color = (0.2, 1.0, 0.2)
                kind = 'pedestrian'
            else:
                color = (1.0, 0.2, 0.2)
                kind = 'vehicle'

            gx, gy = trk['pos']

            detections.extend([
                float(tid),
                float(gx),
                float(gy),
                float(size_x),
                float(size_y),
                float(size_z),
                float(type_id),
                float(speed),
                1.0,
                float(motion_state),
            ])

            box = Marker()
            box.header.frame_id = self.frame_id
            box.header.stamp = msg.header.stamp
            box.ns = kind
            box.id = marker_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(cx_local)
            box.pose.position.y = float(cy_local)
            box.pose.position.z = float(cz_local)
            box.pose.orientation.w = 1.0
            box.scale.x = max(float(size_x) + 0.4, 0.5)
            box.scale.y = max(float(size_y) + 0.4, 0.5)
            box.scale.z = max(float(size_z) + 0.3, 0.6)
            box.color.r = color[0]
            box.color.g = color[1]
            box.color.b = color[2]
            box.color.a = 0.85
            box.lifetime.nanosec = 250_000_000
            markers.append(box)

            text = Marker()
            text.header = box.header
            text.ns = 'obstacle_text'
            text.id = 1000 + marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(cx_local)
            text.pose.position.y = float(cy_local)
            text.pose.position.z = float(cz_local + (size_z * 0.5) + 1.6)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.9
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.lifetime.nanosec = 250_000_000
            text.text = (
                f'ID:{tid}\n'
                f'{motion_label}\n'
                f'{speed:.2f} m/s'
            )
            markers.append(text)

            marker_id += 1

        self.obstacle_pub.publish(Float32MultiArray(data=detections))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _local_to_global(self, lx: float, ly: float) -> Tuple[float, float]:
        cos_y = math.cos(self.ego_yaw)
        sin_y = math.sin(self.ego_yaw)
        gx = self.ego_x + lx * cos_y - ly * sin_y
        gy = self.ego_y + lx * sin_y + ly * cos_y
        return gx, gy

    def _majority_valid_class(self, cls_ids: np.ndarray) -> int:
        valid = cls_ids[cls_ids >= 0]
        if valid.size == 0:
            return -1
        vals, counts = np.unique(valid, return_counts=True)
        return int(vals[np.argmax(counts)])

    def _type_id_from_yolo_class(self, cls_id: int) -> int:
        if cls_id == 0:
            return 1
        if cls_id in [2, 3, 5, 7]:
            return 2
        return 0

    def _type_id_from_size(self, size_x: float, size_y: float, size_z: float) -> int:
        if size_x <= 1.2 and size_y <= 1.2 and 0.8 <= size_z <= 2.5:
            return 1
        return 2


def main(args=None):
    rclpy.init(args=args)
    node = LidarPerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()