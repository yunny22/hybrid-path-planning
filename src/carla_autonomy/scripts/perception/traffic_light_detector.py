import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from pathlib import Path
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import cv2
from ultralytics import YOLO


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        self.bridge = CvBridge()
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').value or self._default_model_path()
        self.get_logger().info(f'커스텀 YOLOv8 모델 로딩 중: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('모델 로드 완료. 신호등 탐지를 시작합니다.')

        self.image_sub = self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_traffic_light/image',
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/carla/ego_vehicle/traffic_light_recognition/image',
            10
        )
        self.state_pub = self.create_publisher(String, '/perception/traffic_light', 10)

        self.stable_state = 'UNKNOWN'
        self.candidate_state = 'UNKNOWN'
        self.candidate_count = 0
        self.last_valid_seen_time = self.get_clock().now()

        self.red_confirm_count = 3
        self.yellow_confirm_count = 3
        self.green_confirm_count = 2
        self.red_conf_thresh = 0.35
        self.yellow_conf_thresh = 0.35
        self.green_conf_thresh = 0.28
        self.lost_timeout_sec = 0.35

    def _default_model_path(self):
        try:
            package_share = Path(get_package_share_directory('carla_autonomy'))
            for candidate in (package_share / 'models' / 'best.pt', package_share / 'best.pt'):
                if candidate.exists():
                    return str(candidate)
            return str(package_share / 'models' / 'best.pt')
        except PackageNotFoundError:
            return str(Path(__file__).resolve().parents[2] / 'models' / 'best.pt')

    def _pick_best_detection(self, results):
        best_state = 'UNKNOWN'
        best_conf = 0.0
        best_box = None

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            class_id = int(box.cls[0])

            if class_id == 0:
                state = 'GREEN'
                thresh = self.green_conf_thresh
            elif class_id == 1:
                state = 'RED'
                thresh = self.red_conf_thresh
            elif class_id == 2:
                state = 'YELLOW'
                thresh = self.yellow_conf_thresh
            else:
                continue

            if conf >= thresh and conf > best_conf:
                best_state = state
                best_conf = conf
                best_box = (x1, y1, x2, y2)

        return best_state, best_conf, best_box

    def _update_stable_state(self, observed_state):
        now = self.get_clock().now()

        if observed_state != 'UNKNOWN':
            self.last_valid_seen_time = now
            if observed_state == self.candidate_state:
                self.candidate_count += 1
            else:
                self.candidate_state = observed_state
                self.candidate_count = 1

            required = {
                'RED': self.red_confirm_count,
                'YELLOW': self.yellow_confirm_count,
                'GREEN': self.green_confirm_count,
            }[observed_state]

            if self.candidate_count >= required:
                self.stable_state = observed_state
        else:
            dt = (now - self.last_valid_seen_time).nanoseconds * 1e-9
            if dt > self.lost_timeout_sec:
                self.stable_state = 'UNKNOWN'
                self.candidate_state = 'UNKNOWN'
                self.candidate_count = 0

        return self.stable_state

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        img_h, img_w = cv_image.shape[:2]

        roi_x_min = int(img_w * 0.30)
        roi_x_max = int(img_w * 0.70)
        roi_y_min = int(img_h * 0.00)
        roi_y_max = int(img_h * 0.50)

        cv2.rectangle(cv_image, (roi_x_min, roi_y_min), (roi_x_max, roi_y_max), (255, 0, 0), 2)

        roi_img = cv_image[roi_y_min:roi_y_max, roi_x_min:roi_x_max]
        if roi_img.shape[0] == 0 or roi_img.shape[1] == 0:
            return

        results = self.model(roi_img, verbose=False)
        observed_state, observed_conf, observed_box = self._pick_best_detection(results)
        final_state = self._update_stable_state(observed_state)

        if observed_box is not None:
            rx1, ry1, rx2, ry2 = observed_box
            x1 = rx1 + roi_x_min
            y1 = ry1 + roi_y_min
            x2 = rx2 + roi_x_min
            y2 = ry2 + roi_y_min

            if observed_state == 'GREEN':
                color_bgr = (0, 255, 0)
            elif observed_state == 'RED':
                color_bgr = (0, 0, 255)
            else:
                color_bgr = (0, 255, 255)

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color_bgr, 3)
            label = f'[obs={observed_state} {observed_conf:.2f}] [pub={final_state}]'
            cv2.putText(cv_image, label, (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
        else:
            cv2.putText(
                cv_image,
                f'[obs=UNKNOWN] [pub={final_state}]',
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

        msg_str = String()
        msg_str.data = final_state
        self.state_pub.publish(msg_str)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'발행 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
