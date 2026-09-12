import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import collections # 시계열 필터링을 위한 큐 모듈 추가

class StopLineDetector(Node):
    def __init__(self):
        super().__init__('stop_line_detector')
        
        # CARLA 전방 카메라 구독
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_stop_line/image',
            self.image_callback,
            10)
            
        # 정지선 발견 여부를 퍼블리시
        self.stop_line_pub = self.create_publisher(Bool, '/perception/stop_line', 10)
        
        # 인식 결과 시각화 퍼블리셔
        self.debug_image_pub = self.create_publisher(Image, '/perception/stop_line_debug/image', 10)

        self.bridge = CvBridge()
        
        self.declare_parameter('history_size', 9)
        self.declare_parameter('detection_threshold', 4)
        self.declare_parameter('roi_top_ratio', 0.38)
        self.declare_parameter('roi_bottom_ratio', 0.98)
        self.declare_parameter('white_value_min', 180)
        self.declare_parameter('white_sat_max', 60)
        self.declare_parameter('hough_threshold', 45)
        self.declare_parameter('min_line_length_ratio', 0.22)
        self.declare_parameter('max_line_gap', 28)
        self.declare_parameter('max_abs_angle_deg', 12.0)
        self.declare_parameter('white_row_ratio_threshold', 0.12)
        self.declare_parameter('white_component_threshold', 220)

        history_size = int(self.get_parameter('history_size').value)
        self.detection_threshold = int(self.get_parameter('detection_threshold').value)
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self.roi_bottom_ratio = float(self.get_parameter('roi_bottom_ratio').value)
        self.white_value_min = int(self.get_parameter('white_value_min').value)
        self.white_sat_max = int(self.get_parameter('white_sat_max').value)
        self.hough_threshold = int(self.get_parameter('hough_threshold').value)
        self.min_line_length_ratio = float(self.get_parameter('min_line_length_ratio').value)
        self.max_line_gap = int(self.get_parameter('max_line_gap').value)
        self.max_abs_angle_deg = float(self.get_parameter('max_abs_angle_deg').value)
        self.white_row_ratio_threshold = float(self.get_parameter('white_row_ratio_threshold').value)
        self.white_component_threshold = int(self.get_parameter('white_component_threshold').value)

        self.detection_history = collections.deque(maxlen=history_size)

        self.get_logger().info(
            f"정지선 인식 노드 시작: {self.detection_threshold}/{history_size} 프레임 확정"
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            return

        height, width = cv_image.shape[:2]

        roi_top = int(height * self.roi_top_ratio)
        roi_bottom = int(height * self.roi_bottom_ratio)
        roi_image = cv_image[roi_top:roi_bottom, 0:width]
        if roi_image.size == 0:
            return

        # 2. 하얀색 추출
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, self.white_value_min])
        upper_white = np.array([180, self.white_sat_max, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.GaussianBlur(white_mask, (5, 5), 0)

        # 3. 엣지 추출
        edges = cv2.Canny(white_mask, 40, 120)

        min_line_length = max(80, int(width * self.min_line_length_ratio))
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=min_line_length,
            maxLineGap=self.max_line_gap,
        )

        raw_detected = False # 현재 프레임에서의 단순 인식 결과
        raw_count = 0

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
                length = math.hypot(x2 - x1, y2 - y1)
                global_y = roi_top + 0.5 * (y1 + y2)
                crosses_center = min(x1, x2) < width * 0.60 and max(x1, x2) > width * 0.40
                in_distance_band = height * 0.42 <= global_y <= height * 0.98
                
                if (
                    (abs(angle) < self.max_abs_angle_deg or abs(angle) > 180.0 - self.max_abs_angle_deg) and
                    length >= min_line_length and
                    crosses_center and
                    in_distance_band
                ):
                    raw_count += 1
                    raw_detected = True
                    cv2.line(cv_image, (x1, y1 + roi_top), (x2, y2 + roi_top), (0, 0, 255), 5)
                    break 

        if not raw_detected:
            white_row_counts = np.count_nonzero(white_mask > 0, axis=1)
            white_rows = np.where(white_row_counts >= int(width * self.white_row_ratio_threshold))[0]
            if white_rows.size > 0:
                connected_pixels = int(np.count_nonzero(white_mask > 0))
                if connected_pixels >= self.white_component_threshold:
                    raw_detected = True
                    raw_count += 1
                    y_idx = int(np.median(white_rows))
                    cv2.line(cv_image, (0, roi_top + y_idx), (width - 1, roi_top + y_idx), (0, 0, 255), 4)

        # 🌟 6. 시계열 필터 적용: 현재 프레임 결과를 큐에 넣고 확정 판정
        self.detection_history.append(raw_detected)
        
        final_stop_line_detected = sum(self.detection_history) >= self.detection_threshold

        # 디버깅용 텍스트 출력 (시각화 화면 좌측 상단)
        status_text = (
            f"Stop Line: {'DETECTED' if final_stop_line_detected else 'None'} "
            f"({sum(self.detection_history)}/{self.detection_history.maxlen}, raw={raw_count})"
        )
        color = (0, 255, 0) if final_stop_line_detected else (0, 0, 255)
        cv2.putText(cv_image, status_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        # 결과 퍼블리시
        msg_bool = Bool()
        msg_bool.data = bool(final_stop_line_detected)
        self.stop_line_pub.publish(msg_bool)

        # 시각화 화면 퍼블리시
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            pass
            
def main(args=None):
    rclpy.init(args=args)
    detector = StopLineDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
