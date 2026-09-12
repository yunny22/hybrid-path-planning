#!/usr/bin/env python3
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloPerception(Node):
    def __init__(self):
        super().__init__('yolo_perception')

        self.declare_parameter('role_name', 'ego_vehicle')
        role_name = self.get_parameter('role_name').value

        # 1. CARLA 실제 카메라 토픽 이름
        self.camera_topic = f'/carla/{role_name}/rgb_traffic_light/image'
        
        # 2. 라이다 노드로 보내줄 2D 바운딩 박스 데이터 토픽
        self.bbox_topic = '/yolo_detections'
        
        # 3. 디버깅용 (박스 쳐진 화면) 퍼블리셔
        self.debug_image_topic = f'/carla/{role_name}/yolo_perception/image'

        self.bridge = CvBridge()
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').value or str(Path(__file__).with_name('yolov8n.pt'))

        try:
            self.get_logger().info(f'보행자/차량 인식용 YOLOv8n 모델 로딩 중: {model_path}')
            self.model = YOLO(model_path)
            self.get_logger().info('YOLOv8n 로딩 완료!')
        except Exception as exc:
            self.get_logger().warn(f'YOLO 모델 로딩 실패: {exc}')
            self.model = None

        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_cb, qos_profile_sensor_data)
        self.bbox_pub = self.create_publisher(Float32MultiArray, self.bbox_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Debug image topic: {self.debug_image_topic}')

    def image_cb(self, msg):
        # 1. ROS 이미지를 OpenCV 이미지로 변환
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'이미지 변환 실패: {exc}')
            return

        detections = []
        annotated_frame = cv_image
        
        if self.model is not None:
            # 2. YOLOv8 추론 실행 (verbose=False로 터미널 도배 방지)
            results = self.model(cv_image, verbose=False)

            # 3. 결과 파싱
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # 클래스 ID (0: 사람, 2: 자동차, 3: 오토바이, 5: 버스, 7: 트럭 등)
                    cls_id = int(box.cls[0])
                    
                    # 자율주행에서 피해야 할 주요 동적 장애물만 필터링
                    if cls_id in [0, 2, 3, 5, 7]: 
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        detections.extend([float(cls_id), x1, y1, x2, y2])

            annotated_frame = results[0].plot()
        
        # 4. 데이터 전송 (라이다 노드가 이 데이터를 받아서 퓨전하게 됩니다)
        self.bbox_pub.publish(Float32MultiArray(data=detections))

        # 5. [선택] 디버깅용 이미지 전송 (박스 그려진 결과물)
        debug_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
