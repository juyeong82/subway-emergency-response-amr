import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os
from datetime import datetime

class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector_node')
        
        # QoS 설정을 sensor_data(Best Effort)로 변경하여 전송 지연 완화 시도
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot5/oakd/rgb/image_raw/compressed',
            self.listener_callback,
            qos_profile
        )
        
        self.save_dir = 'captured_images'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.get_logger().info(f"Save directory: {os.path.abspath(self.save_dir)}")

    def listener_callback(self, msg):
        try:
            # 1. 데이터 길이 체크 (핵심: 빈 데이터 방어)
            if len(msg.data) == 0:
                self.get_logger().warn("Received empty image data. Skipping...")
                return

            # 2. cv_bridge 대신 직접 numpy로 디코딩 (더 안정적)
            # 바이트 데이터를 numpy 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # 이미지 디코딩
            cv_image = cv2.resize(np_arr, (640, 480))
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 3. 디코딩 실패 체크
            if cv_image is None:
                self.get_logger().warn("Failed to decode image. Format might be wrong.")
                return
            
            # 화면 출력
            cv2.imshow("YOLO Data Collector", cv_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.save_image(cv_image)
            elif key == ord('q'):
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def save_image(self, image):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{self.save_dir}/img_{timestamp}.jpg"
        cv2.imwrite(filename, image)
        self.get_logger().info(f"Saved: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()