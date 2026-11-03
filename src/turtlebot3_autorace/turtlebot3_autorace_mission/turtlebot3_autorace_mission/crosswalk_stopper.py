import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool

class CrosswalkStopper(Node):
    def __init__(self):
        super().__init__('crosswalk_stopper')

        self.bridge = CvBridge()
        self.traffic_state = 0  # 255: red, 128: orange, 0: green
        self.stop_sent = False

        # Subscribe to traffic light state
        self.create_subscription(UInt8, '/detect/traffic_light', self.traffic_callback, 10)
        # Subscribe to camera image
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)

        self.stop_flag_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        self.get_logger().info('CrosswalkStopper Node Started')

    def traffic_callback(self, msg):
        self.traffic_state = msg.data

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                return
        except Exception as e:
            self.get_logger().error(f"Decode error: {e}")
            return

        if self.traffic_state in [255]: 
            if self.detect_crosswalk(cv_image):
                self.get_logger().info("✅ Crosswalk Detected")
                if not self.stop_sent:
                    self.get_logger().info("🛑 🛑 \U0001F6D1 Red/Orange Light + Crosswalk Detected. Emergency Stop!")
                    self.stop_sent = True
                    self.stop_flag_pub.publish(Bool(data=True))
            else:
                self.stop_sent = False
        else:
            self.get_logger().info("🟢 Green light. Resume moving.")
            self.get_logger().info(" ")
            self.stop_sent = False
            self.stop_flag_pub.publish(Bool(data=False))

    def detect_crosswalk(self, image):
        h, w, _ = image.shape
        roi = image[int(h*11/14):, :]  # Bottom third

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 170])
        upper_white = np.array([180, 100, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        opened = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count_large = 0
        for c in contours:
            area = cv2.contourArea(c)
            x, y, w_, h_ = cv2.boundingRect(c)
            if area > 50 and w_ > 8 :
                count_large += 1
        return count_large >= 2

def main(args=None):
    rclpy.init(args=args)
    node = CrosswalkStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
