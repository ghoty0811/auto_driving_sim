from enum import Enum
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, UInt8MultiArray

from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class DetectSign(Node):

    def __init__(self):
        super().__init__('detect_sign')

        self.sub_image_type = 'raw'  # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/camera/image_compensated/compressed',
                self.cbFindTrafficSign,
                10
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                '/camera/image_compensated',
                self.cbFindTrafficSign,
                10
            )

        self.pub_model_results = self.create_publisher(UInt8MultiArray, '/detect/model_results', 10)
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage,
                '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        self.cvBridge = CvBridge()
        self.dictSign = {
            "intersection" : 0,
            "left" : 1,
            "right" : 2,
            "stop" : 3,
            "tunnel" : 4,
            "parking" : 5,
            "construction" : 6
        }
        self.counter = 1

        default_model_path = os.path.join(
            get_package_share_directory('turtlebot3_autorace_detect'),
            'resource', 'best.pt'
        )
        self.declare_parameter('model_path', default_model_path)
        self.model = YOLO(self.get_parameter('model_path').value)

        self.get_logger().info('DetectSign Node Initialized')
        
        self.pub_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)


    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed.
        # This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            # converting compressed image to opencv image
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        is_founded = [0, 0, 0, 0, 0, 0, 0]
        results = self.model(cv_image_input, verbose=False)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = box.conf[0].item()
            area = abs(x2 - x1) * abs(y2 - y1)

            # Draw bounding box and label
            if conf < 0.7 or area < 500:
                continue
            self.get_logger().info(f"{label} area : {area}")

            is_founded[self.dictSign[label]] = 1
            
            if label == "tunnel" and conf > 0.7 and area > 500:
                self.pub_sign.publish(UInt8(data=4))

            cv2.rectangle(cv_image_input, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                cv_image_input, f'{label} {conf:.2f}',
                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2
            )
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(
                    cv_image_input, 'jpg'
                )
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_imgmsg(
                    cv_image_input, 'bgr8'
                )
            )

        msg_results = UInt8MultiArray()
        msg_results.data = is_founded
        self.pub_model_results.publish(msg_results)
        # self.get_logger().info(f"is_founded : {is_founded}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
