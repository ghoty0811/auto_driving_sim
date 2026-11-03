#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, Gilbert, Ashe Kim, Jun

from enum import Enum
import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, UInt8MultiArray, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ultralytics import YOLO
import math
import time

def euler_from_quaternion(msg):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).

    msg: geometry_msgs.msg.Quaternion.
    return: (roll, pitch, yaw) tuple.
    """
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class DetectSign(Node):

    def __init__(self):
        super().__init__('detect_sign')
        self.TrafficSign = Enum('TrafficSign', 'intersection left right')
        self.dictSign = {
            "intersection" : 0,
            "left" : 1,
            "right" : 2,
            "stop" : 3,
            "tunnel" : 4,
            "parking" : 5,
            "construction" : 6
        }
        self.ready2park = False
        self.is_parking = False
        self.bridge = CvBridge()
        self.lane_detected = False

        # Parameter settings
        self.danger_distance = 0.24    # Danger zone y threshold (meters)
        self.danger_width = 0.12       # Danger zone x width (meters)
        self.speed = 0.03              # Forward speed during avoidance

        # PD control parameters (for turning)
        self.turn_Kp = 0.45
        self.turn_Kd = 0.03
        self.turn_threshold = 0.05     # Tolerance to decide when turning is complete
        self.last_turn_error = 0.0

        # State machine variables
        # States: 'NORMAL', 'AVOID_TURN', 'AVOID_STRAIGHT', 'RETURN_TURN'
        self.state = 'NORMAL'
        self.desired_theta = None      # Target angle (radians)
        self.original_theta = None     # Original heading at the start of avoidance mode

        # Current odom information
        self.current_theta = 0.0
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0

        self.sub_model = self.create_subscription(UInt8MultiArray, '/detect/model_results', self.cbFindTrafficSign, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.lane_state_sub = self.create_subscription(
            UInt8,
            '/detect/lane_state',
            self.lane_state_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_projected',
            self.image_callback,
            10
        )
        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        self.avoid_cmd_pub = self.create_publisher(
            Twist,
            '/avoid_control',
            10
        )
        self.avoid_active_pub = self.create_publisher(
            Bool,
            '/avoid_active',
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            '/lane_detection/image_result',
            10
        )

        self.timer = None
        self.get_logger().info('DetectParkingSign Node Initialized')

    def cbFindTrafficSign(self, msg_results):
        is_founded = msg_results.data
        if is_founded[self.dictSign["parking"]] == 1:
            msg_sign = UInt8()
            msg_sign.data = self.dictSign["parking"]
            self.get_logger().info("Find Parking")
            self.pub_traffic_sign.publish(msg_sign)
            self.ready2park = True
        if is_founded[self.dictSign["stop"]] == 1 and not self.is_parking and self.ready2park:
            self.is_parking = True
            self.get_logger().info("Timer Start")
            self.timer = self.create_timer(0.1, self.process_loop)


    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        euler = euler_from_quaternion(q)
        self.current_theta = euler[2]
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y

    def lane_state_callback(self, msg):
        # self.get_logger().info(f"line state change : {msg.data}")
        self.lane_state = msg.data

    def image_callback(self, msg):
        if self.state == 'NORMAL':
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            self.get_logger().error('Image conversion failed')
            return

        img_height, img_width = cv_image.shape[:2]

        # Define ROI
        roi_size = img_height
        roi_x_start = (img_width - roi_size) // 2
        roi_x_end = roi_x_start + roi_size
        roi = cv_image[0:roi_size, roi_x_start:roi_x_end]

        margin = 20

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi/180, threshold=50, minLineLength=100, maxLineGap=10
        )

        positive_found = False
        negative_found = False

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    # Calculate line length
                    line_length = np.hypot(x2 - x1, y2 - y1)
                    if line_length < 100:
                        continue
                    # Check if the line covers near the top and bottom of the ROI
                    if max(y1, y2) < margin or min(y1, y2) > (img_height - margin):
                        continue

                    if (x2 - x1) == 0:
                        continue
                    slope = (y2 - y1) / (x2 - x1)
                    if slope > 0:
                        positive_found = True
                    elif slope < 0:
                        negative_found = True
            if positive_found and negative_found:
                self.lane_detected = True
            else:
                self.lane_detected = False
        else:
            self.lane_detected = False

        overlay_text = 'Lane Detected' if self.lane_detected else 'Lane Not Detected'
        text_color = (0, 255, 0) if self.lane_detected else (0, 0, 255)
        cv2.putText(
            cv_image, overlay_text, (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 2
        )
        
        # self.get_logger().info(overlay_text)
        cv_image = cv2.resize(cv_image, (200, 120), interpolation=cv2.INTER_LINEAR)   # image resize to print rqt
        result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(result_msg)

    def process_loop(self):
        if self.state == 'NORMAL':
            self.process_normal_state()
        elif self.state == 'AVOID_TURN':
            self.process_avoid_turn_state()
        elif self.state == 'AVOID_STRAIGHT':
            self.process_avoid_straight_state()

    def process_normal_state(self):
        self.get_logger().info('Stop sign detected.')
        self.original_theta = self.current_theta
        self.desired_theta = self.normalize_angle(
            self.current_theta + math.radians(180)
        )
        self.state = 'AVOID_STRAIGHT'

    def process_avoid_turn_state(self):
        # self.get_logger().info('Turn Start')
        error = self.normalize_angle(self.desired_theta - self.current_theta)
        angular_z = self.turn_Kp * error + self.turn_Kd * (error - self.last_turn_error)
        self.last_turn_error = error
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.avoid_cmd_pub.publish(twist)
        self.publish_active(True)
        if abs(error) < self.turn_threshold:
            self.get_logger().info('Avoidance turn completed. Destroy Timer.')
            self.last_turn_error = 0.0
            self.state = 'NORMAL'
            self.publish_active(False)
            self.is_parking = False
            self.ready2park = False
            self.destroy_timer(self.timer)
            self.timer = None

            msg_sign = UInt8()
            msg_sign.data = self.dictSign["left"]
            self.pub_traffic_sign.publish(msg_sign)

    def process_avoid_straight_state(self):
        # self.get_logger().info('Detect Lane Start!')
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = 0.0
        self.avoid_cmd_pub.publish(twist)
        self.publish_active(True)
        if self.lane_detected:
            self.get_logger().info('Lane detected. Starting return turn.')
            self.state = 'AVOID_TURN'

    def publish_active(self, active: bool):
        bool_msg = Bool()
        bool_msg.data = active
        self.avoid_active_pub.publish(bool_msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
