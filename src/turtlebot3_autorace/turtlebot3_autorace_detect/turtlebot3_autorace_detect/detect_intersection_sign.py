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
from std_msgs.msg import UInt8, UInt8MultiArray

from ultralytics import YOLO

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

        self.sub_model = self.create_subscription(UInt8MultiArray, '/detect/model_results', self.cbFindTrafficSign, 10)
        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        self.get_logger().info('DetectIntersectionSign Node Initialized')

    def cbFindTrafficSign(self, msg_results):
        is_founded = msg_results.data

        if is_founded[self.dictSign["intersection"]]:
            msg_sign = UInt8()
            msg_sign.data = self.dictSign["intersection"]
            self.get_logger().info("Find Intersection")
            self.pub_traffic_sign.publish(msg_sign)
        if is_founded[self.dictSign["left"]]:
            msg_sign = UInt8()
            msg_sign.data = self.dictSign["left"]
            self.get_logger().info("Find Left")
            self.pub_traffic_sign.publish(msg_sign)
        if is_founded[self.dictSign["right"]]:
            msg_sign = UInt8()
            msg_sign.data = self.dictSign["right"]
            self.get_logger().info("Find Right")
            self.pub_traffic_sign.publish(msg_sign)

def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
