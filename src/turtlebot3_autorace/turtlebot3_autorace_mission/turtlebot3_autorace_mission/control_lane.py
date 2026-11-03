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
# Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8
from std_msgs.msg import Float64

class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )
        self.create_subscription(
            UInt8,
            '/detect/traffic_sign',
            self.callback_detect_sign,
            10
        )
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        # PD control related variables
        self.last_error = 0
        self.MAX_VEL = 0.05

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()
        
        self.emergency_stop = False
        
        self.create_subscription(Bool, '/emergency_stop', self.stop_callback, 10)
        
        
        # 터널 FSM 제어
        self.tunnel_mode = False
        self.tunnel_sign_detected = False
        self.cooldown = False
        self.tunnel_timer = None
        self.timer_duration_sec = 20.0 # 터널 입구 표지판 인식 후 20초간 표지판 무시

        # 표지판 감지 무시 상태 (출구 직후)
        self.ignore_sign = False
        self.sign_ignore_timer = None
        self.sign_ignore_duration = 10.0  # 터널 출구 표지판 인식 후 10초간 표지판 무시
        
        self.last_center = Float64()
        self.last_center.data = 500.0


    def callback_get_max_vel(self, max_vel_msg):
      self.MAX_VEL = max_vel_msg.data
        
    def callback_detect_sign(self, msg):
        if self.ignore_sign:
            # self.get_logger().info("🚫 표지판 감지 무시 중 (출구 직후)")
            return

        if msg.data == 4:  # tunnel
            if not self.tunnel_sign_detected:
                self.get_logger().info("🟡 터널 감속 시작")
                self.tunnel_sign_detected = True
                self.tunnel_mode = True
                self.cooldown = False

                if self.tunnel_timer:
                    self.tunnel_timer.cancel()
                self.tunnel_timer = self.create_timer(
                    self.timer_duration_sec,
                    self.exit_tunnel_timer
                )
            elif not self.cooldown:
                self.get_logger().info("⛔ 입구 중복 감지 – 타이머 만료 대기 중")
            elif self.cooldown:
                self.get_logger().info("🟢 터널 종료 감지 – 감속 해제")
                self.tunnel_mode = False
                self.tunnel_sign_detected = False
                self.cooldown = False

                if self.tunnel_timer:
                    self.tunnel_timer.cancel()
                    self.tunnel_timer = None

                self.ignore_sign = True
                if self.sign_ignore_timer:
                    self.sign_ignore_timer.cancel()
                self.sign_ignore_timer = self.create_timer(
                    self.sign_ignore_duration,
                    self.reset_sign_ignore_flag
                )

    def exit_tunnel_timer(self):
        self.cooldown = True
        self.get_logger().info("⏲ 타이머 만료 – 출구 표지판 감지 대기 중")

    def reset_sign_ignore_flag(self):
        self.ignore_sign = False
        self.get_logger().info("✅ 표지판 감지 재활성화")

    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.

        If avoidance mode is enabled, lane following control is ignored.
        """
        self.last_center = desired_center
        
        if self.avoid_active:
            return
        
        if self.emergency_stop:
            # self.get_logger().info("🛑 🛑 Emergency Stop – publish zero & return")
            self.pub_cmd_vel.publish(Twist())
            return 

        center = desired_center.data
        error = center - 500
        Kp, Kd = 0.0025, 0.007
        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        twist = Twist()
        
        if self.tunnel_mode:
            twist.linear.x = 0.02
        else:
            twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.1)

        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        
        self.pub_cmd_vel.publish(twist)


    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        # if self.avoid_active:
        #     self.get_logger().info('🚨 회피 모드 ON.')
        # else:
        #     self.get_logger().info('🔁🔁 회피 종료 – 차선 추종 복귀.')

    def shut_down(self):
        # self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)
        
    def stop_callback(self, msg):
        prev_state = self.emergency_stop
        self.emergency_stop = msg.data

        if prev_state and not self.emergency_stop:
            # self.get_logger().info("🟢 Emergency Stop 해제됨 – 다시 출발 시도")
            self.callback_follow_lane(self.last_center)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()