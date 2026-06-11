# Copyright 2023 Josh Newans
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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time


class FollowPerson(Node):

    def __init__(self):
        super().__init__('follow_person')
        self.declare_parameter("detected_person_topic", "/person_tracker/detected_person")
        self.declare_parameter("cmd_vel_topic", "/person_tracker/cmd_vel")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("target_timeout_secs", 0.5)
        self.declare_parameter("angular_gain", 0.5)
        self.declare_parameter("forward_speed", 0.12)
        self.declare_parameter("max_linear_velocity", 0.15)
        self.declare_parameter("max_angular_velocity", 0.35)
        self.declare_parameter("max_target_size", 0.55)
        self.declare_parameter("forward_error_limit", 0.35)
        self.declare_parameter("center_deadband", 0.08)
        self.declare_parameter("filter_value", 0.8)
        self.declare_parameter("control_rate_hz", 10.0)

        detected_person_topic = self.get_parameter("detected_person_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.target_timeout_secs = float(self.get_parameter("target_timeout_secs").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.max_target_size = float(self.get_parameter("max_target_size").value)
        self.forward_error_limit = float(self.get_parameter("forward_error_limit").value)
        self.center_deadband = float(self.get_parameter("center_deadband").value)
        self.filter_value = float(self.get_parameter("filter_value").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.subscription = self.create_subscription(
            Point,
            detected_person_topic,
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)

        timer_period = 1.0 / max(control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_x = 0.0
        self.target_size = 0.0
        self.last_receive_time = time.monotonic() - 10000
        self.last_lost_log_time = 0.0

        self.get_logger().info(f"Following target from {detected_person_topic}")
        if self.enable_motion:
            self.get_logger().warning(f"Motion enabled, publishing velocity commands to {cmd_vel_topic}")
        else:
            self.get_logger().warning("Motion disabled, no velocity commands will be published")

    def timer_callback(self):
        if not self.enable_motion:
            return

        msg = Twist()
        target_age = time.monotonic() - self.last_receive_time

        if target_age > self.target_timeout_secs:
            now = time.monotonic()
            if now - self.last_lost_log_time > 2.0:
                self.get_logger().warning("Target lost, publishing zero velocity")
                self.last_lost_log_time = now
            self.publisher_.publish(msg)
            return

        turn_error = 0.0 if abs(self.target_x) < self.center_deadband else self.target_x
        msg.angular.z = self.clamp(
            -self.angular_gain * turn_error,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )

        person_is_far = self.target_size < self.max_target_size
        person_is_forward = abs(self.target_x) < self.forward_error_limit
        if person_is_far and person_is_forward:
            msg.linear.x = self.clamp(
                self.forward_speed,
                0.0,
                self.max_linear_velocity,
            )

        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_x = self.target_x * f + msg.x * (1-f)
        self.target_size = self.target_size * f + msg.z * (1-f)
        self.last_receive_time = time.monotonic()

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)
    follow_person = FollowPerson()
    rclpy.spin(follow_person)
    follow_person.destroy_node()
    rclpy.shutdown()
