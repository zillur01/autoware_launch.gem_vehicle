#!/usr/bin/env python3
# Copyright 2026 Zillur Rahman
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
"""
Publish /joint_states for the GEM e4 steering hinges and wheels.

Inputs (Autoware vehicle status):
  /vehicle/status/steering_status  autoware_vehicle_msgs/SteeringReport (steering_tire_angle [rad])
  /vehicle/status/velocity_status  autoware_vehicle_msgs/VelocityReport (longitudinal_velocity [m/s])

The bicycle-model tire angle is split into left/right hinge angles with Ackermann geometry, and the
wheel angles are integrated from the longitudinal velocity. Without any input the joints are
published at zero, so the wheels are always visible in RViz.
"""

import math

from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class GemJointStatePublisher(Node):
    def __init__(self):
        super().__init__('gem_joint_state_publisher')
        # Geometry of gem_vehicle_description/urdf/vehicle.xacro
        self.wheel_base = self.declare_parameter('wheel_base', 2.565).value
        self.steering_track = self.declare_parameter('steering_track', 1.308).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.295).value
        self.max_hinge_angle = self.declare_parameter('max_hinge_angle', 0.61).value
        rate = self.declare_parameter('publish_rate', 30.0).value

        self.tire_angle = 0.0
        self.velocity = 0.0
        self.wheel_angle = 0.0
        self.last_time = None

        self.create_subscription(
            SteeringReport, '/vehicle/status/steering_status', self.on_steering,
            qos_profile_sensor_data)
        self.create_subscription(
            VelocityReport, '/vehicle/status/velocity_status', self.on_velocity,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self.on_timer)

    def on_steering(self, msg):
        self.tire_angle = msg.steering_tire_angle

    def on_velocity(self, msg):
        self.velocity = msg.longitudinal_velocity

    def hinge_angles(self):
        delta = self.tire_angle
        if abs(delta) < 1e-6:
            return 0.0, 0.0
        radius = self.wheel_base / math.tan(delta)  # signed turning radius at the rear axle center
        half = self.steering_track / 2.0
        left = math.atan(self.wheel_base / (radius - half))
        right = math.atan(self.wheel_base / (radius + half))
        lim = self.max_hinge_angle
        return max(-lim, min(lim, left)), max(-lim, min(lim, right))

    def on_timer(self):
        now = self.get_clock().now()
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if 0.0 < dt < 1.0:  # ignore clock jumps (e.g. rosbag loop)
                self.wheel_angle = math.remainder(
                    self.wheel_angle + self.velocity / self.wheel_radius * dt, 2.0 * math.pi)
        self.last_time = now

        left, right = self.hinge_angles()
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [
            'left_steering_hinge_joint', 'right_steering_hinge_joint',
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint',
        ]
        msg.position = [left, right] + [self.wheel_angle] * 4
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GemJointStatePublisher()
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
