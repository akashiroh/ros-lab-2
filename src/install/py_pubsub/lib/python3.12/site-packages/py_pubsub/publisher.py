# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math

import matplotlib.pyplot as plt
import numpy as np

def lowpass_filter(Y: float, y: float, fc=0.5, dt=10.0):
    '''
        Y(t+dt) = β Y(t) + (1-β)y(t)
        β = (2π*fc*dt)/(2π*dt*fc + 1)
    '''
    beta = 2*math.pi*fc*dt / (2*math.pi*fc*dt + 1)
    return beta * Y + (1 - beta) * y



class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_publisher')

        # publishers and subscribers
        self.publisher = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/diff_drive/scan', self.listener_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/diff_drive/odometry', self.odometry_callback, 10)

        self.state = "INITIAL_STATE"
        
        # rotation
        self.target_angle = math.pi / 2.4
        self.current_yaw = 0.
        self.last_yaw = None
        self.start_yaw = None
        self.accumulated_yaw = 0.

        # lowpass filter
        self.Y = 0.

        self.noise, self.filtered = [], []

    def get_yaw_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def odometry_callback(self, msg: Odometry):
        current_orientation = msg.pose.pose.orientation
        new_yaw = self.get_yaw_from_quaternion(current_orientation)

        self.current_yaw = new_yaw

        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            self.last_yaw = self.current_yaw
            self.accumulated_yaw = 0.0
            return

        d_yaw = self.last_yaw - self.current_yaw
        self.accumulated_yaw += d_yaw
        self.last_yaw = self.current_yaw

        if abs(self.accumulated_yaw) >= abs(self.target_angle):
            self.stop_rotation()

    def listener_callback(self, msg: LaserScan):
        front_range = msg.ranges[0]

        # initialize Y(t)
        if self.Y == 0:
            self.Y = front_range

        new_y = lowpass_filter(self.Y, front_range)
        self.Y = new_y
        
        self.noise.append(front_range)
        self.filtered.append(new_y)

        # front_range = new_y

        dist_from_wall_thresh = 2

        if self.state == "INITIAL_STATE":
            if front_range < dist_from_wall_thresh:
                self.state = "ROTATING"
                self.start_yaw = None
                self.get_logger().info(f"Starting Rotating")
                self.rotate_right()
            elif front_range > dist_from_wall_thresh:
                self.state = "MOVING"
                self.get_logger().info("MOVING")
                self.move()
        elif self.state == "MOVING":
            if front_range < dist_from_wall_thresh:
                self.state = "ROTATING"
                self.start_yaw = None
                self.get_logger().info(f"Starting Rotating")
                self.rotate_right()
            elif front_range > dist_from_wall_thresh:
                self.get_logger().info(f"Distance to wall: {front_range:.4f}")
                self.move()
        elif self.state == "ROTATING":
            ...
        else:
            self.get_logger().info(f"UNKOWN STATE.")

            fig, axes = plt.subplots(2, 1, sharex=True, sharey=True, figsize=(5,5))
            axes[0].plot(np.arange(len(self.noise)), self.noise)
            axes[0].set_ylabel("Distance")
            axes[0].set_title("Noisy Lidar Sensor Readings")
            axes[1].plot(np.arange(len(self.filtered)), self.filtered)
            axes[1].set_ylabel("Distance")
            axes[1].set_xlabel("Timesteps")
            axes[1].set_title("Filtered Lidar Sensor Readings")
            plt.savefig("test.png")
            breakpoint()



    def move(self):
        msg = Twist()
        msg.linear.x = 50.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def rotate_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.57
        self.publisher.publish(msg)

    def rotate_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -1.57
        self.publisher.publish(msg)

    def stop_rotation(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)

        self.state = "MOVING"
        # self.state = "TESTING"
        self.get_logger().info(f"Rotation complete")

        self.start_yaw = None
        self.last_yaw = None
        self.accumulated_yaw = 0.0


def main(args=None):
    rclpy.init(args=args)

    publisher = DiffDriveController()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
