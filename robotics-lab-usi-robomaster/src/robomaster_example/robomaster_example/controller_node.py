import rclpy
import rclpy.logging
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys
import math


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        # Open-loop figure-eight parameters
        self.linear_speed = 0.3  # [m/s]
        self.angular_frequency = 0.1  # [Hz]

        # angular speed in rad/s
        self.angular_speed = 2.0 * math.pi * self.angular_frequency

        self.start_time_s = None

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Create a subscriber to the topic 'odom'
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

    def start(self):
        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1 / 60, self.update_callback)

    def stop(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(
                *pose2d
            ),
            throttle_duration_sec=0.5,
        )

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w,
        )

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        pose2 = (
            pose3.position.x,
            pose3.position.y,
            yaw,
        )

        return pose2

    def update_callback(self):
        cmd_vel = self.move_as_eight()
        self.vel_publisher.publish(cmd_vel)

    def move_as_eight(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed

        current_time_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time_s = current_time_s - self.start_time_s

        # Time needed to complete one full circle
        circle_period_s = 1.0 / self.angular_frequency

        # Alternate direction every circle
        circle_count = int(elapsed_time_s // circle_period_s)

        if circle_count % 2 == 0:
            cmd_vel.angular.z = self.angular_speed
        else:
            cmd_vel.angular.z = -self.angular_speed

        return cmd_vel


def main():
    rclpy.init(args=sys.argv)

    node = ControllerNode()
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop()


if __name__ == "__main__":
    main()
