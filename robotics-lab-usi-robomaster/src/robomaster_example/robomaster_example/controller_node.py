import rclpy
import rclpy.logging
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from enum import Enum

import sys
import math


class State(Enum):
    SEARCHING_WALL = 1  # When the robot goes forward fast looking for a wall
    APPROACHING_WALL = 2  # Spotted a wall, we approach slowly
    CENTERING_WALL = 3  # Reached the wall, we rotate to center it
    DONE_CENTERED_FORWARD = 4  # Wall is centered, we stop


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
        self.tof = {"br": None, "fr": None, "bl": None, "fl": None}
        self.tof_max = 10.0  # [m] maximum distance that the ToF sensors can measure
        self.min_dist_from_wall = (
            0.2  # [m] minimum distance from the wall to consider it as "close enough"
        )

        self.state = State.SEARCHING_WALL

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

        # Subscribers for tof sensors
        # 0 - back right
        self.tof_br_sub = self.create_subscription(
            Range, "range_0", self.tof_br_callback, 10
        )
        # 1 - front right
        self.tof_fr_sub = self.create_subscription(
            Range, "range_1", self.tof_fr_callback, 10
        )
        # 2 - back left
        self.tof_bl_sub = self.create_subscription(
            Range, "range_2", self.tof_bl_callback, 10
        )
        # 3 - front left
        self.tof_fl_sub = self.create_subscription(
            Range, "range_3", self.tof_fl_callback, 10
        )

    def tof_br_callback(self, msg):
        self.tof["br"] = msg.range

    def tof_fr_callback(self, msg):
        self.tof["fr"] = msg.range

    def tof_bl_callback(self, msg):
        self.tof["bl"] = msg.range

    def tof_fl_callback(self, msg):
        self.tof["fl"] = msg.range

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

        # self.get_logger().info(
        #     "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(
        #         *pose2d
        #     ),
        #     throttle_duration_sec=0.5,
        # )

    def make_cmd_vel(self, linear_x: float, angular_z: float) -> Twist:
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        return cmd_vel

    def tof_sees_something(self, value: float) -> bool:
        return value is not None and value < self.tof_max

    def search_or_approach_wall(self, fl, fr):
        """
        This method is used when we are in the SEARCHING_WALL state, it checks the ToF sensor values to decide if we are still searching for a wall or if we have spotted one and we need to approach it
        """
        fl_seen = self.tof_sees_something(fl)
        fr_seen = self.tof_sees_something(fr)

        if not fl_seen and not fr_seen:
            self.get_logger().info("No wall in sight, going forward...")
            return self.make_cmd_vel(0.5, 0.0)

        if (fl is not None and fl <= self.min_dist_from_wall) or (
            fr is not None and fr <= self.min_dist_from_wall
        ):
            self.state = State.CENTERING_WALL
            self.get_logger().info("Reached the wall, starting centering...")
            return self.make_cmd_vel(0.0, 0.0)

        self.get_logger().info(
            f"Approaching | Distance from wall: fl: {fl:.2f}, fr: {fr:.2f}"
        )

        return self.make_cmd_vel(0.2, 0.0)

    def center_with_wall(self, fl, fr):
        if (
            self.tof_sees_something(fl)
            and self.tof_sees_something(fr)
            and math.isclose(fl, fr, abs_tol=0.01)
        ):
            self.state = State.DONE_CENTERED_FORWARD
            self.get_logger().info("Wall is centered, stopping...")
            return self.make_cmd_vel(0.0, 0.0)

        side = 1 if fl < fr else -1
        self.get_logger().info(
            "Centering | rotating {} | fl: {:.2f}, fr: {:.2f}, diff: {:.2f}".format(
                "left" if side == 1 else "right",
                fl,
                fr,
                abs(fl - fr),
            )
        )

        return self.make_cmd_vel(0.0, side * 0.1)

    def go_to_wall(self):
        fl = self.tof["fl"]
        fr = self.tof["fr"]

        if fl is None or fr is None:
            self.get_logger().info("Waiting for ToF sensor data...")
            return self.make_cmd_vel(0.0, 0.0)

        if self.state == State.SEARCHING_WALL:
            return self.search_or_approach_wall(fl, fr)

        if self.state == State.CENTERING_WALL:
            return self.center_with_wall(fl, fr)

        if self.state == State.DONE_CENTERED_FORWARD:
            return self.make_cmd_vel(0.0, 0.0)

        self.get_logger().warn(f"Unknown wall state: {self.state}")
        return self.make_cmd_vel(0.0, 0.0)

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
        # cmd_vel = self.move_as_eight()
        cmd_vel: Twist = self.go_to_wall()
        self.get_logger().debug(
            "Publishing cmd_vel (linear.x: {:.2f}, angular.z: {:.2f})".format(
                cmd_vel.linear.x, cmd_vel.angular.z
            )
        )
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
