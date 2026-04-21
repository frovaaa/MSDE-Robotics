import rclpy
import rclpy.logging
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys
import math


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
        self.centered_w_wall = False
        self.centering_w_wall = False

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

    def go_to_wall(self):
        # This method uses the ToF sensors to go straight until it detects a change in the distance values
        # Ones it finds it it goes forward until a certain point,
        # then it centers itself with the wall by rotating until the front sensors are approx the same distance
        if self.tof["fl"] is not None and self.tof["fr"] is not None:
            self.get_logger().debug(
                "ToF sensor data: fl: {:.2f}, fr: {:.2f}".format(
                    self.tof["fl"], self.tof["fr"]
                )
            )
            if self.tof["fl"] >= self.tof_max and self.tof["fr"] >= self.tof_max:
                # No wall in sight, we just go fast forward, no rotation
                self.get_logger().info("No wall in sight, going forward...")
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = 0.0
                return cmd_vel
            elif (
                self.tof["fl"] <= self.min_dist_from_wall
                or self.tof["fr"] <= self.min_dist_from_wall
                or self.centering_w_wall
            ):
                # We are close enough to the wall, we stop
                self.centering_w_wall = True
                self.get_logger().info("Reached the wall, stopping...")
                if (
                    self.tof["fl"] < self.tof_max
                    and self.tof["fr"] < self.tof_max
                    and math.isclose(self.tof["fl"], self.tof["fr"], abs_tol=0.03)
                ):
                    self.centered_w_wall = True
                    self.get_logger().info("Wall is centered, stopping...")
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    return cmd_vel
                else:
                    side = 1 if self.tof["fl"] < self.tof["fr"] else -1
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = side * 0.1
                    self.get_logger().info(
                        "Wall is not centered, rotating {}... | Difference of {:.2f}".format(
                            "left" if side == 1 else "right",
                            abs(self.tof["fl"] - self.tof["fr"]),
                        )
                    )
                    self.get_logger().info(
                        f"Rotating | Distance from wall: fl: {self.tof['fl']:.2f}, fr: {self.tof['fr']:.2f}"
                    )
                    return cmd_vel

                # cmd_vel = Twist()
                # cmd_vel.linear.x = 0.0
                # cmd_vel.angular.z = 0.0
                # return cmd_vel
            elif not self.centering_w_wall and (
                self.tof["fl"] < self.tof_max or self.tof["fr"] < self.tof_max
            ):
                # We see a wall form one of the sensors, we approach slowly
                self.get_logger().info("Spotted a wall, Approaching...")
                self.get_logger().info(
                    f"Approaching | Distance from wall: fl: {self.tof['fl']:.2f}, fr: {self.tof['fr']:.2f}"
                )
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.2
                cmd_vel.angular.z = 0.0
                return cmd_vel
        else:
            self.get_logger().info("Waiting for ToF sensor data...")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            return cmd_vel

        self.get_logger().info("Not possible to get here...")
        self.get_logger().info(
            "tof_fl: {}, tof_fr: {}".format(self.tof["fl"], self.tof["fr"])
        )
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        return cmd_vel

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
