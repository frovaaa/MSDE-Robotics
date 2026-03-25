#
#  move2goal_node.py
#  Velocity controller that moves a Turtlesim turtle toward a user-specified goal position.
#
#  Elia Cereda <elia.cereda@idsia.ch>
#  Simone Arreghini <simone.arreghini@idsia.ch>
#  Dario Mantegazza <dario.mantegazza@idsia.ch>
#  Mirko Nava <mirko.nava@idsia.ch>
#
#  Copyright (C) 2019-2025 IDSIA, USI-SUPSI
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import sys
import time
from math import pow, sin, cos, atan2, sqrt

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from action_usi_angry_turtle_interfaces.action import MoveToGoal


class Move2GoalNode(Node):
    def __init__(self):
        # Creates a node with name 'move2goal'
        super().__init__("move2goal")

        # Callback group definition to allow concurrent execution of callbacks
        # e.g. receiving new goals while moving towards the current goal, receiving the current pose while moving, etc.
        self._callback_group = ReentrantCallbackGroup()

        # Create attributes to store the goal and current poses and tolerance
        self.current_pose = None

        # Create a publisher for the topic '/turtle1/cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Create a subscriber to the topic '/turtle1/pose', which will call self.pose_callback every
        # time a message of type Pose is received
        self.pose_subscriber = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Move2Goal node initialized, waiting for goal pose to be set..."
        )

        # Here we define the ActionServer with the needed callbacks
        self._action_server = ActionServer(
            self,
            MoveToGoal,
            "move_to_goal",
            execute_callback=self.move_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

    def goal_callback(self, goal_request):
        self.get_logger().info("GOAL ARRIVED")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("CANCEL CALLBACK")
        return CancelResponse.ACCEPT

    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)

    def move_callback(self, goal_handle):
        """Callback called whenever a new action goal is received"""
        self.get_logger().info(f"Received new goal handle: {goal_handle.request}")

        result = MoveToGoal.Result()
        goal_pose = Pose()
        goal_pose.x = goal_handle.request.x
        goal_pose.y = goal_handle.request.y
        tolerance = goal_handle.request.tolerance

        # We wait for the position to be available, and we check if the user requested to cancel
        while self.current_pose is None and rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal cancelled before starting to move")
                goal_handle.canceled()
                result.success = False
                result.final_error = -1.0
                return result

            self.get_logger().info("Waiting for current pose to be received...")
            time.sleep(0.1)

        # Position is available
        feedback = MoveToGoal.Feedback()
        feedback.current_x = round(self.current_pose.x, 4)
        feedback.current_y = round(self.current_pose.y, 4)
        feedback.distance_to_goal = round(
            self.euclidean_distance(goal_pose, self.current_pose), 4
        )

        # Set the goal state to succeeded
        # goal_handle.succeed()
        # result = MoveToGoal.Result()
        # result.success = True
        # result.final_error = 0.0
        # return result

        while rclpy.ok():
            # Check if the goal has been cancelled by the user
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal cancelled by user")
                goal_handle.canceled()
                result.success = False
                result.final_error = -1.0
                return result

            if self.euclidean_distance(goal_pose, self.current_pose) >= tolerance:
                # We still haven't reached the goal pose. Use a proportional controller to compute velocities
                # that will move the turtle towards the goal (https://en.wikipedia.org/wiki/Proportional_control)

                # Twist represents 3D linear and angular velocities, in turtlesim we only care about 2 dimensions:
                # linear velocity along the x-axis (forward) and angular velocity along the z-axis (yaw angle)
                cmd_vel = Twist()
                cmd_vel.linear.x = self.linear_vel(goal_pose, self.current_pose)
                cmd_vel.angular.z = self.angular_vel(goal_pose, self.current_pose)

                # Publish the command
                self.vel_publisher.publish(cmd_vel)

                # Update the feedback to be sent to the user
                feedback.current_x = round(self.current_pose.x, 4)
                feedback.current_y = round(self.current_pose.y, 4)
                feedback.distance_to_goal = round(
                    self.euclidean_distance(goal_pose, self.current_pose), 4
                )
                goal_handle.publish_feedback(feedback)
            else:
                self.get_logger().info("Goal reached")

                # Stop the turtle
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.vel_publisher.publish(cmd_vel)

                goal_handle.succeed()
                result.success = True
                result.final_error = round(
                    self.euclidean_distance(goal_pose, self.current_pose), 4
                )
                return result
            time.sleep(0.1)

    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(
            pow((goal_pose.x - current_pose.x), 2)
            + pow((goal_pose.y - current_pose.y), 2)
        )

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_theta = self.steering_angle(goal_pose, current_pose)
        return constant * self.angular_difference(goal_theta, current_pose.theta)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = Move2GoalNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # We try to spin the node
    try:
        executor.spin()
    finally:
        # Destroy the node explicitly
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# Changes compared to the original code:
"""
- The node now doesn't automatically shut down after reaching the goal, but instead it waits for a new goal to be set by the user.
- The goal_pose is not passed as an argument to the constructor of the node, but instead it is set by calling the set_goal method. This allows us to change the goal pose while the node is running.
- Added ActionServer to allow other nodes to set a new goal pose by sending a goal to the action server, and get feedback.
"""
