import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.msg import Pose
from collections import deque

from action_usi_angry_turtle_interfaces.action import MoveToGoal


class WriterNode(Node):
    def __init__(self):
        # Create a node with name 'writer'
        super().__init__("writer")

        self.get_logger().info("Writer node has been started")

        # We will save the current turtle1 pose, which will be updated by the pose_callback
        self.current_pose = None

        # Queue to store the relative transforms to the current position that will form the path to be followed by the turtle.
        self.goals_queue = deque()

        # Here we create the action client that will send the goals to the move2goal action server
        self._action_client = ActionClient(self, MoveToGoal, "move_to_goal")
        self._active_goal_handle = None

        # Create a subscriber to the topic '/turtle1/pose' which will call self.pose_callback
        # every time there is a new message in the topic of type Pose
        # For now we are controlling only 1 turtle, the default one
        # TODO: modify the code to control multiple turtles, by subscribing to the topic '/XYZ/pose' where XYZ is the turtle workspace
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

        # TODO: Change Start to write USI
        self.get_logger().info("Writing USI")
        start_pose = Pose()
        start_pose.x = 1.0
        start_pose.y = 9.0
        # We fill the queue with the relative transforms to draw the letter U starting from the initial position
        self.goals_queue.extend([(start_pose.x, start_pose.y)])
        self.goals_queue.extend(self.letter_U(start_pose, size=4.0))
        self.goals_queue.extend(self.letter_S(start_pose, size=4.0))
        self.goals_queue.extend(self.letter_I(start_pose, size=4.0))
        self.start_queue()

    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)

    def start_queue(self):
        """Starts sending the goals in the queue to the move2goal action server."""
        if not self.goals_queue:
            self.get_logger().info("No goals in the queue to send.")
            return
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Move2Goal action server not available!")
            return
        self._send_next_goal()

    def _send_next_goal(self):
        """Send the next available goal to the action server and set callback for result"""
        if not self.goals_queue:
            self.get_logger().info("All waypoints completed")
            return

        x, y = self.goals_queue.popleft()
        goal = MoveToGoal.Goal()
        goal.x = x
        goal.y = y
        goal.tolerance = 0.01

        self.get_logger().info(f"Sending next goal: x={goal.x}, y={goal.y}")

        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Callback executed when the task is accepted or rejected by the action server."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by the action server.")
            return

        # We set the active goal handle to be able to cancel the goal if needed
        self._active_goal_handle = goal_handle
        self.get_logger().info(
            "Goal accepted by the action server, waiting for result..."
        )

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Callback executed when the result of the action is received."""
        result_msg = future.result().result
        status = future.result().status

        self.get_logger().info(
            f"Goal finished: success={result_msg.success}, "
            f"final_error={round(result_msg.final_error, 4)}, status={status}"
        )

        # If the waypoint was successfully reached, we send the next one in the queue
        if result_msg.success:
            self._send_next_goal()
        else:
            self.get_logger().warn(
                "Failed to reach the goal, not sending the next one in the queue."
            )

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: x={round(fb.current_x, 4)}, y={round(fb.current_y, 4)}, distance_to_goal={round(fb.distance_to_goal, 4)}"
        )

    def letter_U(self, initial_pos, size=1.0):
        """Returns a list of relative transforms to be applied to the current (initial) position to draw the letter U."""
        """
        Top Left = 1,9
        Top Right = 9,9
        Bottom Right = 9,1
        Bottom Left = 1,1
        """
        return [
            (
                initial_pos.x,
                initial_pos.y - 0.5 * size,
            ),  # We start from the top left, we go down
            (initial_pos.x + 0.25 * size, initial_pos.y - 0.5 * size),  # We go right
            (initial_pos.x + 0.25 * size, initial_pos.y),  # We go up
        ]

    def letter_S(self, initial_pos, size=1.0):
        """Returns a list of relative transforms to be applied to the current (initial) position to draw the letter S."""
        """
        Top Left = 1,9
        Top Right = 9,9
        Bottom Right = 9,1
        Bottom Left = 1,1
        """
        return [
            (
                initial_pos.x + 1.0 * size,
                initial_pos.y,
            ),  # We start from the top left, we go right
            (
                initial_pos.x + 0.5 * size,
                initial_pos.y - 0.05 * size,
            ),  # left first curve
            (
                initial_pos.x + 1.0 * size,
                initial_pos.y - 0.3 * size,
            ),  # right first curve
            (initial_pos.x + 0.5 * size, initial_pos.y - 0.5 * size),
        ]

    def letter_I(self, initial_pos, size=1.0):
        return [
            (
                initial_pos.x + 1.15 * size,
                initial_pos.y - 0.5 * size,
            ),  # Going to the right of the S
            (initial_pos.x + 1.15 * size, initial_pos.y),
        ]


def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the WriterNode class
    writer_node = WriterNode()

    # Spin the node
    rclpy.spin(writer_node)


if __name__ == "__main__":
    main()
