import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.msg import Pose
from collections import deque
from enum import Enum

from turtlesim.srv import SetPen

from action_usi_angry_turtle_interfaces.action import MoveToGoal


class State(Enum):
    """
    Class that defines the states of the state machine that the writer node will implement to manage the writing, chasing and returning behaviors.
    """

    WRITING = "WRITING"
    ANGRY = "ANGRY"
    RETURNING = "RETURNING"


class WriterNode(Node):
    """
    This node is responsible of sending the goals to the move2goal action server to make the turtle draw the letters USI.
    It is also responsible of the state machine WRITING -> ANGRY -> RETURNING, where:
        - WRITING: the turtle is writing the letters, following the goals in the queue
        - ANGRY: the turtle has detected the second turtle and is going to eliminate it
        - RETURNING: the turtle has eliminated the second turtle and is going back to the last position to resume writing the letters from where it stopped
    """

    # Pen configurations
    PEN_OFF = SetPen.Request(r=0, g=0, b=0, width=0, off=True)
    PEN_ON = SetPen.Request(r=255, g=255, b=255, width=4, off=False)

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

        # Pen service client to be able to change color and lift the pen
        self.pen_client = self.create_client(SetPen, "/turtle1/set_pen")

        # Create a subscriber to the topic '/turtle1/pose' which will call self.pose_callback
        # every time there is a new message in the topic of type Pose
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

        # Distance in meters at which the turtle will start chasing the second turtle when detected
        self.k_chase = 2.0

        # Initialize the state machine in the WRITING state
        self.state = State.WRITING

        # TODO: Change Start to write USI
        self.get_logger().info("Writing USI")
        start_pose = Pose()
        start_pose.x = 1.0
        start_pose.y = 9.0

        # We fill the queue with the relative transforms to draw the letter U starting from the initial position
        self.goals_queue.extend([(start_pose.x, start_pose.y, self.PEN_OFF)])
        self.goals_queue.extend(self.letter_U(start_pose, size=8.0))
        self.goals_queue.extend(self.letter_S(start_pose, size=8.0))
        self.goals_queue.extend(self.letter_I(start_pose, size=8.0))
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

        pen = None
        x, y = None, None
        entry = self.goals_queue.popleft()

        if len(entry) == 2:
            x, y = entry
            pen = None
        elif len(entry) == 3:
            x, y, pen = entry

        goal = MoveToGoal.Goal()
        goal.x = x
        goal.y = y
        goal.tolerance = 0.5

        # First we check if pen is defined
        if pen is not None:
            self.get_logger().info(
                f"Setting pen: r={pen.r}, g={pen.g}, b={pen.b}, width={pen.width}, off={pen.off}"
            )
            self._set_pen(pen)

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
        """Callback executed when feedback is received from the movement action server."""
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
                self.PEN_ON,
            ),  # We start from the top left, we go down
            (
                initial_pos.x + 0.25 * size,
                initial_pos.y - 0.5 * size,
            ),  # We go right
            (initial_pos.x + 0.25 * size, initial_pos.y),  # We go up
        ]

    def letter_S(self, initial_pos, size=1.0):
        """Returns a list of relative transforms to be applied to the initial position of the word USI to draw the letter S."""
        """
        Top Left = 1,9
        Top Right = 9,9
        Bottom Right = 9,1
        Bottom Left = 1,1
        """
        return [
            (
                initial_pos.x + 0.9 * size,
                initial_pos.y,
                self.PEN_OFF,
            ),  # We start from the top left, we go right
            (
                initial_pos.x + 0.4 * size,
                initial_pos.y - 0.05 * size,
                self.PEN_ON,
            ),  # left first curve
            (
                initial_pos.x + 1.0 * size,
                initial_pos.y - 0.3 * size,
            ),  # right first curve
            (initial_pos.x + 0.4 * size, initial_pos.y - 0.5 * size),
        ]

    def letter_I(self, initial_pos, size=1.0):
        return [
            (
                initial_pos.x + 1.1 * size,
                initial_pos.y - 0.5 * size,
                self.PEN_OFF,
            ),  # Going to the right of the S
            (initial_pos.x + 1.1 * size, initial_pos.y, self.PEN_ON),
        ]

    def _set_pen(self, pen_request):
        """Sends a request to the set_pen service to change the pen configuration."""
        if not self.pen_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SetPen service not available!")
            return

        # We assume that the pen request will work,
        # so we don't wait for the response and we don't set a callback for it
        # TODO: Might change
        self.pen_client.call_async(pen_request)


def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the WriterNode class
    writer_node = WriterNode()

    # Spin the node
    rclpy.spin(writer_node)


if __name__ == "__main__":
    main()
