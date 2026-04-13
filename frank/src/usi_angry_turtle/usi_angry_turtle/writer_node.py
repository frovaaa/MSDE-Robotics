import time

import rclpy
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.msg import Pose
from collections import deque
from enum import Enum

from turtlesim.srv import SetPen, Kill, Spawn

from action_usi_angry_turtle_interfaces.action import MoveToGoal


class State(Enum):
    """
    Class that defines the states of the state machine that the writer node will implement to manage the writing, chasing and returning behaviors.
    State tells us the current high-level mode
    """

    WRITING = "WRITING"
    ANGRY = "ANGRY"
    RETURNING = "RETURNING"


class ActiveGoalType(Enum):
    """
    Class that defines the type of the active goal, to be able to distinguish if the current active goal is a writing goal or a chasing goal.
    This is useful to decide the behavior of the node when a new goal is received while we are in the ANGRY state, because if we are chasing the second turtle and we receive a new writing goal, we want to ignore it until we have finished chasing the second turtle and we are back to the last writing position.
    ActiveGoal tells us the current action goal type
    """

    WRITING = "WRITING"
    CHASING = "CHASING"
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
        self.goal_in_progress = False

        # Pen service client to be able to change color and lift the pen
        self.pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        # We also need a service to kill turtle2
        self.kill_client = self.create_client(Kill, "/kill")
        # Spawn client to when we add turtle2
        self.spawn_client = self.create_client(Spawn, "/spawn")

        # Create a subscriber to the topic '/turtle1/pose' which will call self.pose_callback
        # every time there is a new message in the topic of type Pose
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

        # We also create a subscriber to the topic '/turtle2/pose' to be able to detect the second turtle and chase it when detected
        self.turtle2_pose = None
        self.turtle2_subscriber = self.create_subscription(
            Pose, "/turtle2/pose", self.turtle2_pose_callback, 10
        )

        self.chase_timer = None
        self.last_entry = None
        self.last_writing_position = None
        # Distance in meters at which the turtle will start chasing the second turtle when detected
        self.k_chase = 2.0
        # Distance in meters at which the turtle2 will be killed
        self.k_kill = 0.5

        # Initialize the state machine in the WRITING state
        self.state = State.WRITING
        self.active_goal_type = ActiveGoalType.WRITING

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

    def turtle2_pose_callback(self, msg):
        """Callback called every time a new Pose message is received from the topic /turtle2/pose."""
        self.turtle2_pose = msg
        self.turtle2_pose.x = round(self.turtle2_pose.x, 4)
        self.turtle2_pose.y = round(self.turtle2_pose.y, 4)

        if self.current_pose is not None:
            distance = (
                (self.current_pose.x - self.turtle2_pose.x) ** 2
                + (self.current_pose.y - self.turtle2_pose.y) ** 2
            ) ** 0.5

            if self.state == State.WRITING and self.current_pose is not None:
                if distance < self.k_chase:
                    self.get_logger().warn(
                        f"Second turtle detected at distance {round(distance, 4)}! Starting to chase it..."
                    )
                    # turn off the pen
                    self._set_pen(self.PEN_OFF)
                    # we set the active goal type to CHASING
                    self.active_goal_type = ActiveGoalType.CHASING

                    # We cancel the current goal,
                    # when it is succesfully canceled, the callback will change the state to ANGRY
                    if self._active_goal_handle is not None:
                        cancel_future = self._active_goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(self._cancel_done_callback)
                    else:
                        self.get_logger().warn(
                            "No active goal handle to cancel, but we detected the second turtle. This should not happen."
                        )
                        # We change state anyway to start chasing the second turtle, even if we couldn't cancel the current goal for some reason
                        self.state = State.ANGRY
            elif self.state == State.ANGRY and distance < self.k_kill:
                self.get_logger().warn(
                    f"Second turtle is very close at distance {round(distance, 4)}! Terminating it..."
                )

                # We stop the chase timer
                if self.chase_timer is not None:
                    self.chase_timer.cancel()
                    self.chase_timer = None

                    # we also cancel any active goal
                if self._active_goal_handle is not None and self.goal_in_progress:
                    cancel_future = self._active_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self._cancel_chasing_done_callback)
                else:
                    self._start_returning_after_kill()

    def _cancel_chasing_done_callback(self, future):
        """Callback executed when the active chasing goal is canceled before returning."""
        cancel_response = future.result()

        if cancel_response.return_code == 0:
            self.get_logger().info("Active chasing goal successfully canceled.")
        else:
            self.get_logger().warn(
                f"Failed to cancel chasing goal. Return code: {cancel_response.return_code}"
            )

        self.goal_in_progress = False
        self._start_returning_after_kill()

    def _start_returning_after_kill(self):
        """Kill turtle2, switch to RETURNING, and send the return goal."""
        self._kill_turtle("turtle2")

        self.state = State.RETURNING
        self.active_goal_type = ActiveGoalType.RETURNING
        self._set_pen(self.PEN_OFF)

    def _cancel_done_callback(self, future):
        """Callback executed when the writing goal is canceled to begin chasing."""
        cancel_response = future.result()

        if cancel_response.return_code == 0:
            self.get_logger().warn("Current writing goal successfully canceled.")
            if self.last_entry is not None:
                self.get_logger().info(
                    f"Re-queueing the last writing goal to be able to resume writing after chasing the second turtle. Last entry: {self.last_entry}"
                )
                self.goals_queue.appendleft(self.last_entry)
        else:
            self.get_logger().warn(
                f"Failed to cancel the current writing goal. Return code: {cancel_response.return_code}"
            )

        self.goal_in_progress = False
        self.state = State.ANGRY

        if self.chase_timer is None:
            self.chase_timer = self.create_timer(0.2, self._chase_turtle2)

    def _chase_turtle2(self):
        """Function called by the timer to keep sending goals to chase the second turtle until we are close enough to it."""
        if self.turtle2_pose is None:
            self.get_logger().warn("Turtle2 pose is not available, cannot chase it.")
            return

        if self.current_pose is None:
            self.get_logger().warn(
                "Current pose is not available, cannot chase the second turtle."
            )
            return

        if self.goal_in_progress:
            self.get_logger().info(
                "Goal in progress, waiting to send the next chasing goal..."
            )
            return

        if self.state != State.ANGRY:
            return

        # We send a goal to move to the current position of the second turtle
        goal = MoveToGoal.Goal()
        goal.x = self.turtle2_pose.x
        goal.y = self.turtle2_pose.y
        goal.tolerance = 0.5

        self.active_goal_type = ActiveGoalType.CHASING

        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)

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
        self.last_entry = entry
        # Deep copy self.current_pose
        if self.current_pose is not None:
            self.last_writing_position = Pose()
            self.last_writing_position.x = self.current_pose.x
            self.last_writing_position.y = self.current_pose.y

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

        self.active_goal_type = ActiveGoalType.WRITING

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
        self.goal_in_progress = True
        self.get_logger().info(
            "Goal accepted by the action server, waiting for result..."
        )

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Callback executed when the result of the action is received."""
        result_msg = future.result().result
        status = future.result().status

        self.goal_in_progress = False

        self.get_logger().info(
            f"Goal finished: success={result_msg.success}, "
            f"final_error={round(result_msg.final_error, 4)}, status={status}"
        )

        if self.active_goal_type == ActiveGoalType.WRITING:
            if self.state == State.WRITING and result_msg.success:
                self.get_logger().info(
                    "Successfully reached the writing goal, sending the next one in the queue..."
                )
                self._send_next_goal()
        elif self.active_goal_type == ActiveGoalType.CHASING:
            # we do not resume writing
            pass
        elif self.active_goal_type == ActiveGoalType.RETURNING:
            if self.state == State.RETURNING and result_msg.success:
                self.get_logger().info(
                    "Successfully returned to the last writing position, resuming writing..."
                )
                self.state = State.WRITING
                self.active_goal_type = ActiveGoalType.WRITING
                self._set_pen(self.PEN_ON)
                self._send_next_goal()

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
        self.pen_client.call_async(pen_request)

    def _kill_turtle(self, name):
        """Sends a request to the kill service to eliminate the turtle with the given name."""
        if not self.kill_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Kill service not available!")
            return

        request = Kill.Request()
        request.name = name
        future = self.kill_client.call_async(request)
        future.add_done_callback(self._kill_done_callback)

    def _kill_done_callback(self, future):
        """Called when turtle2 has been killed. Now respawn it randomly."""
        try:
            future.result()
            self.get_logger().info("Turtle2 killed successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to kill turtle2: {e}")
            return

        self._spawn_turtle_random("turtle2")

    def _spawn_turtle_random(self, name):
        """Spawn a turtle with the given name in a random valid turtlesim position."""
        if not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Spawn service not available!")
            return

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = random.uniform(0.0, 6.28)
        request.name = name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self._spawn_done_callback)

    def _spawn_done_callback(self, future):
        """Called when turtle2 has been respawned. Now return to the writing position."""
        try:
            response = future.result()
            self.get_logger().info(
                f"Turtle respawned successfully with name: {response.name}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle2: {e}")
            return

        if self.last_writing_position is not None:
            goal = MoveToGoal.Goal()
            goal.x = self.last_writing_position.x
            goal.y = self.last_writing_position.y
            goal.tolerance = 0.1

            send_goal_future = self._action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback,
            )
            send_goal_future.add_done_callback(self._goal_response_callback)


def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the WriterNode class
    writer_node = WriterNode()

    # Spin the node
    rclpy.spin(writer_node)


if __name__ == "__main__":
    main()
