import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class WriterNode(Node):
    def __init__(self):
        # Create a node with name 'writer'
        super().__init__("writer")

        self.get_logger().info("Writer node has been started")

        # Create a subscriber to the topic '/turtle1/pose' which will call self.pose_callback
        # every time there is a new message in the topic of type Pose
        # For now we are controlling only 1 turtle, the default one
        # TODO: modify the code to control multiple turtles, by subscribing to the topic '/XYZ/pose' where XYZ is the turtle workspace
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.get_logger().info(f"Received pose of turtle1: msg={msg}")


def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the WriterNode class
    writer_node = WriterNode()

    # Spin the node
    rclpy.spin(writer_node)


if __name__ == "__main__":
    main()
