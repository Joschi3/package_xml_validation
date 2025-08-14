import rclpy
from rclpy.node import Node
import rclpy.exceptions


class PythonPkg(Node):
    def __init__(self):
        """Constructor"""
        super().__init__("python_pkg")

        self.setup()

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""


def main():
    rclpy.init()
    node = PythonPkg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
