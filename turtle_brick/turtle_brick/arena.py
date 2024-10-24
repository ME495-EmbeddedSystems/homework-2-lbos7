import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class Arena(Node):

    def __init__(self):
        super.__init__('arena')
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy)


def main(args=None):

    """Entrypoint for the arena ROS node."""
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)