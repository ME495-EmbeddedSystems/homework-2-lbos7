import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
# from physics import World
from turtle_brick_interfaces.srv import Place
from std_msgs.msg import Empty

class Arena(Node):

    def __init__(self):
        super().__init__('arena')
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.timer = self.create_timer(1/250, self.timer_callback)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', qos)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', qos)
        self.place_service = self.create_service(Place, 'place', self.place_callback)
        # self.drop_service = self.create_service(Empty, 'drop', self.drop_callback)

        self.brick_location = []

        self.brick = Marker()
        self.brick.header.frame_id = 'world'
        self.brick.id = 0
        self.brick.type = Marker.CUBE
        self.brick.action = Marker.ADD
        self.brick.scale.x = .5
        self.brick.scale.y = .5
        self.brick.scale.z = .5
        self.brick.color.r = 1.0
        self.brick.color.g = 1.0
        self.brick.color.b = 0.0
        self.brick.color.a = 1.0
    
    
        self.m_west = Marker()
        self.m_west.header.frame_id = 'world'
        self.m_west.id = 1
        self.m_west.type = Marker.CUBE
        self.m_west.action = Marker.ADD
        self.m_west.scale.x = .25
        self.m_west.scale.y = 11.5
        self.m_west.scale.z = 1.5
        self.m_west.pose.position.x = -.125
        self.m_west.pose.position.y = 5.5
        self.m_west.pose.position.z = 0.75
        self.m_west.pose.orientation.x = 0.0
        self.m_west.pose.orientation.y = 0.0
        self.m_west.pose.orientation.z = 0.0
        self.m_west.pose.orientation.w = 0.0
        self.m_west.color.r = 1.0
        self.m_west.color.g = 1.0
        self.m_west.color.b = 1.0
        self.m_west.color.a = 1.0

        self.m_east = Marker()
        self.m_east.header.frame_id = 'world'
        self.m_east.id = 2
        self.m_east.type = Marker.CUBE
        self.m_east.action = Marker.ADD
        self.m_east.scale.x = .25
        self.m_east.scale.y = 11.5
        self.m_east.scale.z = 1.5
        self.m_east.pose.position.x = 11.125
        self.m_east.pose.position.y = 5.5
        self.m_east.pose.position.z = 0.75
        self.m_east.pose.orientation.x = 0.0
        self.m_east.pose.orientation.y = 0.0
        self.m_east.pose.orientation.z = 0.0
        self.m_east.pose.orientation.w = 0.0
        self.m_east.color.r = 1.0
        self.m_east.color.g = 1.0
        self.m_east.color.b = 1.0
        self.m_east.color.a = 1.0

        self.m_north = Marker()
        self.m_north.header.frame_id = 'world'
        self.m_north.id = 3
        self.m_north.type = Marker.CUBE
        self.m_north.action = Marker.ADD
        self.m_north.scale.x = 11.5
        self.m_north.scale.y = .25
        self.m_north.scale.z = 1.5
        self.m_north.pose.position.x = 5.5
        self.m_north.pose.position.y = 11.125
        self.m_north.pose.position.z = 0.75
        self.m_north.pose.orientation.x = 0.0
        self.m_north.pose.orientation.y = 0.0
        self.m_north.pose.orientation.z = 0.0
        self.m_north.pose.orientation.w = 0.0
        self.m_north.color.r = 1.0
        self.m_north.color.g = 1.0
        self.m_north.color.b = 1.0
        self.m_north.color.a = 1.0

        self.m_south = Marker()
        self.m_south.header.frame_id = 'world'
        self.m_south.id = 4
        self.m_south.type = Marker.CUBE
        self.m_south.action = Marker.ADD
        self.m_south.scale.x = 11.5
        self.m_south.scale.y = .25
        self.m_south.scale.z = 1.5
        self.m_south.pose.position.x = 5.5
        self.m_south.pose.position.y = -.125
        self.m_south.pose.position.z = 0.75
        self.m_south.pose.orientation.x = 0.0
        self.m_south.pose.orientation.y = 0.0
        self.m_south.pose.orientation.z = 0.0
        self.m_south.pose.orientation.w = 0.0
        self.m_south.color.r = 1.0
        self.m_south.color.g = 1.0
        self.m_south.color.b = 1.0
        self.m_south.color.a = 1.0

        time = self.get_clock().now().to_msg()
        self.m_west.header.stamp = time
        self.m_east.header.stamp = time
        self.m_north.header.stamp = time
        self.m_south.header.stamp = time

        self.m_array = MarkerArray()
        self.m_array.markers = [self.m_west, self.m_east, self.m_north, self.m_south]
        
        self.marker_array_pub.publish(self.m_array)

    def timer_callback(self):
        pass

    def place_callback(self, request, response):
        self.brick_location = request.brick_location
        self.brick.pose.position.x = request.brick_location.x
        self.brick.pose.position.y = request.brick_location.y
        self.brick.pose.position.z = request.brick_location.z
        self.brick.pose.orientation.x = 0.0
        self.brick.pose.orientation.y = 0.0
        self.brick.pose.orientation.z = 0.0
        self.brick.pose.orientation.w = 0.0
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.brick)
        return response

    def drop_callback(self, request, response):
        pass



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