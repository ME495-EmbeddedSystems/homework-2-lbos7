"""A node for simulating an arena in rviz."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from turtle_brick.physics import World
from turtle_brick_interfaces.srv import Place
from std_srvs.srv import Empty
from enum import Enum, auto
import math


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration

    """

    STOPPED = auto(),
    PLACED = auto(),
    CAUGHT = auto(),
    FALLING = auto(),
    DROPPING = auto()


class Arena(Node):
    """
    Node for simulating evironment in rviz.

    Publishes
    ---------
    goal_pose : geometry_msgs/msg/PoseStamped - the goal pose for the robot to move
    visualization_marker : visualization_msgs/msg/Marker - the brick marker
    visualization_marker_array : turtle_brick_interfaces/msg/Tilt - the arena walls

    Services
    ----------
    place : turtle_brick_interfaces/srv/Place - places a brick in 3D space
    drop : std_srvs/srv/Empty - drops the current brick

    Parameters
    ----------
    platform_height : float64 - the height of the robot's platform
    wheel_radius : float64 - the radius of the robot's wheel
    max_velocity : float64 - the maximum velocity at which the robot can travel
    gravity_accel : float64 - the gravitational acceleration constant
    brick_side_length : float64 - the side length of the brick marker
    frequency : float64 - the frequency at which the timer callback is executed

    """

    def __init__(self):
        super().__init__('arena')
        qos = QoSProfile(depth=10,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.declare_parameter('platform_height', 1.1)
        self.declare_parameter('wheel_radius', .15)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.declare_parameter('brick_side_length', .25)
        self.declare_parameter('frequency', 250)
        self.platform_height = self.get_parameter('platform_height').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value
        self.brick_side_length = self.get_parameter('brick_side_length').value
        self.frequency = self.get_parameter('frequency').value

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        self.marker_pub = self.create_publisher(Marker,
                                                'visualization_marker',
                                                qos)
        self.m_array_pub = self.create_publisher(MarkerArray,
                                                 'visualization_marker_array',
                                                 qos)

        self.place_service = self.create_service(Place,
                                                 'place',
                                                 self.place_callback)
        self.drop_service = self.create_service(Empty,
                                                'drop',
                                                self.drop_callback)

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self, 10)

        self.state = State.STOPPED
        self.world_phys = []
        self.world_phys_tilted = []
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.platform_angle = 0.0
        self.prev_platform_angle = 0.0

        self.brick = Marker()
        self.brick.header.frame_id = 'world'
        self.brick.id = 0
        self.brick.type = Marker.CUBE
        self.brick.action = Marker.ADD
        self.brick.scale.x = self.brick_side_length
        self.brick.scale.y = self.brick_side_length
        self.brick.scale.z = self.brick_side_length
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
        self.m_west.pose.orientation.w = 1.0
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
        self.m_east.pose.orientation.w = 1.0
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
        self.m_north.pose.orientation.w = 1.0
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
        self.m_south.pose.orientation.w = 1.0
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
        self.m_array.markers = [self.m_west,
                                self.m_east,
                                self.m_north,
                                self.m_south]

        self.m_array_pub.publish(self.m_array)

    def timer_callback(self):
        """
        Timer callback for managing rviz environment.

        Publishes commands at a set frequency paramter (commands vary based on
        arena state)

        """
        try:
            tf_world_base = self.buffer.lookup_transform('world',
                                                         'base_link',
                                                         rclpy.time.Time())
            tf_world_brick = self.buffer.lookup_transform('world',
                                                          'brick',
                                                          rclpy.time.Time())
            tf_world_platform = self.buffer.lookup_transform('world',
                                                             'platform',
                                                             rclpy.time.Time())
        except tf2_ros.LookupException:
            pass
        except tf2_ros.ConnectivityException:
            pass
        except tf2_ros.ExtrapolationException:
            pass

        if self.state == State.FALLING:
            self.world_phys.drop()
            self.brick.pose.position.z = self.world_phys.brick[2]

            trans = TransformStamped()
            trans.header.frame_id = 'world'
            trans.child_frame_id = 'brick'

            check = self.world_phys.brick[2] + self.brick_side_length/2
            brick_dist = math.dist([self.world_phys.brick[0],
                                    self.world_phys.brick[1]],
                                   [tf_world_base.transform.translation.x,
                                    tf_world_base.transform.translation.y])
            if check <= self.brick_side_length/2:
                self.state = State.STOPPED
                self.brick.pose.position.z = self.brick_side_length/2
            elif (brick_dist <= self.world_phys.radius) and (self.world_phys.brick[2] + self.brick_side_length/2 <= self.platform_height):
                self.state = State.CAUGHT
                self.brick.pose.position.z = self.platform_height + self.brick_side_length/2
                self.offset_x = self.brick.pose.position.x - tf_world_base.transform.translation.x
                self.offset_y = self.brick.pose.position.y - tf_world_base.transform.translation.y

            trans.transform.translation.x = self.brick.pose.position.x
            trans.transform.translation.y = self.brick.pose.position.y
            trans.transform.translation.z = self.brick.pose.position.z

            time = self.get_clock().now().to_msg()
            self.brick.header.stamp = time
            trans.header.stamp = time
            self.marker_pub.publish(self.brick)
            self.tf_broadcaster.sendTransform(trans)

        elif self.state == State.PLACED:
            trans = TransformStamped()
            trans.header.frame_id = 'world'
            trans.child_frame_id = 'brick'
            trans.transform.translation.x = self.brick.pose.position.x
            trans.transform.translation.y = self.brick.pose.position.y
            trans.transform.translation.z = self.brick.pose.position.z
            trans.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(trans)

        elif self.state == State.CAUGHT:
            trans = TransformStamped()
            trans.header.frame_id = 'platform'
            trans.child_frame_id = 'brick'

            self.prev_platform_angle = self.platform_angle
            self.platform_angle = 2*math.acos(tf_world_platform.transform.rotation.w)
            if self.platform_angle != 0.0:
                if self.platform_angle != self.prev_platform_angle:
                    self.world_phys_tilted = World([0, 0, 0],
                                                self.gravity_accel*math.sin(self.platform_angle),
                                                .35,
                                                1/250)
                    self.world_phys_tilted.drop()
                else:
                    self.world_phys_tilted.drop()

                trans.transform.translation.y = self.offset_y + self.world_phys_tilted.brick[2]

            else:
                trans.transform.translation.y = self.offset_y

            trans.transform.translation.x = self.offset_x
            trans.transform.translation.z = .05 + self.brick_side_length/2

            self.brick.pose.position.x = tf_world_brick.transform.translation.x
            self.brick.pose.position.y = tf_world_brick.transform.translation.y
            self.brick.pose.position.z = tf_world_brick.transform.translation.z
            self.brick.pose.orientation.x = tf_world_brick.transform.rotation.x
            self.brick.pose.orientation.y = tf_world_brick.transform.rotation.y
            self.brick.pose.orientation.z = tf_world_brick.transform.rotation.z
            self.brick.pose.orientation.w = tf_world_brick.transform.rotation.w

            time = self.get_clock().now().to_msg()
            self.brick.header.stamp = time
            trans.header.stamp = time
            self.marker_pub.publish(self.brick)
            self.tf_broadcaster.sendTransform(trans)

            brick_dist = math.dist([tf_world_brick.transform.translation.x,
                                    tf_world_brick.transform.translation.y],
                                    [tf_world_platform.transform.translation.x,
                                    tf_world_platform.transform.translation.y])
            if brick_dist > (.35 + self.brick_side_length/2):
                self.state = State.STOPPED
        elif self.state == State.STOPPED:
            trans = TransformStamped()
            trans.header.frame_id = 'world'
            trans.child_frame_id = 'brick'
            trans.transform.translation.x = self.brick.pose.position.x
            trans.transform.translation.y = self.brick.pose.position.y
            trans.transform.translation.z = self.brick.pose.position.z
            trans.transform.rotation.x = self.brick.pose.orientation.x
            trans.transform.rotation.y = self.brick.pose.orientation.y
            trans.transform.rotation.z = self.brick.pose.orientation.z
            trans.transform.rotation.w = self.brick.pose.orientation.w
            trans.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(trans)

    def place_callback(self, request, response):
        """
        Place callback function for placing bricks.

        Places a brick at a specified position in 3D space

        Args:
        ----
        request : the brick_location field contains a point dictionary with x,
        y, and z components
        response : (Empty) - place response

        Return:
        ------
        response : (Empty) - place response

        """
        if type(self.world_phys) == type([]):
            self.world_phys = World([request.brick_location.x,
                                     request.brick_location.y,
                                     request.brick_location.z], self.gravity_accel, .35, 1/250)
        else:
            self.world_phys.brick = [request.brick_location.x,
                                     request.brick_location.y,
                                     request.brick_location.z]
            
        self.brick.pose.position.x = request.brick_location.x
        self.brick.pose.position.y = request.brick_location.y
        self.brick.pose.position.z = request.brick_location.z
        self.brick.pose.orientation.x = 0.0
        self.brick.pose.orientation.y = 0.0
        self.brick.pose.orientation.z = 0.0
        self.brick.pose.orientation.w = 1.0

        trans = TransformStamped()
        trans.header.frame_id = 'world'
        trans.child_frame_id = 'brick'
        trans.transform.translation.x = request.brick_location.x
        trans.transform.translation.y = request.brick_location.y
        trans.transform.translation.z = request.brick_location.z
        trans.transform.rotation.x = 0.0
        trans.transform.rotation.y = 0.0
        trans.transform.rotation.z = 0.0
        trans.transform.rotation.w = 1.0

        time = self.get_clock().now().to_msg()
        self.brick.header.stamp = time
        trans.header.stamp = time
        self.marker_pub.publish(self.brick)
        self.tf_broadcaster.sendTransform(trans)
        self.state = State.PLACED
        return response

    def drop_callback(self, request, response):
        """
        Drop callback for the dropping bricks.

        Drops the brick marker

        Args:
        ----
        request : (Empty) - drop request
        response : (Empty) - drop response

        Return:
        ------
        response : (Empty) - drop response

        """
        if self.state == State.PLACED:
            self.state = State.FALLING
        return response


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
