import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Pose, Point
import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from turtle_brick_interfaces.msg import Tilt
from std_msgs.msg import Header
from enum import Enum, auto
import math
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration

    """

    WAITING = auto(),
    MOVING = auto(),
    CAUGHT = auto(),
    DROPPING = auto()


class Catcher(Node):
    """
    Node for controlling the turtle robot to catch brick in rviz.

    Publishes
    ---------
    goal_pose : geometry_msgs/msg/PoseStamped - the goal pose for the robot to move
    visualization_marker : visualization_msgs/msg/Marker - markers in rviz
    tilt : turtle_brick_interfaces/msg/Tilt - the angle of the robot's platform

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
        super().__init__('catcher')
        qos = QoSProfile(depth=10,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.declare_parameter('platform_height', 1.1)
        self.declare_parameter('wheel_radius', .15)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.declare_parameter('brick_side_length', .25)
        self.declare_parameter('frequency', 100)
        self.platform_height = self.get_parameter('platform_height').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value
        self.brick_side_length = self.get_parameter('brick_side_length').value
        self.frequency = self.get_parameter('frequency').value

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        self.goal_pose_pub = self.create_publisher(PoseStamped,
                                                   'goal_pose',
                                                   10)
        self.marker_pub = self.create_publisher(Marker,
                                                'visualization_marker',
                                                qos)
        self.tilt_pub = self.create_publisher(Tilt, 'tilt', 10)

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self, 10)

        self.state = State.WAITING
        self.prev_brick_z = 0.0

        self.text = Marker()
        self.text.header.frame_id = 'world'
        self.text.id = 7
        self.text.type = Marker.TEXT_VIEW_FACING
        self.text.text = 'Unreachable'
        self.text.action = Marker.ADD
        self.text.scale.x = 5.0
        self.text.scale.y = 1.0
        self.text.scale.z = 1.0
        self.text.pose.position.x = 5.5
        self.text.pose.position.y = 5.5
        self.text.pose.position.z = 5.0
        self.text.color.r = 1.0
        self.text.color.g = 0.0
        self.text.color.b = 0.0
        self.text.color.a = 1.0
        self.text.lifetime = Duration(sec=3)

    def timer_callback(self):
        """
        Timer callback for controlling robot.

        Publishes commands at a set frequency paramter (commands vary based on
        robot state)

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

            if self.state == State.WAITING:
                
                if (tf_world_brick.transform.translation.z > self.platform_height) and (tf_world_brick.transform.translation.z < self.prev_brick_z):
                    t_xy = (math.dist([tf_world_brick.transform.translation.x, tf_world_brick.transform.translation.y],
                                    [tf_world_base.transform.translation.x, tf_world_base.transform.translation.y]) - .35)/self.max_velocity
                    t_z = ((2*(tf_world_brick.transform.translation.z - self.platform_height))/self.gravity_accel)**.5

                    if t_xy <= t_z:
                        self.state = State.MOVING
                        x = tf_world_brick.transform.translation.x
                        y = tf_world_brick.transform.translation.y
                        self.goal_pose_pub.publish(
                            PoseStamped(header=Header(
                                stamp=self.get_clock().now().to_msg(),
                                frame_id='world'),
                                pose=Pose(position=Point(x=x, y=y))))
                    else:
                        self.marker_pub.publish(self.text)

            elif self.state == State.MOVING:

                brick_dist = math.dist([tf_world_brick.transform.translation.x,
                                        tf_world_brick.transform.translation.y],
                                       [tf_world_base.transform.translation.x,
                                        tf_world_base.transform.translation.y])
                
                if (tf_world_brick.transform.translation.z == (self.platform_height + .125)):
                    self.state = State.CAUGHT
                    self.goal_pose_pub.publish(
                        PoseStamped(header=Header(
                            stamp=self.get_clock().now().to_msg(),
                            frame_id='odom'),
                            pose=Pose(position=Point(x=0.0, y=0.0))))
                
            elif self.state == State.CAUGHT:

                base_dist = math.dist([tf_world_base.transform.translation.x,
                                       tf_world_base.transform.translation.y],
                                      [5.544,
                                       5.544])

                if base_dist < .05:
                    self.state = State.DROPPING

            elif self.state == State.DROPPING:

                self.tilt_pub.publish(Tilt(angle=.6))

                brick_dist = math.dist([tf_world_brick.transform.translation.x,
                                        tf_world_brick.transform.translation.y],
                                       [tf_world_platform.transform.translation.x,
                                        tf_world_platform.transform.translation.y])
                
                if brick_dist > (.35 + self.brick_side_length/2):
                    self.state = State.WAITING
                    self.tilt_pub.publish(Tilt(angle=0.0))

            self.prev_brick_z = tf_world_brick.transform.translation.z

        except tf2_ros.LookupException:
            pass
        except tf2_ros.ConnectivityException:
            pass
        except tf2_ros.ExtrapolationException:
            pass


def main(args=None):
    """Entrypoint for the catcher ROS node."""
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
