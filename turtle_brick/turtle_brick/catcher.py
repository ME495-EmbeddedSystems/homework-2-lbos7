import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Pose, TransformStamped
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from turtle_brick.physics import World
from turtle_brick_interfaces.srv import Place
from turtle_brick_interfaces.msg import Tilt
from std_srvs.srv import Empty
from std_msgs.msg import Header
from enum import Enum, auto
import math
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped, Quaternion

class Catcher(Node):

    def __init__(self):
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.declare_parameter('platform_height', 1.1)
        self.declare_parameter('wheel_radius', .15)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.declare_parameter('brick_side_length', .25)
        self.platform_height = self.get_parameter('platform_height').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value

        self.timer = self.create_timer(1/100, self.timer_callback)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', qos)
        self.tilt_pub = self.create_publisher(Tilt, 'tilt', 10)
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self, 10)

        self.prev_brick_z = 0.0
        self.decision_made = False
        self.can_catch = False
        self.text_shown_for_3_secs = False
        self.caught = False
        self.time_stamp = self.get_clock().now().to_msg()

        self.text = Marker()
        self.text.header.frame_id = 'world'
        self.text.id = 7
        self.text.type = Marker.TEXT_VIEW_FACING
        self.text.text = 'Unreachable.'
        self.text.action = Marker.ADD
        self.text.scale.x = 5.0
        self.text.scale.y = 1.0
        self.text.scale.z = 1.0
        self.text.pose.position.x = 5.0
        self.text.pose.position.y = 1.0
        self.text.pose.position.z = 2.0
        self.text.color.r = 1.0
        self.text.color.g = 0.0
        self.text.color.b = 0.0
        self.text.color.a = 1.0

    def timer_callback(self):
        try:
            tf_world_base = self.buffer.lookup_transform('world', 'base_link', rclpy.time.Time())
            tf_world_brick = self.buffer.lookup_transform('world', 'brick', rclpy.time.Time())
            tf_odom_base = self.buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}')

        if not self.caught:
            if not self.decision_made:

                if (tf_world_brick.transform.translation.z > self.platform_height) and (tf_world_brick.transform.translation.z < self.prev_brick_z):
                    t_xy = (math.dist([tf_world_brick.transform.translation.x, tf_world_brick.transform.translation.y],
                                    [tf_world_base.transform.translation.x, tf_world_base.transform.translation.y]) - .35)/self.max_velocity
                    t_z = ((2*(tf_world_brick.transform.translation.z - self.platform_height))/self.gravity_accel)**.5

                    if t_xy <= t_z:
                        self.can_catch = True
                    else:
                        self.can_catch = False

                    self.decision_made = True

            else:
                
                if self.can_catch:
                    self.goal_pose_pub.publish(
                        PoseStamped(header=Header(
                                        stamp=self.get_clock().now().to_msg(), frame_id='world'),
                                    pose=Pose(
                                        x=tf_world_brick.transform.translation.x, y=tf_world_brick.transform.translation.y)))
                    
                    if tf_world_brick.transform.translation.z == (self.platform_height + .125):
                        self.caught = True

                else:
                    time = self.get_clock().now().to_msg()
                    if not self.text_shown_for_3_secs:
                        self.marker_pub.publish(self.text)
                        self.time_stamp = self.get_clock().now().to_msg()
                        self.text.color.a = 0.0
                    else:
                        self.marker_pub.publish(self.text)
                        self.text.color.a = 1.0
                        self.decision_made = False
                        self.text_shown_for_3_secs = False
                    
                    time = self.get_clock().now().to_msg()
                    if time.sec - self.time_stamp.sec >= 3.0:
                        self.text_shown_for_3_secs = True

        else:
            self.goal_pose_pub.publish(
                        PoseStamped(header=Header(
                                        stamp=self.get_clock().now().to_msg(), frame_id='odom'),
                                    pose=Pose(x=0.0, y=0.0)))
            
            if (tf_odom_base.transform.translation.x == 0) and (tf_odom_base.transform.translation.y == 0):
                self.tilt_pub.publish(.6)





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