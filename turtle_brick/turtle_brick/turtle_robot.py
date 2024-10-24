import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from turtle_brick_interfaces.msg import Tilt
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from enum import Enum, auto
import math
from math import pi

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    STOPPED = auto(),
    MOVING = auto()

class Turtle_Robot(Node):

    def __init__(self):
        super().__init__('turtle_robot')
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.declare_parameter('platform_height', 1.1)
        self.declare_parameter('wheel_radius', .15)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.platform_height = self.get_parameter('platform_height').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value
        

        self.tmr = self.create_timer(1/100, self.timer_callback)
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self.tilt_sub = self.create_subscription(Tilt, 'tilt', self.tilt_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self, 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos)

        self.state = State.STOPPED
        self.pose = []
        self.prev_pose = []
        self.goalpose = []
        self.prev_goalpose = []
        self.goal_pose_set = False
        self.arrived_at_pose = False
        self.tilt = 0.0
        self.start_x = 5.544
        self.start_y = 5.544
        self.val = 1.0
        self.stem_ang = 0.0
        self.distance_error = 0.0
        self.gain = 1
        self.minimum_vel = 2.0
        self.threshold = .05

        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = 'world'
        world_odom_tf.child_frame_id = 'odom'
        world_odom_tf.transform.translation.x = self.start_x
        world_odom_tf.transform.translation.y = self.start_y
        self.static_tf_broadcaster.sendTransform(world_odom_tf)
        self.get_logger().info('Static Transform: world->odom')

        odom_base_tf = TransformStamped()
        odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        odom_base_tf.header.frame_id = 'odom'
        odom_base_tf.child_frame_id = 'base_link'
        self.static_tf_broadcaster.sendTransform(odom_base_tf)


    def timer_callback(self):

        joint_state = JointState()
        joint_state.name = ['connector_to_platform', 'base_to_stem', 'stem_to_wheel']

        if type(self.pose) != type([]):
            if not self.state == State.STOPPED:

                if self.goalpose.header.frame_id == 'odom':
                    goalpose_x = self.goalpose.pose.position.x + self.start_x
                    goalpose_y = self.goalpose.pose.position.y + self.start_y
                elif self.goalpose.header.frame_id == 'world':
                    goalpose_x = self.goalpose.pose.position.x
                    goalpose_y = self.goalpose.pose.position.y

                self.distance_error = math.dist([goalpose_x, goalpose_y], [self.pose.x, self.pose.y])
                self.stem_ang = math.atan2(goalpose_y - self.pose.y, goalpose_x - self.pose.x)

                if self.stem_ang > math.pi:
                    self.stem_ang = self.stem_ang - 2*math.pi
                elif self.stem_ang < -math.pi:
                    self.stem_ang = 2*math.pi + self.stem_ang

                if type(self.prev_goalpose) == type([]):
                    joint_state.position = [self.tilt,
                                            self.stem_ang,
                                            math.dist([self.pose.x, self.pose.y],
                                                      [self.start_x, self.start_y])/self.wheel_radius]
                else:
                    if self.prev_goalpose.header.frame_id == 'odom':
                        prev_goalpose_x = self.prev_goalpose.pose.position.x + self.start_x
                        prev_goalpose_y = self.prev_goalpose.pose.position.y + self.start_y
                    elif self.prev_goalpose.header.frame_id == 'world':
                        prev_goalpose_x = self.prev_goalpose.pose.position.x
                        prev_goalpose_y = self.prev_goalpose.pose.position.y
                    joint_state.position = [self.tilt,
                                            self.stem_ang,
                                            math.dist([self.pose.x, self.pose.y],
                                                      [prev_goalpose_x, prev_goalpose_y])/self.wheel_radius]

                if self.distance_error*self.gain > self.max_velocity:
                    self.vel_pub.publish(
                        Twist(linear=Vector3(x=self.max_velocity*math.cos(self.stem_ang),
                                            y=self.max_velocity*math.sin(self.stem_ang)),
                            angular=Vector3(z=0.0)))
                    joint_state.velocity = [0, 0, self.max_velocity/self.wheel_radius]
                else:
                    self.vel_pub.publish(
                        Twist(linear=Vector3(x=max(self.distance_error*self.gain,
                                                self.minimum_vel)*math.cos(self.stem_ang),
                                            y=max(self.distance_error*self.gain,
                                                self.minimum_vel)*math.sin(self.stem_ang)),
                            angular=Vector3(z=0.0)))
                    joint_state.velocity = [0,
                                            0,
                                            max(self.distance_error*self.gain, self.minimum_vel)/self.wheel_radius]
            else:
                self.vel_pub.publish(
                        Twist(linear=Vector3(x=0.0, y=0.0),
                            angular=Vector3(z=0.0)))
                if type(self.prev_goalpose) == type([]):
                    joint_state.position = [self.tilt,
                                            self.stem_ang,
                                            math.dist([self.pose.x, self.pose.y],
                                                      [self.start_x, self.start_y])/self.wheel_radius]
                else:
                    if self.prev_goalpose.header.frame_id == 'odom':
                        prev_goalpose_x = self.prev_goalpose.pose.position.x + self.start_x
                        prev_goalpose_y = self.prev_goalpose.pose.position.y + self.start_y
                    elif self.prev_goalpose.header.frame_id == 'world':
                        prev_goalpose_x = self.prev_goalpose.pose.position.x
                        prev_goalpose_y = self.prev_goalpose.pose.position.y
                    joint_state.position = [self.tilt,
                                            self.stem_ang,
                                            math.dist([self.pose.x, self.pose.y],
                                                      [prev_goalpose_x, prev_goalpose_y])/self.wheel_radius]
                joint_state.velocity = [0, 0, 0]
        else:
            joint_state.position = [self.tilt, 0, 0]

        if self.distance_error < self.threshold:
            self.state = State.STOPPED

        time = self.get_clock().now().to_msg()
        joint_state.header.stamp = time
        self.joint_state_pub.publish(joint_state)


    def pose_callback(self, pose):
        """ Updates the self.pose, self.prev_pose, and self.actual_distance variables 
        
            Args:
                pose (dictionary) : the current pose of the turtle
        """
        self.prev_pose = self.pose
        self.pose = pose
        if type(self.pose) == type(self.prev_pose):
            odom_base_tf = TransformStamped()
            odom_base_tf.header.stamp = self.get_clock().now().to_msg()
            odom_base_tf.header.frame_id = 'odom'
            odom_base_tf.child_frame_id = 'base_link'
            odom_base_tf.transform.translation.x = self.pose.x - self.start_x
            odom_base_tf.transform.translation.y = self.pose.y - self.start_y
            self.tf_broadcaster.sendTransform(odom_base_tf)
    
    def goal_pose_callback(self, goalpose):
        if type(goalpose) != type([]) and self.goalpose != goalpose:
            self.prev_goalpose = self.goalpose
            self.goalpose = goalpose
            self.state = State.MOVING

    def tilt_callback(self, tilt):
        if self.tilt != tilt:
            self.tilt = tilt


def main(args=None):

    """Entrypoint for the turtle_robot ROS node."""
    rclpy.init(args=args)
    node = Turtle_Robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
