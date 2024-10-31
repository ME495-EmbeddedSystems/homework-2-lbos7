"""A node for controlling turtle robot."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from turtlesim.msg import Pose
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped, PoseWithCovariance, TwistWithCovariance, Point
from nav_msgs.msg import Odometry
from turtle_brick_interfaces.msg import Tilt
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from enum import Enum, auto
import math


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration

    """

    STOPPED = auto(),
    MOVING = auto()


class Turtle_Robot(Node):
    """
    Node for controlling the turtle robot.

    Subscribes
    ---------
    pose : turtlesim_msgs/msg/Pose - the current pose of the turtle in
    turtlesim
    goal_pose : geometry_msgs/msg/PoseStamped - the goal pose for the robot to
    move
    tilt : turtle_brick_interfaces/msg/Tilt - the angle the robot's platform
    should be tilted

    Publishes
    ---------
    cmd_vel : geometry_msgs/msg/Twist - the speed the turtle in turtlesim
    odom : nav_msgs/msg/Odometry - the odometry of the robot
    joint_states : sensor_msgs/msg/JointState - the current joint states of the robot

    Parameters
    ----------
    platform_height : float64 - the height of the robot's platform
    wheel_radius : float64 - the radius of the robot's wheel
    max_velocity : float64 - the maximum velocity at which the robot can travel
    gravity_accel : float64 - the gravitational acceleration constant
    frequency : float64 - the frequency at which the timer callback is executed

    """

    def __init__(self):
        super().__init__('turtle_robot')
        qos = QoSProfile(depth=10,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.declare_parameter('platform_height', 1.1)
        self.declare_parameter('wheel_radius', .15)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.declare_parameter('frequency', 100)
        self.platform_height = self.get_parameter('platform_height').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value
        self.frequency = self.get_parameter('frequency').value
        
        self.tmr = self.create_timer(1/100, self.timer_callback)

        self.pose_sub = self.create_subscription(Pose,
                                                 'pose',
                                                 self.pose_callback,
                                                 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped,
                                                      'goal_pose',
                                                      self.goal_pose_callback,
                                                      10)
        self.tilt_sub = self.create_subscription(Tilt,
                                                 'tilt',
                                                 self.tilt_callback,
                                                 10)
        
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState,
                                                     'joint_states',
                                                     10)
        
        self.tf_broadcaster = TransformBroadcaster(self, 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos)

        self.state = State.STOPPED
        self.pose = []
        self.prev_pose = []
        self.goalpose = []
        self.prev_goalpose = []
        self.goal_pose_set = False
        self.arrived_at_pose = False
        self.tilt_angle = 0.0
        self.start_x = 5.544
        self.start_y = 5.544
        self.val = 1.0
        self.stem_ang = 0.0
        self.distance_error = 0.0
        self.gain = 1
        self.minimum_vel = 2.0
        self.threshold = .015
        self.accum_dist = 0.0

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
        self.tf_broadcaster.sendTransform(odom_base_tf)

    def timer_callback(self):
        """
        Timer callback for controlling the turtle robot.

        Publishes commands at a set frequency paramter (commands vary based on
        robot state)

        """
        joint_state = JointState()
        joint_state.name = ['connector_to_platform',
                            'base_to_stem',
                            'stem_to_wheel']

        if self.state == State.MOVING:
            if self.goalpose.header.frame_id == 'odom':
                goalpose_x = self.goalpose.pose.position.x + self.start_x
                goalpose_y = self.goalpose.pose.position.y + self.start_y
            elif self.goalpose.header.frame_id == 'world':
                goalpose_x = self.goalpose.pose.position.x
                goalpose_y = self.goalpose.pose.position.y

            self.distance_error = math.dist([goalpose_x, goalpose_y],
                                            [self.pose.x, self.pose.y])
            self.stem_ang = math.atan2(goalpose_y - self.pose.y,
                                       goalpose_x - self.pose.x)

            if self.stem_ang > math.pi:
                self.stem_ang = self.stem_ang - 2*math.pi
            elif self.stem_ang < -math.pi:
                self.stem_ang = 2*math.pi + self.stem_ang

            if type(self.prev_goalpose) == type([]):
                joint_state.position = [self.tilt_angle,
                                        self.stem_ang,
                                        (math.dist([self.pose.x, self.pose.y],
                                                  [self.start_x, self.start_y])/self.wheel_radius) + self.accum_dist]
            else:
                if self.prev_goalpose.header.frame_id == 'odom':
                    prev_goalpose_x = self.prev_goalpose.pose.position.x + self.start_x
                    prev_goalpose_y = self.prev_goalpose.pose.position.y + self.start_y
                elif self.prev_goalpose.header.frame_id == 'world':
                    prev_goalpose_x = self.prev_goalpose.pose.position.x
                    prev_goalpose_y = self.prev_goalpose.pose.position.y
                joint_state.position = [self.tilt_angle,
                                        self.stem_ang,
                                        (math.dist([self.pose.x,
                                                   self.pose.y],
                                                  [prev_goalpose_x,
                                                   prev_goalpose_y])/self.wheel_radius) + self.accum_dist]

            joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_pub.publish(joint_state)   
            self.vel_pub.publish(
                Twist(linear=Vector3(
                    x=self.max_velocity*math.cos(self.stem_ang),
                    y=self.max_velocity*math.sin(self.stem_ang)),
                    angular=Vector3(z=0.0)))
            if type(self.pose) != type([]):
                self.odom_pub.publish(
                    Odometry(header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id="odom"),
                        child_frame_id = "base_link",
                        pose=PoseWithCovariance(
                            pose=geometry_msgs.msg.Pose(
                                position=Point(
                                    x=self.pose.x,
                                    y=self.pose.y))),
                        twist=TwistWithCovariance(
                            twist=Twist(
                                linear=Vector3(
                                    x=self.max_velocity*math.cos(self.stem_ang),
                                    y=self.max_velocity*math.sin(self.stem_ang)),
                                angular=Vector3(z=0.0)))))

            if self.distance_error < self.threshold:
                self.state = State.STOPPED
                if type(self.prev_goalpose) != type([]):
                    self.accum_dist = math.dist([self.goalpose.pose.position.x,
                                                 self.goalpose.pose.position.y],
                                                 [self.prev_goalpose.pose.position.x,
                                                  self.prev_goalpose.pose.position.y])

        elif self.state == State.STOPPED:
            self.vel_pub.publish(
                        Twist(linear=Vector3(x=0.0, y=0.0),
                              angular=Vector3(z=0.0)))
            if type(self.pose) != type([]):
                Odometry(header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id="odom"),
                        child_frame_id = "base_link",
                        pose=PoseWithCovariance(
                            pose=geometry_msgs.msg.Pose(
                                position=Point(
                                    x=self.pose.x,
                                    y=self.pose.y))),
                        twist=TwistWithCovariance(
                            twist=Twist(
                                linear=Vector3(
                                    x=0.0,
                                    y=0.0),
                                angular=Vector3(z=0.0))))
            joint_state.position = [self.tilt_angle,
                                    self.stem_ang,
                                    self.accum_dist]
            joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_pub.publish(joint_state)


    def pose_callback(self, pose):
        """
        Pose callback to update self.pose, self.prev_pose, and odom_base_tf.

        Updates the necessary arguments as the turtle moves in turtlesim

        Args:
        ----
        pose : (turtlesim_msgs/msg/Pose) - the current pose of the turtle

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
        """
        Goal pose callback to update self.goalpose and self.state if necessary.

        Updates the robot's goal pose when a message is published

        Args:
        ----
        goalpose : (geometry_msgs/msg/PoseStamped) - the goal pose the robot
        should move to

        """
        if type(goalpose) != type([]) and self.goalpose != goalpose:
            self.prev_goalpose = self.goalpose
            self.goalpose = goalpose
            self.state = State.MOVING

    def tilt_callback(self, tilt):
        """
        Tilt callback to update self.tilt_angle.

        Updates tilt angle if the angle of the platform has changed

        Args:
        ----
        tilt : (turtle_brick_interfaces/msg/Tilt) - the current angle of
        the robot's platform

        """
        if self.tilt_angle != tilt.angle:
            self.tilt_angle = tilt.angle


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
