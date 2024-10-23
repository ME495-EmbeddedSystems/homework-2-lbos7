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
        self.tmr = self.create_timer(1/100, self.timer_callback)
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, 'goal_pos', self.goal_pose_callback, 10)
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
        self.tilt = 0
        self.start_x = 5.544
        self.start_y = 5.544
        self.val = 1.0

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

        # odom_base_tf = TransformStamped()
        # odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        # odom_base_tf.header.frame_id = 'odom'
        # odom_base_tf.child_frame_id = 'base_link'
        # world_odom_tf.transform.translation.x = 0
        # world_odom_tf.transform.translation.y = 0
        # self.tf_broadcaster.sendTransform(odom_base_tf)

        # joint_state = JointState()
        # joint_state.header.stamp = self.get_clock().now().to_msg()
        # joint_state.name = ['connector_to_platform', 'base_to_stem', 'stem_to_wheel']
        # joint_state.position = [0, 0, 0]

        # self.joint_state_pub.publish(joint_state)


    def timer_callback(self):
        if type(self.pose) != type([]):

            if self.pose.x > 9:
                self.val = -1.0
            elif self.pose.x < 2:
                self.val = 1.0
            self.vel_pub.publish(Twist(linear=Vector3(x=self.val, y=0.0), angular=Vector3(z=0.0)))
            joint_state = JointState()
            joint_state.name = ['connector_to_platform', 'base_to_stem', 'stem_to_wheel']
            joint_state.position = [0, 0, (self.pose.x - self.start_x)/.15]
            joint_state.velocity = [0, 0, 1/.15]

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
            self.tf_broadcaster.sendTransform(odom_base_tf)
    
    def goal_pose_callback(self, goalpose):
        self.goalpose = goalpose

    def tilt_callback(self, tilt):
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
