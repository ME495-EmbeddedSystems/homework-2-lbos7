"""Run the turtle robot in rviz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            parameters=[
                {"holonomic" : "True"}
            ]
        ),
        Node(
            package="turtle_brick",
            executable="arena",
            remappings=[("cmd_vel", "turtle1/cmd_vel"),
                        ("pose", "turtle1/pose"),
                        ("joint_states", "joint_states"),
                        ("goal_pose", "goal_pose"),
                        ("tilt", "tilt")],
            parameters=[PathJoinSubstitution(
                              [FindPackageShare("turtle_brick"),
                              "turtle.urdf.xacro"
                               ])]
        )
        ])