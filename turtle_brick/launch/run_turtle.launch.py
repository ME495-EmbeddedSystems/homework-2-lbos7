"""Run the turtle robot in rviz."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ # unnecessary
                PathJoinSubstitution([FindPackageShare("turtle_brick"),
                                      "show_turtle.launch.py"])
            ]),
            launch_arguments={"use_jsp": "none"}.items()
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            parameters=[{"holonomic" : True}]
        ),
        Node(
            package="turtle_brick",
            executable="turtle_robot",
            remappings=[("cmd_vel", "turtle1/cmd_vel"),
                        ("pose", "turtle1/pose"),
                        ("joint_states", "joint_states"),
                        ("goal_pose", "goal_pose"),
                        ("tilt", "tilt")],
            parameters=[PathJoinSubstitution(
                              [FindPackageShare("turtle_brick"),
                              "turtle.yaml"
                               ])]
        )
        ])
