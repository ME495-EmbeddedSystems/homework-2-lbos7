"""Display the turtle in rviz and catch a brick."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("turtle_brick"),
                                      "run_turtle.launch.py"])
            ])
        ),
        Node(
            package="turtle_brick",
            executable="arena",
            remappings=[("visualization_marker", "visualization_marker"),
                        ("visualization_marker_array", "visualization_marker_array")],
            parameters=[PathJoinSubstitution(
                              [FindPackageShare("turtle_brick"),
                              "turtle.yaml"
                               ])]
        ),
        Node(
            package="turtle_brick",
            executable="catcher",
            remappings=[("visualization_marker", "visualization_marker"),
                        ("goal_pose", "goal_pose"),
                        ("tilt", "tilt")],
            parameters=[PathJoinSubstitution(
                              [FindPackageShare("turtle_brick"),
                              "turtle.yaml"
                               ])]
        )
        ])