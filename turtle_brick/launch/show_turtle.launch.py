"""Display the turtle in rviz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_jsp", default_value='gui', description="Which method of publishing joints: 'gui' (default), 'jsp', or 'none'"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                          PathJoinSubstitution(
                              #A plain urdf file is also a valid xacro file so we use xacro here for convenience only
                              [FindPackageShare("turtle_brick"),
                              "turtle.urdf.xacro"
                               ])])}
            ]
        ),
        Node(
            package=IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration("use_jsp"), "gui"),
                        if_value="joint_state_publisher_gui",
                        else_value=IfElseSubstitution(
                                        EqualsSubstitution(LaunchConfiguration("use_jsp"), "jsp"),
                                        if_value="joint_state_publisher"
                                        )
                        ),
            executable=IfElseSubstitution(
                        EqualsSubstitution(LaunchConfiguration("use_jsp"), "gui"),
                        if_value="joint_state_publisher_gui",
                        else_value=IfElseSubstitution(
                                        EqualsSubstitution(LaunchConfiguration("use_jsp"), "jsp"),
                                        if_value="joint_state_publisher"
                                        )
                        )
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("turtle_brick"), "view_robot.rviz"])]
        )
        ])