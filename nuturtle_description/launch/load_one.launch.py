from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='launch rviz2'),
        DeclareLaunchArgument('use_jsp', default_value='true',
                              description='launch joint_state_publisher'),
        DeclareLaunchArgument('color', default_value='purple',
                              description='color of turtlebot3',
                              choices=['red', 'green', 'blue', 'purple']),
        SetLaunchConfiguration('rviz_filename', ["config/basic_",
                                                 LaunchConfiguration('color'), ".rviz"]),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description":
                    Command([ExecutableInPackage("xacro", "xacro"), " ",
                            PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                                  "urdf/turtlebot3_burger.urdf.xacro"]),
                            " ", "color:=", LaunchConfiguration("color")]),
                 "frame_prefix": [LaunchConfiguration('color'), "/"]
                 }
                    ],
            namespace=LaunchConfiguration('color')
            ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_jsp'),
                                                    "' == \'true\' "])),
            namespace=LaunchConfiguration('color')
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_rviz'),
                                                    "' == \'true\' "])),
            arguments=["-d", PathJoinSubstitution(
                        [FindPackageShare("nuturtle_description"),
                         LaunchConfiguration('rviz_filename')]),
                       "-f", [LaunchConfiguration('color'), "/base_link"]],
            namespace=LaunchConfiguration('color'),
            on_exit=Shutdown()
            )
    ])
