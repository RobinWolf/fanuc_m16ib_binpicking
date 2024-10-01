from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    zivid_sim_package = "zivid_simulation"

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "test_scene",
            default_value='scan2.zdf',
            description="choose the test scene which the mock hardware should use. the data must be located in the 'data' folder !",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value='true',
            description="set to true if you want to launch rviz GUI",
        )
    )


    #init launch arguments, transfer to variables
    test_scene = LaunchConfiguration("test_scene")
    launch_rviz = LaunchConfiguration('launch_rviz')


    #launch the hardware controllers (currently only mock hardware supported) --> control_node, robot_state_pub_node, joint_state_broadcaster_node, joint_trajectory_controller_node
    test_scene_path = PathJoinSubstitution([FindPackageShare(zivid_sim_package), "zivid", test_scene])
    camera_node = Node(
        package="zivid_simulation",
        executable="zivid_sim_node",
        parameters=[{"simScanPath":test_scene_path}],
        )


    # launch whole rviz node with stored config and motion planning parameters
    rviz_config_file = PathJoinSubstitution([FindPackageShare(zivid_sim_package), "rviz", "camera.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz)
    )


    nodes_to_start = [
        camera_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 