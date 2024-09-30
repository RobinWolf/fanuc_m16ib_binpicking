from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    moveit_package = "fanuc_m16ib_moveit_config"
    bringup_package = "fanuc_m16ib_bringup"
    control_package = "fanuc_m16ib_control"
    description_package = "fanuc_m16ib_description"

    declared_arguments = []
    # agv launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='fanuc_m16ib_',
            description="Prefix for the links and joints, can be used for multi robot szenarios",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='true',
            description="activate ros2 control nodes for hardware interaction (real bringup, not only oint states gui)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='true',
            description="use the mock hardware on the agv, because in this repo the agv is only used for collision avoidance, agv control is done in a seperate container",
        )
    )


    # general arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value='true',
            description="set to true if you want to launch moveit for the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value='true',
            description="set to true if you want to launch rviz GUI",
        )
    )

    # zivid arguments settings_yaml
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_zivid",
            default_value='true',
            description="set to true if you want to launch the zivid node for interacting with the camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "zivid_settings",
            default_value='settings_309024.yml',
            description="set the path to the yaml with camera settings, zivid default: src/zivid-ros/zivid_samples/settings/camera_settings.yml",
        )
    )#settings_309024 settings_new_11122023


    #init launch arguments, transfer to variables
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    generate_ros2_control_tag = LaunchConfiguration('generate_ros2_control_tag') 
    launch_moveit = LaunchConfiguration('launch_moveit')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_zivid = LaunchConfiguration('launch_zivid')
    zivid_settings = LaunchConfiguration('zivid_settings')





    #launch the hardware controllers (currently only mock hardware supported) --> control_node, robot_state_pub_node, joint_state_broadcaster_node, joint_trajectory_controller_node
    load_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(control_package), 'launch']), "/hardware_controller.launch.py"]),
            condition=IfCondition(launch_moveit),
            launch_arguments={
                "use_mock_hardware:=": use_mock_hardware,
                "generate_ros2_control_tag": generate_ros2_control_tag,
                "tf_prefix": tf_prefix,
            }.items(),
    )

    # launch moveit nodes -> move_group_node, moveit_wrapper_node
    load_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(moveit_package), 'launch']), "/moveit.launch.py"]),
            condition=IfCondition(launch_moveit),
            launch_arguments={
                "use_mock_hardware:=": use_mock_hardware,
                "generate_ros2_control_tag": generate_ros2_control_tag,
                "tf_prefix": tf_prefix,
            }.items(),
    )

    # launch zivid node and set config
    zivid_config_file = PathJoinSubstitution([FindPackageShare(bringup_package), "zivid", zivid_settings])
    zivid_node = Node(
        package="zivid_camera",
        executable="zivid_camera",
        parameters=[{"settings_file_path":zivid_config_file}],
        condition=IfCondition(launch_zivid)
        )


    # launch whole rviz node with stored config and motion planning parameters
    rviz_config_file = PathJoinSubstitution([FindPackageShare(bringup_package), "rviz", "moveit_and_zivid.rviz"])
    robot_description_kinematics_file = PathJoinSubstitution(   # needed for the interactive markers in rviz
        [
            FindPackageShare(moveit_package),
            "config",
            "kinematics.yaml",
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description_kinematics_file,
        ],
        condition=IfCondition(launch_rviz)
    )



    nodes_to_start = [
        #load_controllers,
        #load_moveit,
        rviz_node,
        zivid_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 