from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    description_package = "fanuc_m16ib_description"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='fanuc_m16ib_',
            description="Prefix for the links and joints in the robot cell, can be used for multi-robot cells",
        )
    )        
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value='true',
            description="start the robot with fake(mock) hardware or real controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='false',
            description="enable ros2 control for hardware interface (realtime control, mock, simulated or real hardware)",
        )
    )


    tf_prefix = LaunchConfiguration("tf_prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "fanuc_m16ib_model.urdf.xacro"]),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "generate_ros2_control_tag:=",
            generate_ros2_control_tag,
        ]
    )

    #convert robot_description_content to string, thet it can be passed as yaml to the robot_state_publisher Node --> unsafe without !
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 
    
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "rviz_config.rviz"]) # define path to rviz-config file

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    #run joint_state_publisher node, because we don't need to bringup the robot model (without no transforms !) --> only for visualizing porposes     
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)