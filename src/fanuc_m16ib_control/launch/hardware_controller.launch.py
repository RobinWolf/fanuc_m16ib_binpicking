from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
#define the packages for driver and description
    description_package = "fanuc_m16ib_description"
    control_package = "fanuc_m16ib_control"

#declare launch arguments (can be passed in the command line while launching)
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='fanuc_m16ib_',
            description="Prefix for the links and joints in the robot cell",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value='true',
            description="start the robot with fake (mock) hardware or real controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='true',
            description="enable ros2 control for hardware interface (realtime control, mock, simulated or real hardware)",
        )
    )



    tf_prefix = LaunchConfiguration("tf_prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")



#define the robot description content
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

    #load the description and the controllers from desired package
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 

    #load the controller manager yaml
    robot_controller_config = PathJoinSubstitution([FindPackageShare(control_package), "config", "hardware_controllers.yaml"])


    # define the nodes to launch 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controller_config],
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", "-c", "/controller_manager"],          
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)