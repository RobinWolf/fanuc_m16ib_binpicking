import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue



def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for the links and joints in the robot cell",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_sub",
            default_value='"sub_"',
            description="Prefix for the subframe of the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_arm",
            default_value='"arm_"',
            description="Prefix for the robotarm inside the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_grip",
            default_value='"grip_"',
            description="Prefix for the gripper inside the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value='"true"',
            description="start the robot with fake (mock) hardware or real controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value='"192.168.212.203"',
            description="The IP-Adress with which the robot hardware joins the common network",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ssid",
            default_value="DIY-Robotics",
            description="The SSID from the common network (PC and ESP must be member of this network)",
        )
    )

    

    # Initialize Arguments
    tf_prefix = LaunchConfiguration("tf_prefix")
    tf_prefix_sub = LaunchConfiguration("tf_prefix_sub")
    tf_prefix_arm = LaunchConfiguration("tf_prefix_arm")
    tf_prefix_grip = LaunchConfiguration("tf_prefix_grip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_ssid = LaunchConfiguration("robot_ssid")


    #define used packages
    description_package = "diy_robot_full_cell_description"
    moveit_package = "diy_robot_wer24_moveit"


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "joint_limits.yaml"]
    )
    initial_positions_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "initial_positions.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "cell_model.urdf.xacro"]),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "initial_positions_params:=",
            initial_positions_params,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "tf_prefix_sub:=",
            tf_prefix_sub,
            " ",
            "tf_prefix_arm:=",
            tf_prefix_arm,
            " ",
            "tf_prefix_grip:=",
            tf_prefix_grip,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_ssid:=",
            robot_ssid,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_package), "srdf", "robot.srdf.xacro"]),
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "tf_prefix_sub:=",
                tf_prefix_sub,
                " ",
                "tf_prefix_arm:=",
                tf_prefix_arm,
                " ",
                "tf_prefix_grip:=",
                tf_prefix_grip,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "config", "kinematics.yaml"]
    )


    planning_group = {"planning_group": "manipulator"}


    moveit_wrapper = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics, planning_group],
    )

    return LaunchDescription(declared_arguments + [moveit_wrapper])
