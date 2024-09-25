import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_param_builder import load_yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def load_yaml_internal(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def opaque_test(context, *args, **kwargs):
    tf_prefix = LaunchConfiguration("tf_prefix")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    moveit_package = "fanuc_m16ib_moveit_config"
    description_package = "fanuc_m16ib_description"
    

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "urdf",
            "fanuc_m16ib_model.urdf.xacro",
        ]
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
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
    robot_description_semantic_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "srdf",
            "fanuc_m16ib.srdf.xacro",
        ]
    )
    robot_description_semantic = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_semantic_file,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )

    #### load configs and dependencies for movegroup node and wrapper ####

    # controller/ action server for trajectory execution
    controllers_file = os.path.join(
        get_package_share_directory(moveit_package),
        "config",
        "controllers.yaml",
    )

    controllers_dict = load_yaml(Path(controllers_file.perform(context)))

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_dict,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # ik-solver
    robot_description_kinematics_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "config",
            "kinematics.yaml",
        ]
    )

    # planning pipeline (algorithms and settings)
    planning_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }

    ompl_planning_yaml = load_yaml_internal(moveit_package, "config/ompl.yaml")
    planning_pipeline["move_group"].update(ompl_planning_yaml)

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    planning_group = {"planning_group": "fanuc_m16ib"}
    

    # Octomap/ Moveit Perception setup
    # octomap_config = {'octomap_frame': 'igus_camera_link_optical', 
    #                   'octomap_resolution': 0.05,
    #                   'max_range': 3.0}

    # octomap_updater_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare(moveit_package),
    #         "config",
    #         "sensor_pointcloud.yaml",
    #     ]
    # )
    # octomap_updater_config = load_yaml(Path(octomap_updater_file.perform(context)))

    moveit_args_not_concatenated = [
        {"robot_description": robot_description.perform(context)},
        {"robot_description_semantic": robot_description_semantic.perform(context)},
        moveit_controllers,
        planning_scene_monitor_parameters,
        planning_pipeline,
        {
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
        planning_pipeline,
        #octomap_config,
        #octomap_updater_config
    ]

    # Concatenate all dictionaries together, else moveitpy won't read all parameters
    moveit_args = dict()
    for d in moveit_args_not_concatenated:
        moveit_args.update(d)



    #### Nodes to launch ####
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_args,
            robot_description_kinematics_file,
        ],
    )

    moveit_wrapper_node = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics_file, planning_group],
    )


    # nodes to start
    return [
        move_group_node,
        moveit_wrapper_node,
    ]




def generate_launch_description():
#declare launch arguments (can be passed in the command line while launching)

    
    tf_prefix_arg = DeclareLaunchArgument(
            "tf_prefix",
            default_value="fanuc_m16ib_"
        )

    use_mock_hardware_arg = DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="start the robot with fake(mock) hardware or real controller",
        )
    
    generate_ros2_control_tag_arg = DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value="false",
            description="enable ros2 control for hardware interface (realtime control, mock, simulated or real hardware)",
        )


    ld = LaunchDescription()

    ld.add_action(use_mock_hardware_arg)
    ld.add_action(tf_prefix_arg)
    ld.add_action(generate_ros2_control_tag_arg)

    ld.add_action(OpaqueFunction(function=opaque_test))

    return ld