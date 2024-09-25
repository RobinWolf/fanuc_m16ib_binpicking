#include <moveit_wrapper/moveit_wrapper.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

// Links for moveit sourcecode
// MovgGroup interface: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp
// PlanningScene interface: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/planning_scene_interface/src/planning_scene_interface.cpp



namespace moveit_wrapper
{
    MoveitWrapper::MoveitWrapper(const rclcpp::NodeOptions &options) : Node("moveit_wrapper", options)
    {
        //create callbacks
        _i_move_group_initialized = false;
        this->get_parameter("planning_group", _planning_group);
        _pose_target_lin_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _pose_target_ptp_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _joint_position_target_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _reset_planning_group_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _velocity_target_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //create services and bind callback functions
        _move_to_pose_lin = this->create_service<moveit_wrapper::srv::MoveToPose>("move_to_pose_lin", std::bind(&MoveitWrapper::move_to_pose_lin, this, _1, _2), rmw_qos_profile_services_default, _pose_target_lin_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_lin service initialized.");

        _move_to_pose_ptp = this->create_service<moveit_wrapper::srv::MoveToPose>("move_to_pose_ptp", std::bind(&MoveitWrapper::move_to_pose_ptp, this, _1, _2), rmw_qos_profile_services_default, _pose_target_ptp_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_ptp service initialized.");

        _move_to_joint_position = this->create_service<moveit_wrapper::srv::MoveToJointPosition>("move_to_joint_position", std::bind(&MoveitWrapper::move_to_joint_position, this, _1, _2), rmw_qos_profile_services_default, _joint_position_target_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_joint_position service initialized.");

        _reset_planning_group = this->create_service<moveit_wrapper::srv::String>("reset_planning_group", std::bind(&MoveitWrapper::reset_planning_group, this, _1, _2), rmw_qos_profile_services_default, _reset_planning_group_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "reset_planning_group service initialized.");

        _setVelocityScaling = this->create_service<moveit_wrapper::srv::SetVelocity>("setVelocityScaling", std::bind(&MoveitWrapper::setVelocityScaling, this, _1, _2), rmw_qos_profile_services_default, _velocity_target_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "setVelocityScaling service initialized.");

        rclcpp::Rate loop_rate(1);
        loop_rate.sleep();


        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Initialized.");
    }

    void MoveitWrapper::init_move_group()
    {
        static const std::string PLANNING_GROUP = _planning_group;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), PLANNING_GROUP.c_str());
        _move_group.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), _planning_group));

        // set goal planning time
        _move_group->setPlanningTime(1.0);

        // set refernce frame for planning to ur_base_link
        _move_group->setPoseReferenceFrame("igus_base_link");


        _i_move_group_initialized = true;

        rclcpp::Rate loop_rate(1000);
        loop_rate.sleep();
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Ready to receive commands.");
    }

    void MoveitWrapper::reset_planning_group(const std::shared_ptr<moveit_wrapper::srv::String::Request> request,
                std::shared_ptr<moveit_wrapper::srv::String::Response> response)
    {
        _i_move_group_initialized = false;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), request->data.c_str());
        _planning_group = request->data.c_str();
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), _planning_group.c_str());
        _move_group->stop();
        _move_group->clearPoseTargets();

        init_move_group();
        
        response->success = true;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "reset_planning_group callback executed.");
    }


    void MoveitWrapper::move_to_pose_lin(const std::shared_ptr<moveit_wrapper::srv::MoveToPose::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToPose::Response> response)
    {
        bool success = false;
        if(_i_move_group_initialized)
        {   
            _move_group->stop();
            _move_group->clearPoseTargets();
            _move_group->setStartStateToCurrentState();

            // set planning reference frame according to out robot system
            _move_group->setPoseReferenceFrame("igus_base_link");

            //https://github.com/moveit/moveit2/blob/main/moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_frame_planning.cpp

            // Set goal state
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(request->pose);


            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.001;                   
            double fraction = _move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            rclcpp::Time start = rclcpp::Clock().now();

            if (fraction >= 1.0)
            {
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Achieved %f %% of Cartesian path", fraction * 100.);
                // Compute time parameterization to also provide velocities
                robot_trajectory::RobotTrajectory rt(_move_group->getRobotModel(), _move_group->getName());
                rt.setRobotTrajectoryMsg(*_move_group->getCurrentState(), trajectory);
                trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
                // Recalculate timestamps in reference to velocityscaling factor
                bool success_timeparam = time_parameterization.computeTimeStamps(rt, _veloctiy_target);
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "scale new timestamps with %f %% of -------> new trajectory",_veloctiy_target * 100.);
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Computing time stamps %s", success_timeparam ? "SUCCEEDED" : "FAILED");
                // Store trajectory in current_plan_
                moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
                rt.getRobotTrajectoryMsg(current_plan_.trajectory_);
                current_plan_.planning_time_ = (rclcpp::Clock().now() - start).seconds();

                if(success_timeparam) {
                     success = (_move_group->execute(current_plan_)== moveit::core::MoveItErrorCode::SUCCESS);
                 }

                response->success = success;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_lin callback executed with %s", success ? "SUCCEEDED" : "FAILED");
    }


    void MoveitWrapper::move_to_pose_ptp(const std::shared_ptr<moveit_wrapper::srv::MoveToPose::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToPose::Response> response)
    {
        bool planning_success = false;
        bool success = false;

        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();
            _move_group->setStartStateToCurrentState();
                    
            // set planning reference frame according to out robot system
            _move_group->setPoseReferenceFrame("igus_base_link");

            _move_group->setPoseTarget(request->pose);
            
            for (int i = 0; i < 3; i++)
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                planning_success = (_move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "CartesianPTP planning %s", planning_success ? "SUCCEEDED" : "FAILED");
                if(planning_success) {
                    break;
                }
            }

            if(planning_success) {
                success = (_move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "CartesianPTP planning failed after 3 attempts.");
            }
        }
        response->success = success;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_ptp callback executed with %s", success ? "SUCCEEDED" : "FAILED");
    }


    void MoveitWrapper::move_to_joint_position(const std::shared_ptr<moveit_wrapper::srv::MoveToJointPosition::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToJointPosition::Response> response)
    {
        bool success = false;
        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();
            _move_group->setStartStateToCurrentState();

            success = ptp_joint(request->joint_position);
        }
        response->success = success;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_joint_position callback executed with %s", success ? "SUCCEEDED" : "FAILED");
    }


    bool MoveitWrapper::ptp_joint(const std::vector<double> joint_position)
    {
        bool success = false;
        bool planning_success = false;

        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();
            _move_group->setStartStateToCurrentState();

            _move_group->setJointValueTarget(joint_position);

            for (int i = 0; i < 3; i++)
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                planning_success = (_move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "JointPTP planning %s", planning_success ? "SUCCEEDED" : "FAILED");
                if(planning_success) {
                    break;
                }
            }

            if(planning_success) {
                success = (_move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "JointPTP planning failed after 3 attempts.");
            }
        }
        return success;
    }

    void MoveitWrapper::setVelocityScaling(const std::shared_ptr<moveit_wrapper::srv::SetVelocity::Request> request,
                std::shared_ptr<moveit_wrapper::srv::SetVelocity::Response> response)
    {
        bool success = false;
        if(_i_move_group_initialized)
        {
             // Set the maximum velocity scaling factor
            _veloctiy_target = request->velocity_scaling;
            _move_group->setMaxVelocityScalingFactor(_veloctiy_target);
            success = true;
        }
        response->success = success;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "rescale velocity to %f %% of joint_limits executed",_veloctiy_target * 100.);
    }
}