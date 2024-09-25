#ifndef MOVEIT_WRAPPER
#define MOVEIT_WRAPPER

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_wrapper/srv/move_to_pose.hpp>
#include <moveit_wrapper/srv/move_to_joint_position.hpp>
#include <moveit_wrapper/srv/set_velocity.hpp>
#include <moveit_wrapper/srv/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <stdlib.h>

namespace moveit_wrapper
{
    class MoveitWrapper : public rclcpp::Node
    {
        public:
            MoveitWrapper(const rclcpp::NodeOptions &options);
            ~MoveitWrapper() {};
            void init_move_group();
        private:
            //create callback groups fo managing the callback from the service server nodes
            rclcpp::CallbackGroup::SharedPtr _pose_target_lin_group;
            rclcpp::CallbackGroup::SharedPtr _pose_target_ptp_group;
            rclcpp::CallbackGroup::SharedPtr _joint_position_target_group;
            rclcpp::CallbackGroup::SharedPtr _reset_planning_group_group;
            rclcpp::CallbackGroup::SharedPtr _velocity_target_group;
            
            //create instances of ros2 service instances (assigned content in src with this -> create_service)
            rclcpp::Service<moveit_wrapper::srv::MoveToPose>::SharedPtr _move_to_pose_lin;
            rclcpp::Service<moveit_wrapper::srv::MoveToPose>::SharedPtr _move_to_pose_ptp;
            rclcpp::Service<moveit_wrapper::srv::MoveToJointPosition>::SharedPtr _move_to_joint_position;
            rclcpp::Service<moveit_wrapper::srv::String>::SharedPtr _reset_planning_group;
            rclcpp::Service<moveit_wrapper::srv::SetVelocity>::SharedPtr _setVelocityScaling;
            double _veloctiy_target;

            std::string _planning_group;
            bool _i_move_group_initialized;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _move_group;

            //private member functions
            void move_to_pose_lin(const std::shared_ptr<moveit_wrapper::srv::MoveToPose::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToPose::Response> response);
            void move_to_pose_ptp(const std::shared_ptr<moveit_wrapper::srv::MoveToPose::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToPose::Response> response);
            void move_to_joint_position(const std::shared_ptr<moveit_wrapper::srv::MoveToJointPosition::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToJointPosition::Response> response);
            void reset_planning_group(const std::shared_ptr<moveit_wrapper::srv::String::Request> request,
                std::shared_ptr<moveit_wrapper::srv::String::Response> response);
            bool lin_cart(const geometry_msgs::msg::Pose &pose);
            bool ptp_cart(const geometry_msgs::msg::Pose &pose);
            bool ptp_joint(const std::vector<double> joint_position);
            void setVelocityScaling(const std::shared_ptr<moveit_wrapper::srv::SetVelocity::Request> request,
                std::shared_ptr<moveit_wrapper::srv::SetVelocity::Response> response);

    };
}


#endif