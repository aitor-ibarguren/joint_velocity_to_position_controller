// Copyright (c) 2026 Aitor Ibarguren
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POSITION_CONTROLLERS__JOINT_VELOCITY_TO_POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLERS__JOINT_VELOCITY_TO_POSITION_CONTROLLER_HPP_

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Msgs
#include "std_msgs/msg/float64_multi_array.hpp"

// ROS2 Control
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Eigen
#include "Eigen/Geometry"

// Real Time Tools
#include "realtime_tools/realtime_publisher.hpp"

// Parameters
#include "joint_velocity_to_position_controller/joint_velocity_to_position_controller_params.hpp"

// Kinematics
#include "urdf/model.h"
// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace position_controllers
{
class JointVelocityToPositionController : public controller_interface::ControllerInterface
{
public:
  JointVelocityToPositionController();

  // Command interface
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // State interface
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Update function
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Lifecycle
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Robot vars
  size_t dof_;
  size_t num_cmd_joints_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_joint_limits_, upper_joint_limits_;
  Eigen::VectorXd vel_joint_limits_;
  double max_acceleration_;

  // Robot state
  Eigen::VectorXd joint_positions_;
  Eigen::VectorXd joint_velocities_;

  Eigen::VectorXd joint_velocities_prev_;

  // Robot commands
  Eigen::VectorXd joint_position_commands_;
  Eigen::VectorXd joint_velocity_commands_;

  // Parameter handlers
  joint_velocity_to_position_controller::Params params_;
  std::shared_ptr<joint_velocity_to_position_controller::ParamListener> param_listener_;

  // Control vars
  bool open_loop_;

  // Feedback vars
  bool feedback_active_;
  bool feedback_cart_pose_active_;
  std::string base_link_, tip_link_;

  // KDL
  KDL::JntArray q_;
  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_cmd_subs_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_;
  realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr feedback_pub_rt_;

  // Callbacks
  void joint_vel_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Helper functions
  bool get_joint_limits(const std::vector<std::string> & joint_names);
  bool get_fk_solver(const std::string & base_link, const std::string & tip_link);
  void read_joint_state(Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities);
  Eigen::VectorXd limit_velocity(
    const Eigen::VectorXd & joint_velocity_commands, const Eigen::VectorXd & joint_velocities_prev,
    double max_acceleration, double period);
  void publish_feedback(
    const Eigen::VectorXd & joint_positions, const Eigen::VectorXd & joint_position_commands);
};

}  // namespace position_controllers

#endif  // POSITION_CONTROLLERS__JOINT_VELOCITY_TO_POSITION_CONTROLLER_HPP_