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

#include "joint_velocity_to_position_controller/joint_velocity_to_position_controller.hpp"

namespace position_controllers
{
JointVelocityToPositionController::JointVelocityToPositionController()
: controller_interface::ControllerInterface(), dof_(0), num_cmd_joints_(0)
{
}

controller_interface::CallbackReturn JointVelocityToPositionController::on_init()
{
  // Initialize the parameter handler
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ =
      std::make_shared<joint_velocity_to_position_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointVelocityToPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Check degrees of freedom
  if (dof_ == 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Degrees of freedom MUST valid positive (actual DOF %lu)", dof_);
    throw std::runtime_error("Invalid degrees of freedom");
  }

  // Reserve space for command interfaces
  conf.names.reserve(dof_ * params_.command_interfaces.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration
JointVelocityToPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Check degrees of freedom
  if (dof_ == 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Degrees of freedom MUST valid positive (actual DOF %lu)", dof_);
    throw std::runtime_error("Invalid degrees of freedom");
  }

  // Reserve space for state interfaces
  conf.names.reserve(dof_ * params_.state_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn JointVelocityToPositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Prepare the controller for activation.
  RCLCPP_INFO(get_node()->get_logger(), "Configuring JointVelocityToPositionController");

  // Update the dynamic map parameters TODO: What is this for??
  param_listener_->refresh_dynamic_parameters();

  // Get parameters from listener
  params_ = param_listener_->get_params();

  // Get DoF
  dof_ = params_.joints.size();

  // Get joint names
  joint_names_ = params_.joints;
  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints were specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get joint limits
  if (!get_joint_limits(joint_names_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error retrieving joint limits from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get & verify maximum acceleration
  max_acceleration_ = params_.max_acceleration;

  if (max_acceleration_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'max_acceleration' MUST have a positive value "
      "greater than 0.0");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get open loop
  open_loop_ = params_.open_loop;

  // Get feedback
  feedback_ = params_.feedback;

  // Log
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    RCLCPP_INFO(get_node()->get_logger(), "╠═ Joint %s limits: ", joint_names_[i].c_str());
    RCLCPP_INFO(
      get_node()->get_logger(), "║  ├─ Lower: %f - Upper: %f", lower_joint_limits_[i],
      upper_joint_limits_[i]);
    RCLCPP_INFO(get_node()->get_logger(), "║  ╰─ Velocity: %f", vel_joint_limits_[i]);
  }

  RCLCPP_INFO(get_node()->get_logger(), "╠═ Max. acceleration: %f", max_acceleration_);
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Open loop %s", open_loop_ ? "ACTIVE" : "INACTIVE");
  RCLCPP_INFO(get_node()->get_logger(), "╚═ Feedback %s", feedback_ ? "ACTIVE" : "INACTIVE");

  // Create subscriber & publisher
  joint_velocity_cmd_subs_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    std::string(get_node()->get_name()) + "/joint_vel_cmd", 1,
    std::bind(
      &JointVelocityToPositionController::joint_vel_cmd_callback, this, std::placeholders::_1));

  if (feedback_)
  {
    feedback_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      std::string(get_node()->get_name()) + "/feedback", rclcpp::SystemDefaultsQoS());
    feedback_pub_rt_ =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
        feedback_pub_);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointVelocityToPositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating JointVelocityToPositionController");

  // Initialize joint states
  joint_positions_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_prev_ = Eigen::VectorXd::Zero(dof_);

  // Initialize joint commands
  joint_position_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_ = Eigen::VectorXd::Zero(dof_);

  // Verify hardware interfaces are correctly loaded
  if (command_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces loaded");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces loaded");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize current joint positions and commands
  int idx_pos = 0;
  int idx_vel = 0;
  for (auto & state_interface : state_interfaces_)
  {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Get position
      const auto joint_position_opt = state_interface.get_optional();
      if (!joint_position_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_positions_[idx_pos] = joint_position_opt.value();
        joint_position_commands_[idx_pos] = joint_position_opt.value();
        idx_pos++;
      }
    }
    else if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      // Get velocity
      const auto joint_vel_opt = state_interface.get_optional();
      if (!joint_vel_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_velocities_[idx_vel] = joint_vel_opt.value();
        idx_vel++;
      }
    }
  }

  /// Log
  // Joint positions
  std::ostringstream oss;
  for (double d : joint_positions_) oss << d << ' ';

  RCLCPP_INFO(get_node()->get_logger(), "Initial joint positions: [ %s]'", oss.str().c_str());

  // Joint velocities
  oss.str("");
  oss.clear();
  for (double d : joint_velocities_) oss << d << ' ';

  RCLCPP_INFO(get_node()->get_logger(), "Initial joint velocities: [ %s]'", oss.str().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointVelocityToPositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating JointVelocityToPositionController");

  // Set all values to zero
  joint_positions_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_prev_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_ = Eigen::VectorXd::Zero(dof_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointVelocityToPositionController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating JointVelocityToPositionController");

  // Reset subscribers & publishers
  joint_velocity_cmd_subs_.reset();
  feedback_pub_rt_.reset();
  feedback_pub_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointVelocityToPositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Get joint positions
  if (open_loop_)
  {
    joint_positions_ = joint_position_commands_;
    joint_velocities_ = joint_velocity_commands_;
  }
  else
  {
    read_joint_state(joint_positions_, joint_velocities_);
  }

  // Manage joint velocity/acceleration
  Eigen::VectorXd joint_velocities_limited = limit_velocity(
    joint_velocity_commands_, joint_velocities_prev_, max_acceleration_, period.seconds());

  // Compute next joint positions
  joint_position_commands_ = joint_positions_ + joint_velocity_commands_ * period.seconds();

  // Clip joint position commands to joint limits
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_position_commands_[i] =
      std::clamp(joint_position_commands_[i], lower_joint_limits_[i], upper_joint_limits_[i]);
  }

  // Set joint position
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (!command_interfaces_[i].set_value(static_cast<double>(joint_position_commands_[i])))
      RCLCPP_WARN(get_node()->get_logger(), "Error setting command for joint %zu", i);
  }

  // Manage feedback
  if (feedback_) publish_feedback(joint_positions_, joint_position_commands_);

  // Store prev. joint velocities
  joint_velocities_prev_ = joint_velocities_limited;

  return controller_interface::return_type::OK;
}

void JointVelocityToPositionController::joint_vel_cmd_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // Check size
  if (msg->data.size() != dof_)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received joint velocity command MUST have %d values: "
      "Skipping command!",
      (int)dof_);
    return;
  }

  // Check velocity limits
  for (size_t i = 0; i < msg->data.size(); i++)
  {
    if (fabs(msg->data[i]) > vel_joint_limits_[i])
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Received joint velocity value of joint %s above velocity limit %f: "
        "Skipping command!",
        joint_names_[i].c_str(), vel_joint_limits_[i]);
    }
  }

  // Store values
  joint_velocity_commands_ = Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());
}

bool JointVelocityToPositionController::get_joint_limits(
  const std::vector<std::string> & joint_names)
{
  // Get URDF
  const std::string & urdf = get_robot_description();

  // Init model
  urdf::Model model;
  if (!model.initString(urdf))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed parsing URDF");
    return false;
  }

  // Define Eigen::VectorXd size
  lower_joint_limits_.resize(joint_names.size());
  upper_joint_limits_.resize(joint_names.size());
  vel_joint_limits_.resize(joint_names.size());

  // Get limits
  int idx = 0;
  for (auto & joint_name : joint_names)
  {
    // Get joint
    auto joint = model.getJoint(joint_name);
    if (!joint)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found on URDF", joint_name.c_str());
      return false;
    }

    // Get limits
    lower_joint_limits_[idx] = joint->limits->lower;
    upper_joint_limits_[idx] = joint->limits->upper;
    vel_joint_limits_[idx] = joint->limits->velocity;

    idx++;
  }

  return true;
}

void JointVelocityToPositionController::read_joint_state(
  Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities)
{
  int idx_pos = 0;
  int idx_vel = 0;

  for (auto & state_interface : state_interfaces_)
  {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Get position
      const auto joint_position_opt = state_interface.get_optional();
      if (!joint_position_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_positions[idx_pos] = joint_position_opt.value();
        idx_pos++;
      }
    }
    else if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      // Get velocity
      const auto joint_vel_opt = state_interface.get_optional();
      if (!joint_vel_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_velocities[idx_vel] = joint_vel_opt.value();
        idx_vel++;
      }
    }
  }
}

Eigen::VectorXd JointVelocityToPositionController::limit_velocity(
  const Eigen::VectorXd & joint_velocity_commands, const Eigen::VectorXd & joint_velocities_prev,
  double max_acceleration, double period)
{
  // Get dv & max dv
  Eigen::VectorXd dv = joint_velocity_commands - joint_velocities_prev;
  double max_dv = max_acceleration * period;

  // Clamp dv values
  for (int i = 0; i < dv.size(); i++)
  {
    dv[i] = std::clamp(dv[i], -max_dv, max_dv);
  }

  // Return limited velocity
  return joint_velocities_prev + dv;
}

void JointVelocityToPositionController::publish_feedback(
  const Eigen::VectorXd & joint_positions, const Eigen::VectorXd & joint_position_commands)
{
  std_msgs::msg::Float64MultiArray feedback_msg;

  // Calculate velocities
  Eigen::VectorXd joint_velocities = joint_position_commands - joint_positions;

  // Insert joint positions & velocities
  feedback_msg.data.insert(
    feedback_msg.data.end(), joint_position_commands.data(),
    joint_position_commands.data() + joint_position_commands.size());
  feedback_msg.data.insert(
    feedback_msg.data.end(), joint_velocities.data(),
    joint_velocities.data() + joint_velocities.size());

  // Publish
  feedback_pub_rt_->try_publish(feedback_msg);
}

}  // namespace position_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  position_controllers::JointVelocityToPositionController,
  controller_interface::ControllerInterface)
