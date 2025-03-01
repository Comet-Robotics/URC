// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_example_1/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Include necessary serial port libraries
#include <iostream>
#include <boost/asio.hpp>

namespace ros2_control_demo_example_1
{
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.RRBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  try {
    hw_start_sec_ = std::stod(info_.hardware_parameters.at("example_param_hw_start_duration_sec"));
    hw_stop_sec_ = std::stod(info_.hardware_parameters.at("example_param_hw_stop_duration_sec"));
    hw_slowdown_ = std::stod(info_.hardware_parameters.at("example_param_hw_slowdown"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Error parsing hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize serial port parameters from the hardware info
  try {
    serial_port_name_ = info_.hardware_parameters.at("serial_port");
    serial_baud_rate_ = std::stoi(info_.hardware_parameters.at("serial_baud_rate"));
  } catch (const std::out_of_range & e) {
    RCLCPP_FATAL(get_logger(), "Missing serial port parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument & e) {
    RCLCPP_FATAL(get_logger(), "Invalid serial port parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  // Initialize serial port
  try {
    serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_, serial_port_name_);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(serial_baud_rate_));
  } catch (const boost::system::system_error & error) {
    RCLCPP_FATAL(get_logger(), "Error opening serial port %s: %s", serial_port_name_.c_str(), error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read sensor data from the serial port
  std::string sensor_data;
  try {
    boost::asio::read_until(*serial_port_, serial_buf_, '\n');
    sensor_data = boost::asio::buffer_cast<const char *>(serial_buf_.data());
    serial_buf_.consume(sensor_data.length());

    // Parse the sensor data (assuming comma-separated values)
    std::stringstream ss(sensor_data);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < hw_states_.size()) {
      try {
        hw_states_[i] = static_cast<float>(std::stod(value)) / 94718.5f;
        i++;
      } catch (const std::invalid_argument & e) {
        RCLCPP_WARN(get_logger(), "Invalid sensor data: %s", value.c_str());
      }
    }
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR(get_logger(), "Error reading from serial port: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_states_[i] << " for joint '" << info_.joints[i].name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write commands to the serial port
  std::stringstream command_string;
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    command_string << i + 1 << " " << static_cast<int>(hw_commands_[i]*(37000/2)) << " "; // Format: <motor_number> <position>
  }
  command_string << "\n"; // Add newline character for termination

  try {
    boost::asio::write(*serial_port_, boost::asio::buffer(command_string.str()));
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR(get_logger(), "Error writing to serial port: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_commands_[i] << " for joint '" << info_.joints[i].name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_1::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)