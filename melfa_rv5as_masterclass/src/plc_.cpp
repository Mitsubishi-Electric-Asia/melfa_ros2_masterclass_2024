  //  COPYRIGHT (C) 2024 Mitsubishi Electric Corporation

  //  Licensed under the Apache License, Version 2.0 (the "License");
  //  you may not use this file except in compliance with the License.
  //  You may obtain a copy of the License at

  //      http://www.apache.org/licenses/LICENSE-2.0

  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS,
  //  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  //  See the License for the specific language governing permissions and
  //  limitations under the License.
  //  Contributor(s):
  //     Liu Muyao
  
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <bitset>
#include <std_msgs/msg/u_int8.hpp>
#include "melfa_masterclass_msgs/msg/safety_state.hpp"
#include "melfa_masterclass_msgs/msg/sensor_state.hpp"
#include "melfa_masterclass_msgs/msg/gripper_state.hpp"
#include "melfa_msgs/srv/gpio_configure.hpp"
#include "melfa_msgs/msg/gpio_state.hpp"
#include "melfa_msgs/msg/gpio_command.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

void configure_io_(rclcpp::Node::SharedPtr node_, std::string mode, uint16_t address, uint16_t mask, uint16_t data)
{
  rclcpp::Client<melfa_msgs::srv::GpioConfigure>::SharedPtr client =
      node_->create_client<melfa_msgs::srv::GpioConfigure>("/gpio_controller/configure_gpio");
  auto io_write_request = std::make_shared<melfa_msgs::srv::GpioConfigure::Request>();
  io_write_request->mode = mode;
  io_write_request->bitid = address;
  io_write_request->bitmask = mask;
  if (mode.compare("WRITE_OUT"))
  {
    io_write_request->bitdata = data;
    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "Writing...");
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "Reading...");
  }
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("configure_io_"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "service not available, waiting again...");
  }

  auto io_write_result = client->async_send_request(io_write_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, io_write_result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto service_response = io_write_result.get();

    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "Service Success");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("configure_io_"), "Failed to call service");
  }
}

class PLCNode : public rclcpp::Node
{
public:
  PLCNode() : Node("plc_")
  {
    auto plc_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options.callback_group = plc_callback_group;
    double_solenoid=true;
    gripper_command_publisher_ =
        this->create_publisher<melfa_msgs::msg::GpioCommand>("gpio_controller/gpio_command", 10);
    gripper_state_publisher_ = this->create_publisher<melfa_masterclass_msgs::msg::GripperState>("plc_/gripper_state", 10);
    optical_sensor_publisher_ = this->create_publisher<melfa_masterclass_msgs::msg::SensorState>("plc_/optical_sensor", 10);
    safety_state_publisher_ = this->create_publisher<melfa_masterclass_msgs::msg::SafetyState>("plc_/safety_state", 10);

    misc1_io_subscription_ = this->create_subscription<melfa_msgs::msg::GpioState>(
        "/gpio_controller/misc1_io_state", rclcpp::SensorDataQoS(),
        std::bind(&PLCNode::optical_sensor_callback, this, _1), options);
    safety_io_subscription_ = this->create_subscription<melfa_msgs::msg::GpioState>(
        "/gpio_controller/safety_io_state", rclcpp::SensorDataQoS(), std::bind(&PLCNode::safety_io_callback, this, _1),
        options);
    gripper_state_subscription_ = this->create_subscription<melfa_msgs::msg::GpioState>(
        "/gpio_controller/hand_io_state", rclcpp::SensorDataQoS(),
        std::bind(&PLCNode::gripper_state_callback, this, _1), options);
    gripper_command_subscription_ = this->create_subscription<melfa_masterclass_msgs::msg::GripperState>(
        "/plc_/gripper_command", rclcpp::SensorDataQoS(), std::bind(&PLCNode::gripper_command_callback, this, _1),
        options);
  }

  void gripper_state_callback(const melfa_msgs::msg::GpioState& msg)
  {
    uint16_t gripper_input = msg.output_data;
    auto gripper_state_ = melfa_masterclass_msgs::msg::GripperState();
    gripper_state_.double_solenoid = double_solenoid;

    if (double_solenoid)
    {
      if ((gripper_input & 0b10) == 0b10)
      {
        gripper_state_.hand_1 = true;
      }
      if ((gripper_input & 0b1000) == 0b1000)
      {
        gripper_state_.hand_2 = true;
      }
      if ((gripper_input & 0b100000) == 0b100000)
      {
        gripper_state_.hand_3 = true;
      }
      if ((gripper_input & 0b10000000) == 0b10000000)
      {
        gripper_state_.hand_4 = true;
      }
    }
    else
    {
      if (gripper_input & 0b1)
      {
        gripper_state_.hand_1 = true;
      }
      if (gripper_input & 0b10)
      {
        gripper_state_.hand_2 = true;
      }
      if (gripper_input & 0b100)
      {
        gripper_state_.hand_3 = true;
      }
      if (gripper_input & 0b1000)
      {
        gripper_state_.hand_4 = true;
      }
      if (gripper_input & 0b10000)
      {
        gripper_state_.hand_5 = true;
      }
      if (gripper_input & 0b100000)
      {
        gripper_state_.hand_6 = true;
      }
      if (gripper_input & 0b1000000)
      {
        gripper_state_.hand_7 = true;
      }
      if (gripper_input & 0b10000000)
      {
        gripper_state_.hand_8 = true;
      }
    }
    gripper_state_publisher_->publish(gripper_state_);
  }
  void gripper_command_callback(const melfa_masterclass_msgs::msg::GripperState& msg)
  {
    auto command_ = msg;
    std::string gripper_command_str_;
    uint16_t gripper_command_int_ = 0b0;
    double_solenoid = command_.double_solenoid;
    if (!double_solenoid)
    {
      gripper_command_str_ = std::to_string(command_.hand_1) + std::to_string(command_.hand_2) +
                             std::to_string(command_.hand_3) + std::to_string(command_.hand_4) +
                             std::to_string(command_.hand_5) + std::to_string(command_.hand_6) +
                             std::to_string(command_.hand_7) + std::to_string(command_.hand_8);

      gripper_command_int_ = std::bitset<8>(gripper_command_str_).to_ulong();
    }
    else if (double_solenoid)
    {
      if(command_.hand_1)
      {
        gripper_command_int_ = gripper_command_int_ | 0b10;
      }
      else
      {
        gripper_command_int_ = gripper_command_int_ | 0b01;
      }
      if(command_.hand_2)
      {
        gripper_command_int_ = gripper_command_int_ | 0b1000;
      }
      else
      {
        gripper_command_int_ = gripper_command_int_ | 0b0100;

      }
      if(command_.hand_3)
      {
        gripper_command_int_ = gripper_command_int_ | 0b100000;
      }
      else
      {
        gripper_command_int_ = gripper_command_int_ | 0b010000;

      }
      if(command_.hand_4)
      {
        gripper_command_int_ = gripper_command_int_ | 0b10000000;
      }
      else
      {
        gripper_command_int_ = gripper_command_int_ | 0b01000000;

      }
    }
    auto message = melfa_msgs::msg::GpioCommand();
    message.bitid = 900;
    message.bitmask = 0xFFFF;
    message.bit_recv_type = "MXT_IO_OUT";
    message.bit_send_type = "MXT_IO_OUT";
    message.bitdata = gripper_command_int_;

    gripper_command_publisher_->publish(message);
  }
  void safety_io_callback(const melfa_msgs::msg::GpioState& msg)
  {
    uint16_t safety_input = msg.input_data;
    auto message = melfa_masterclass_msgs::msg::SafetyState();

    if (!(safety_input & 0b01))
    {
      message.dsi_1 = true;
    }
    if (!(safety_input & 0b10))
    {
      message.dsi_2 = true;
    }
    if (!(safety_input & 0b100))
    {
      message.dsi_3 = true;
    }
    if (!(safety_input & 0b1000))
    {
      message.dsi_4 = true;
    }
    if (!(safety_input & 0b10000))
    {
      message.dsi_5 = true;
    }
    if (!(safety_input & 0b100000))
    {
      message.dsi_6 = true;
    }
    if (!(safety_input & 0b1000000))
    {
      message.dsi_7 = true;
    }
    if (!(safety_input & 0b10000000))
    {
      message.dsi_8 = true;
    }
    safety_state_publisher_->publish(message);
  }
  void optical_sensor_callback(const melfa_msgs::msg::GpioState& msg)
  {
    uint16_t sensor_input = msg.input_data;
    auto message = melfa_masterclass_msgs::msg::SensorState();

    if (sensor_input & 0b01)
    {
      message.sensor_0 = true;
    }
    if (sensor_input & 0b10)
    {
      message.sensor_1 = true;
    }
    if (sensor_input & 0b100)
    {
      message.sensor_2 = true;
    }
    if (sensor_input & 0b1000)
    {
      message.sensor_3 = true;
    }
    if (sensor_input & 0b10000)
    {
      message.sensor_4 = true;
    }
    if (sensor_input & 0b100000)
    {
      message.sensor_5 = true;
    }
    if (sensor_input & 0b1000000)
    {
      message.sensor_6 = true;
    }
    if (sensor_input & 0b10000000)
    {
      message.sensor_7 = true;
    }
    optical_sensor_publisher_->publish(message);
  }

private:
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr safety_io_subscription_;
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr misc1_io_subscription_;
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr gripper_state_subscription_;
  rclcpp::Subscription<melfa_masterclass_msgs::msg::GripperState>::SharedPtr gripper_command_subscription_;
  rclcpp::Publisher<melfa_msgs::msg::GpioCommand>::SharedPtr gripper_command_publisher_;
  rclcpp::Publisher<melfa_masterclass_msgs::msg::GripperState>::SharedPtr gripper_state_publisher_;
  rclcpp::Publisher<melfa_masterclass_msgs::msg::SensorState>::SharedPtr optical_sensor_publisher_;
  rclcpp::Publisher<melfa_masterclass_msgs::msg::SafetyState>::SharedPtr safety_state_publisher_;
  rclcpp::SubscriptionOptions options;
  bool double_solenoid = true;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<PLCNode>();

  // configure Misc1_io controller to read memory mapped to industrial network.
  configure_io_(node_, "READ_IN", 6000, 0xffff, 0);

  // configure safety_io controller to read safety io.
  configure_io_(node_, "READ_IN", 128, 0xffff, 0);

  // configure gripper_io controller to monitor gripper state.
  configure_io_(node_, "READ_OUT", 900, 0xffff, 0);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.spin();
  rclcpp::shutdown();
}