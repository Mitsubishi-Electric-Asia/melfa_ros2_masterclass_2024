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
#include <std_msgs/msg/u_int8.hpp>
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

class HMINode : public rclcpp::Node
{
public:
  HMINode() : Node("hmi_")
  {
    task_command_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("task_command", 10);

    analog_data_publisher_ = this->create_publisher<melfa_msgs::msg::GpioCommand>("gpio_controller/gpio_command", 10);
    auto hmi_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options.callback_group = hmi_callback_group;

    plc_link_io_subscription_ = this->create_subscription<melfa_msgs::msg::GpioState>(
        "/gpio_controller/plc_link_io_state", rclcpp::SensorDataQoS(),
        std::bind(&HMINode::analog_data_callback, this, _1), options);
    misc3_io_subscription_ = this->create_subscription<melfa_msgs::msg::GpioState>(
        "/gpio_controller/misc3_io_state", rclcpp::SensorDataQoS(), std::bind(&HMINode::push_button_callback, this, _1),
        options);
  }

  void analog_data_callback(const melfa_msgs::msg::GpioState& msg)
  {
    uint16_t hmi_input_value = msg.input_data;
    uint16_t hmi_output_value = 0;
    float temp = (float)hmi_input_value;
    temp = temp / 100.0;
    if (hmi_input_value % 100 >= 50)
    {
      hmi_output_value = (uint16_t)ceil(temp) * 100;
    }
    else
    {
      hmi_output_value = (uint16_t)floor(temp) * 100;
    }
    auto message = melfa_msgs::msg::GpioCommand();
    message.bitid = 10800;
    message.bitmask = 0xFFFF;
    message.bit_recv_type = "MXT_IO_IN";
    message.bit_send_type = "MXT_IO_OUT";
    message.bitdata = hmi_output_value;

    analog_data_publisher_->publish(message);
  }
  void push_button_callback(const melfa_msgs::msg::GpioState& msg)
  {
    uint16_t hmi_input = msg.input_data;
    auto task_command = std_msgs::msg::UInt8();
    task_command.data = 0;
    switch (hmi_input)
    {
      case 0b0:
        break;
      case 0b1:
        task_command.data = 0b1;
        if (!pick_n_place_pid_)
        {
          pick_n_place_pid_=1;
          RCLCPP_INFO(rclcpp::get_logger("push_button_callback"), "Start received");
        }
        break;
      case 0b10:
        task_command.data = 0b10;
        RCLCPP_INFO(rclcpp::get_logger("push_button_callback"), "Pick Task received");
        break;
      case 0b100:
        task_command.data = 0b100;
        RCLCPP_INFO(rclcpp::get_logger("push_button_callback"), "Place Task received");
        break;
      case 0b1000:
        task_command.data = 0b1000;
        RCLCPP_INFO(rclcpp::get_logger("push_button_callback"), "Exit received");
        if(pick_n_place_pid_)
        {
          pick_n_place_pid_=0;
        }
        break;

      default:
        RCLCPP_WARN(rclcpp::get_logger("push_button_callback"),
                    "WARN: Invalid Task. Multiple button presses "
                    "detected.");
        break;
    }
    task_command_publisher_->publish(task_command);
  }

private:
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr plc_link_io_subscription_;
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr misc3_io_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr task_command_publisher_;
  rclcpp::Publisher<melfa_msgs::msg::GpioCommand>::SharedPtr analog_data_publisher_;
  rclcpp::SubscriptionOptions options;
  uint32_t pick_n_place_pid_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<HMINode>();

  // configure PLC_link_io controller to read analog.
  configure_io_(node_, "READ_IN", 10800, 0xffff, 0);

  // configure PLC_link_io controller to read analog.
  configure_io_(node_, "READ_IN", 12080, 0xffff, 0);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.spin();
  rclcpp::shutdown();
}