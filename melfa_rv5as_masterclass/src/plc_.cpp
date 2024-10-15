
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <std_msgs/msg/u_int8_multi_array.hpp>
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

    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "Service Sucess");
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

    gripper_command_publisher_ =
        this->create_publisher<melfa_msgs::msg::GpioCommand>("gpio_controller/gpio_command", 10);
    optical_sensor_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("optical_sensor", 10);
    safety_state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("safety_state", 10);

    misc1_io_subscription_ = this->create_subscription<melfa_msgs::msg::GpioState>(
        "/gpio_controller/misc1_io_state", rclcpp::SensorDataQoS(),
        std::bind(&PLCNode::optical_sensor_callback, this, _1), options);
    safety_io_subscription_ = this->create_subscription<melfa_msgs::msg::GpioState>(
        "/gpio_controller/safety_io_state", rclcpp::SensorDataQoS(), std::bind(&PLCNode::safety_io_callback, this, _1),
        options);
    gripper_command_subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/plc_/gripper_command", rclcpp::SensorDataQoS(), std::bind(&PLCNode::gripper_command_callback, this, _1),
        options);
  }

  void gripper_command_callback(const std_msgs::msg::UInt8MultiArray& msg)
  {
    uint8_t command_ = msg.data[0];
    uint8_t solenoid_ = msg.data[1];
    uint16_t gripper_command_data = 0x0000;
    if (solenoid_ == 1)
    {
      gripper_command_data = command_ | gripper_command_data;
    }
    else if (solenoid_ == 2)
    {
      if (command_ & 0b1)
      {
        gripper_command_data = 0b10 | gripper_command_data;
      }
      if (!(command_ & 0b1))
      {
        gripper_command_data = 0b01 | gripper_command_data;
      }
      if (command_ & 0b10)
      {
        gripper_command_data = 0b1000 | gripper_command_data;
      }
      if (!(command_ & 0b10))
      {
        gripper_command_data = 0b0100 | gripper_command_data;
      }
      if (command_ & 0b100)
      {
        gripper_command_data = 0b100000 | gripper_command_data;
      }
      if (!(command_ & 0b100))
      {
        gripper_command_data = 0b010000 | gripper_command_data;
      }
      if (command_ & 0b1000)
      {
        gripper_command_data = 0b10000000 | gripper_command_data;
      }
      if (!(command_ & 0b1000))
      {
        gripper_command_data = 0b01000000 | gripper_command_data;
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("gripper_command"), "ERROR: Invalid solenoid setting");
    }
    auto message = melfa_msgs::msg::GpioCommand();
    message.bitid = 900;
    message.bitmask = 0xFFFF;
    message.bit_recv_type = "MXT_IO_IN";
    message.bit_send_type = "MXT_IO_OUT";
    message.bitdata = gripper_command_data;

    gripper_command_publisher_->publish(message);
  }
  void safety_io_callback(const melfa_msgs::msg::GpioState& msg)
  {
    uint16_t safety_input = msg.input_data;
    auto message = std_msgs::msg::UInt8();
    safety_input = ~safety_input;
    safety_input = safety_input^0b11111100;
    message.data = safety_input;
    safety_state_publisher_->publish(message);
  }
  void optical_sensor_callback(const melfa_msgs::msg::GpioState& msg)
  {
    uint16_t sensor_input = msg.input_data;
    // discard input from first sensor
    sensor_input = sensor_input>> 1;
    auto message = std_msgs::msg::UInt8();
    message.data = sensor_input;
    optical_sensor_publisher_->publish(message);
    if (sensor_input & 0b01)
    {
      int i = 0;
    }
    if (sensor_input & 0b10)
    {
      int i = 0;
    }
  }

private:
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr safety_io_subscription_;
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr misc1_io_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr gripper_command_subscription_;
  rclcpp::Publisher<melfa_msgs::msg::GpioCommand>::SharedPtr gripper_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr optical_sensor_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr safety_state_publisher_;
  rclcpp::SubscriptionOptions options;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<PLCNode>();

  // configure Misc1_io controller to read memory mapped to industrial network.
  configure_io_(node_, "READ_IN", 6000, 0xffff, 0);

  // configure safety_io controller to read safety io.
  configure_io_(node_, "READ_IN", 128, 0xffff, 0);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.spin();
  rclcpp::shutdown();
}