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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include "melfa_masterclass_msgs/msg/safety_state.hpp"
#include "melfa_masterclass_msgs/msg/sensor_state.hpp"
#include "melfa_masterclass_msgs/msg/gripper_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// global variables
struct gripperstate_struc_
{
  bool double_solenoid;
  bool hand_1;
  bool hand_2;
  bool hand_3;
  bool hand_4;
  bool hand_5;
  bool hand_6;
  bool hand_7;
  bool hand_8;
};
struct safetystate_struc_
{
  bool DSI_1;
  bool DSI_2;
  bool DSI_3;
  bool DSI_4;
  bool DSI_5;
  bool DSI_6;
  bool DSI_7;
  bool DSI_8;
};
struct sensorstate_struc_
{
  bool sensor_0;
  bool sensor_1;
  bool sensor_2;
  bool sensor_3;
  bool sensor_4;
  bool sensor_5;
  bool sensor_6;
  bool sensor_7;
};
struct gripperstate_struc_ gripper_state_;
struct safetystate_struc_ safety_state_;
struct sensorstate_struc_ sensor_state_;
struct gripperstate_struc_ gripper_command_;
uint8_t task_command_;
uint32_t pid_;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");

class PnPNode : public rclcpp::Node
{
public:
  PnPNode() : Node("pnp_node_")
  {
    auto pnp_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options.callback_group = pnp_callback_group;

    gripper_command_.double_solenoid = true;

    gripper_command_publisher_ =
        this->create_publisher<melfa_masterclass_msgs::msg::GripperState>("plc_/gripper_command", 10);

    sensor_state_subscription_ = this->create_subscription<melfa_masterclass_msgs::msg::SensorState>(
        "/plc_/sensor_state", rclcpp::SensorDataQoS(), std::bind(&PnPNode::sensor_state_callback, this, _1), options);
    safety_state_subscription_ = this->create_subscription<melfa_masterclass_msgs::msg::SafetyState>(
        "/plc_/safety_state", rclcpp::SensorDataQoS(), std::bind(&PnPNode::safety_state_callback, this, _1), options);
    gripper_state_subscription_ = this->create_subscription<melfa_masterclass_msgs::msg::GripperState>(
        "/plc_/gripper_state", rclcpp::SensorDataQoS(), std::bind(&PnPNode::gripper_state_callback, this, _1), options);
    task_command_subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/task_command", rclcpp::SensorDataQoS(), std::bind(&PnPNode::task_command_callback, this, _1), options);
  }
  // sensor callback for optical sensors
  void sensor_state_callback(const melfa_masterclass_msgs::msg::SensorState& msg)
  {
    sensor_state_.sensor_0 = msg.sensor_0;
    sensor_state_.sensor_1 = msg.sensor_1;
    sensor_state_.sensor_2 = msg.sensor_2;
    sensor_state_.sensor_3 = msg.sensor_3;
    sensor_state_.sensor_4 = msg.sensor_4;
    sensor_state_.sensor_5 = msg.sensor_5;
    sensor_state_.sensor_6 = msg.sensor_6;
    sensor_state_.sensor_7 = msg.sensor_7;
  }
  // safety state callback for IEC compliant safety states
  void safety_state_callback(const melfa_masterclass_msgs::msg::SafetyState& msg)
  {
    safety_state_.DSI_1 = msg.dsi_1;
    safety_state_.DSI_2 = msg.dsi_2;
    safety_state_.DSI_3 = msg.dsi_3;
    safety_state_.DSI_4 = msg.dsi_4;
    safety_state_.DSI_5 = msg.dsi_5;
    safety_state_.DSI_6 = msg.dsi_6;
    safety_state_.DSI_7 = msg.dsi_7;
    safety_state_.DSI_8 = msg.dsi_8;
  }
  // gripper state and command
  void gripper_state_callback(const melfa_masterclass_msgs::msg::GripperState& msg)
  {
    auto pub_msg = melfa_masterclass_msgs::msg::GripperState();
    gripper_state_.double_solenoid = msg.double_solenoid;
    gripper_state_.hand_1 = msg.hand_1;
    gripper_state_.hand_2 = msg.hand_2;
    gripper_state_.hand_3 = msg.hand_3;
    gripper_state_.hand_4 = msg.hand_4;
    gripper_state_.hand_5 = msg.hand_5;
    gripper_state_.hand_6 = msg.hand_6;
    gripper_state_.hand_7 = msg.hand_7;
    gripper_state_.hand_8 = msg.hand_8;

    pub_msg.double_solenoid = gripper_command_.double_solenoid;
    pub_msg.hand_1 = gripper_command_.hand_1;
    pub_msg.hand_2 = gripper_command_.hand_2;
    pub_msg.hand_3 = gripper_command_.hand_3;
    pub_msg.hand_4 = gripper_command_.hand_4;
    pub_msg.hand_5 = gripper_command_.hand_5;
    pub_msg.hand_6 = gripper_command_.hand_6;
    pub_msg.hand_7 = gripper_command_.hand_7;
    pub_msg.hand_8 = gripper_command_.hand_8;

    gripper_command_publisher_->publish(pub_msg);
  }
  // receive state instruction from hmi
  void task_command_callback(const std_msgs::msg::UInt8& msg)
  {
    task_command_ = msg.data;
  }

private:
  rclcpp::NodeOptions node_options;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr task_command_subscription_;
  rclcpp::Subscription<melfa_masterclass_msgs::msg::SensorState>::SharedPtr sensor_state_subscription_;
  rclcpp::Subscription<melfa_masterclass_msgs::msg::SafetyState>::SharedPtr safety_state_subscription_;
  rclcpp::Subscription<melfa_masterclass_msgs::msg::GripperState>::SharedPtr gripper_state_subscription_;
  rclcpp::Publisher<melfa_masterclass_msgs::msg::GripperState>::SharedPtr gripper_command_publisher_;

  rclcpp::SubscriptionOptions options;
};

int main(int argc, char** argv)
{
  //  Initializes the ROS C++ client library
  rclcpp::init(argc, argv);

  // Spins up a MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Options to customize the behavior of the node
  rclcpp::NodeOptions node_options;
  // Sets an option for the node to automatically declare parameters from overrides
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_node", node_options);

  auto pnp_node_ = std::make_shared<PnPNode>();

  // Adds the node to executor
  executor.add_node(move_group_node);
  executor.add_node(pnp_node_);

  // Detach makes the thread to run individually in the background
  std::thread([&executor]() { executor.spin(); }).detach();

  // Define planning group
  std::string param_planning_group = move_group_node->get_parameter("planning_group").as_string();
  // std::string param_planning_group = "rv7frlxy";
  static const std::string PLANNING_GROUP = param_planning_group;
  RCLCPP_INFO(LOGGER, "Using planning group: %s", param_planning_group.c_str());
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Define Joint Model Group to extract the set of joints to control from specific planning group
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Class to describe the Planning Scene [Helps to add objects to the planning environment]
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::string EEF_FRAME_ID = move_group_node->get_parameter("EEF_FRAME_ID").as_string();
  std::string BASE_FRAME_ID = move_group_node->get_parameter("BASE_FRAME_ID").as_string();
  std::string FLANGE_FRAME_ID = move_group_node->get_parameter("FLANGE_FRAME_ID").as_string();

  // Current set of Joint Values
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group.setMaxVelocityScalingFactor(move_group_node->get_parameter("velocity_scaling").as_double());
  move_group.setMaxAccelerationScalingFactor(move_group_node->get_parameter("acceleration_scaling").as_double());
  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId("LIN");
  move_group.setGoalPositionTolerance(0.0001);
  move_group.setGoalJointTolerance(0.001);

  geometry_msgs::msg::Pose home0_pose;

  // Add gripper
  moveit_msgs::msg::CollisionObject gripper;
  auto setupGripper = [&]() {
    gripper.id = "gripper";
    gripper.header.frame_id = EEF_FRAME_ID;

    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = cylinder.CYLINDER;
    cylinder.dimensions = { 0.167, 0.047 };

    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.position.x = 0;
    cylinder_pose.position.y = 0;
    cylinder_pose.position.z = 0.167 / 2.0 + 0.001;
    cylinder_pose.orientation.w = 1.0;

    gripper.primitives.push_back(cylinder);
    gripper.primitive_poses.push_back(cylinder_pose);
    gripper.operation = gripper.ADD;
    gripper.subframe_names.resize(1);
    gripper.subframe_poses.resize(1);
    gripper.subframe_names[0] = "tcp";
    gripper.subframe_poses[0].position.z = 0.168 / 2.0;

    std::vector<std::string> touch_links;
    touch_links.push_back(FLANGE_FRAME_ID);
    planning_scene_interface.applyCollisionObject(gripper);
    if (move_group.attachObject(gripper.id, EEF_FRAME_ID, touch_links))
      RCLCPP_INFO(LOGGER, "Add gripper");
  };
  // Add SLP2 area
  auto setupSLP2 = [&]() {
    moveit_msgs::msg::CollisionObject slp2_back;
    moveit_msgs::msg::CollisionObject slp2_front;
    moveit_msgs::msg::CollisionObject slp2_bottom;
    moveit_msgs::msg::CollisionObject slp2_top;
    moveit_msgs::msg::CollisionObject slp2_left;
    moveit_msgs::msg::CollisionObject slp2_right;

    geometry_msgs::msg::Pose pose;

    // back wall
    slp2_back.id = "back";
    slp2_back.header.frame_id = BASE_FRAME_ID;
    slp2_back.primitives.resize(1);
    slp2_back.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    slp2_back.primitives[0].dimensions = { 0.01, 1.0, 0.9 };

    pose.position.x = -0.4;
    pose.position.y = 0.0;
    pose.position.z = 0.45;
    pose.orientation.w = 1.0;
    slp2_back.pose = pose;

    // front wall
    slp2_front.id = "front";
    slp2_front.header.frame_id = BASE_FRAME_ID;
    slp2_front.primitives.resize(1);
    slp2_front.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    slp2_front.primitives[0].dimensions = { 0.01, 1.0, 0.9 };

    pose.position.x = 0.8;
    pose.position.y = 0.0;
    pose.position.z = 0.45;
    pose.orientation.w = 1.0;
    slp2_front.pose = pose;

    // bottom wall
    slp2_bottom.id = "bottom";
    slp2_bottom.header.frame_id = BASE_FRAME_ID;
    slp2_bottom.primitives.resize(1);
    slp2_bottom.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    slp2_bottom.primitives[0].dimensions = { 1.2, 1.0, 0.01 };

    pose.position.x = 0.2;
    pose.position.y = 0.0;
    pose.position.z = -0.01;
    pose.orientation.w = 1.0;
    slp2_bottom.pose = pose;

    // top wall
    slp2_top.id = "top";
    slp2_top.header.frame_id = BASE_FRAME_ID;
    slp2_top.primitives.resize(1);
    slp2_top.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    slp2_top.primitives[0].dimensions = { 1.2, 1.0, 0.01 };

    pose.position.x = 0.2;
    pose.position.y = 0.0;
    pose.position.z = 0.9;
    pose.orientation.w = 1.0;
    slp2_top.pose = pose;

    // left wall
    slp2_left.id = "left";
    slp2_left.header.frame_id = BASE_FRAME_ID;
    slp2_left.primitives.resize(1);
    slp2_left.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    slp2_left.primitives[0].dimensions = { 1.2, 0.01, 0.9 };

    pose.position.x = 0.2;
    pose.position.y = 0.5;
    pose.position.z = 0.45;
    pose.orientation.w = 1.0;
    slp2_left.pose = pose;

    // right wall
    slp2_right.id = "right";
    slp2_right.header.frame_id = BASE_FRAME_ID;
    slp2_right.primitives.resize(1);
    slp2_right.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    slp2_right.primitives[0].dimensions = { 1.2, 0.01, 0.9 };

    pose.position.x = 0.2;
    pose.position.y = -0.5;
    pose.position.z = 0.45;
    pose.orientation.w = 1.0;
    slp2_right.pose = pose;

    int success_count = 0;
    if (planning_scene_interface.applyCollisionObject(slp2_back))
    {
      success_count++;
    }
    sleep(0.1);
    // if(planning_scene_interface.applyCollisionObject(slp2_front))
    // {
    //   success_count++;
    // }
    // sleep(0.1);
    if (planning_scene_interface.applyCollisionObject(slp2_bottom))
    {
      success_count++;
    }
    sleep(0.1);
    if (planning_scene_interface.applyCollisionObject(slp2_top))
    {
      success_count++;
    }
    sleep(0.1);
    if (planning_scene_interface.applyCollisionObject(slp2_left))
    {
      success_count++;
    }
    sleep(0.1);
    if (planning_scene_interface.applyCollisionObject(slp2_right))
    {
      success_count++;
    }
    if (success_count > 4)
    {
      RCLCPP_INFO(LOGGER, "Add SLP2 successful");
    }
  };
  // Start Task
  auto startTask = [&]() {
    std::vector<double> joint_home0;
    current_state->copyJointGroupPositions(joint_model_group, joint_home0);
    // home0
    joint_home0[0] = 0.0000;
    joint_home0[1] = 0.0000;
    joint_home0[2] = 1.57079;
    joint_home0[3] = 0.0000;
    joint_home0[4] = 1.57079;
    joint_home0[5] = 1.57079;
    RCLCPP_INFO(LOGGER, "init joint_home0");

    bool plan_fail = false;

    // Set home0 pose
    move_group.setJointValueTarget(joint_home0);
    RCLCPP_INFO(LOGGER, "set joint_home0 target");
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "plan");

    RCLCPP_INFO(LOGGER, ("Start position"));
    if (success)
    {
      move_group.execute(plan);
      RCLCPP_INFO(LOGGER, "execute");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Planning failed!");
      plan_fail = true;
    }
    if (plan_fail)
    {
      RCLCPP_ERROR(LOGGER, "Planning failed");
      return;
    }
    else
    {
      home0_pose = move_group.getCurrentPose().pose;
    }
  };
  // Pick Task
  auto pickTask = [&]() {
    std::vector<geometry_msgs::msg::Pose> task_points;
    // Approach
    auto pose_1 = home0_pose;
    pose_1.position.y -= 0.35;
    pose_1.position.z -= 0.2;
    task_points.push_back(pose_1);

    // Pick
    auto pose_2 = pose_1;
    pose_2.position.z -= 0.15;
    task_points.push_back(pose_2);

    // Retreat
    task_points.push_back(pose_1);

    bool plan_fail = false;
    int i = 0;
    for (i = 0; i < 3; i++)
    {
      if (plan_fail)
      {
        break;
      }
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose = task_points[i];
      move_group.setJointValueTarget(msg.pose, EEF_FRAME_ID);
      auto const [success, plan] = [&move_group] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
      }();
      if (success)
      {
        move_group.execute(plan);
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        plan_fail = true;
      }
      if (i == 1)
      {
        RCLCPP_INFO(LOGGER, "Close Gripper");
        gripper_command_.double_solenoid = true;
        gripper_command_.hand_1 = true;
        gripper_command_.hand_2 = true;
        sleep(1.0);
      }
    }
    if (plan_fail)
    {
      RCLCPP_ERROR(LOGGER, "Planning failed at pose: %d", i + 1);
      return;
    }
  };
  // Place Task
  auto placeTask = [&]() {
    std::vector<geometry_msgs::msg::Pose> box_1;
    std::vector<geometry_msgs::msg::Pose> task_points;
    geometry_msgs::msg::PoseStamped msg;

    // Approach
    auto pose_1 = home0_pose;
    pose_1.position.y += 0.35;
    pose_1.position.z -= 0.2;
    task_points.push_back(pose_1);

    // Pick
    auto pose_2 = pose_1;
    pose_2.position.z -= 0.15;
    task_points.push_back(pose_2);

    // Retreat
    task_points.push_back(pose_1);
    bool plan_fail = false;

    int i = 0;
    for (i = 0; i < 3; i++)
    {
      if (plan_fail)
      {
        break;
      }
      msg.pose = task_points[i];
      msg.header.frame_id = "world";
      move_group.setJointValueTarget(msg.pose, EEF_FRAME_ID);
      auto const [success, plan] = [&move_group] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
      }();
      if (success)
      {
        move_group.execute(plan);
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        plan_fail = true;
        break;
      }
      if (i == 1)
      {
        RCLCPP_INFO(LOGGER, "Open Gripper");
        gripper_command_.double_solenoid = true;
        gripper_command_.hand_1 = false;
        gripper_command_.hand_2 = false;
        sleep(1.0);
      }
    }
    if (plan_fail)
    {
      RCLCPP_ERROR(LOGGER, "Planning failed at pose: %d", i + 1);
      return;
    }
  };
  // End Task
  auto endTask = [&]() {
    std::vector<std::string> object_ids;
    object_ids.push_back("back");
    // object_ids.push_back("front");
    object_ids.push_back("top");
    object_ids.push_back("bottom");
    object_ids.push_back("left");
    object_ids.push_back("right");
    object_ids.push_back("gripper");
    planning_scene_interface.removeCollisionObjects(object_ids);
  };
  // SLP3 collision object
  moveit_msgs::msg::CollisionObject slp3_;
  bool slp3_flag_ = false;
  auto setupSLP3 = [&]() {
    slp3_.id = "slp3_";
    slp3_.header.frame_id = BASE_FRAME_ID;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.4, 0.2, 0.35 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.6;
    box_pose.position.y = 0;
    box_pose.position.z = 0.175;
    box_pose.orientation.w = 1.0;

    slp3_.primitives.push_back(box);
    slp3_.primitive_poses.push_back(box_pose);
    slp3_.operation = slp3_.ADD;
  };
  setupGripper();
  setupSLP2();
  setupSLP3();
  bool dsi_1_flag_ = false;
  bool start_flag_ = false;
  while (task_command_ != 0b1000)
  {
    switch (task_command_)
    {
      case 0b01:
        pid_ = getppid();
        RCLCPP_INFO(LOGGER, "start task");
        startTask();
        start_flag_ = true;
        break;
      case 0b10:
        if (start_flag_)
        {
          RCLCPP_INFO(LOGGER, "pick task");
          pickTask();
        }
        else
        {
          RCLCPP_INFO(LOGGER, "select start to initialize");
        }
        break;
      case 0b100:
        if (start_flag_)
        {
          RCLCPP_INFO(LOGGER, "place task");
          placeTask();
        }
        else
        {
          RCLCPP_INFO(LOGGER, "select start to initialize");
        }
        break;
      default:
        break;
    }
    // Lower speed when DSI_1 is active
    if (safety_state_.DSI_1 && !dsi_1_flag_)
    {
      move_group.setMaxVelocityScalingFactor(0.07);
      move_group.setMaxAccelerationScalingFactor(0.1);
      dsi_1_flag_ = true;
    }
    else if (!safety_state_.DSI_1 && dsi_1_flag_)
    {
      move_group.setMaxVelocityScalingFactor(0.2);
      move_group.setMaxAccelerationScalingFactor(0.2);
      dsi_1_flag_ = false;
    }
    // Apply slp3 in moveit when DSI_2 is active
    if (safety_state_.DSI_2 && !slp3_flag_)
    {
      slp3_flag_ = true;
      move_group.setPlanningPipelineId("ompl");
      move_group.setPlannerId("RRTstar");
      planning_scene_interface.applyCollisionObject(slp3_);
      move_group.setPlanningTime(5.0);
      move_group.setNumPlanningAttempts(20);
    }
    else if (!safety_state_.DSI_2 && slp3_flag_)
    {
      std::vector<std::string> object_ids;
      object_ids.push_back("slp3_");
      planning_scene_interface.removeCollisionObjects(object_ids);
      slp3_flag_ = false;
    }
  }
  RCLCPP_INFO(LOGGER, "End received");
  endTask();

  rclcpp::shutdown();
  return 0;
}