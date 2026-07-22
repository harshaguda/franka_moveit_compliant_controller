#include <memory>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>  // Added for synchronization signal

namespace {
bool is_zero_orientation(const geometry_msgs::msg::Quaternion& q) {
  return q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0;
}

std::string normalize_label(const std::string& input) {
  std::string out = input;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  out.erase(out.begin(), std::find_if(out.begin(), out.end(), [](unsigned char c) {
    return !std::isspace(c);
  }));
  out.erase(std::find_if(out.rbegin(), out.rend(), [](unsigned char c) {
    return !std::isspace(c);
  }).base(), out.end());
  return out;
}

bool execute_with_attention_pause(
    rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::atomic<bool>& attention_ok,
    bool pause_when_not_attention,
    const std::chrono::milliseconds& poll_period,
    const std::string& segment_desc) {
  std::atomic<bool> execute_done{false};
  moveit::core::MoveItErrorCode execute_result = moveit::core::MoveItErrorCode::FAILURE;

  std::thread exec_thread([&move_group, &plan, &execute_done, &execute_result]() {
    execute_result = move_group.execute(plan);
    execute_done.store(true);
  });

  bool interrupted_for_attention = false;
  while (rclcpp::ok() && !execute_done.load()) {
    if (pause_when_not_attention && !attention_ok.load()) {
      RCLCPP_WARN(
          node->get_logger(),
          "Attention lost during %s. Stopping active trajectory segment.",
          segment_desc.c_str());
      move_group.stop();
      interrupted_for_attention = true;
      break;
    }
    std::this_thread::sleep_for(poll_period);
  }

  if (exec_thread.joinable()) {
    exec_thread.join();
  }

  if (interrupted_for_attention) {
    return false;
  }

  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(node->get_logger(), "Execution failed with code %d", execute_result.val);
    return false;
  }

  return true;
}

void wait_for_attention(
    rclcpp::Node::SharedPtr node,
    const std::atomic<bool>& attention_ok,
    bool pause_when_not_attention,
    const std::chrono::milliseconds& poll_period) {
  if (!pause_when_not_attention) {
    return;
  }
  bool logged_pause = false;
  while (rclcpp::ok() && !attention_ok.load()) {
    if (!logged_pause) {
      RCLCPP_WARN(node->get_logger(), "Trajectory paused: waiting for user attention.");
      logged_pause = true;
    }
    std::this_thread::sleep_for(poll_period);
  }
  if (logged_pause) {
    RCLCPP_INFO(node->get_logger(), "User attention recovered. Resuming trajectory.");
  }
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("franka_cartesian_trajectory_executor", options);

  const auto move_group_name = node->has_parameter("move_group")
                                   ? node->get_parameter("move_group").as_string()
                                   : node->declare_parameter<std::string>("move_group", "fr3_arm");
  const auto eef_step = node->has_parameter("eef_step")
                            ? node->get_parameter("eef_step").as_double()
                            : node->declare_parameter<double>("eef_step", 0.01);
  const auto jump_threshold = node->has_parameter("jump_threshold")
                                  ? node->get_parameter("jump_threshold").as_double()
                                  : node->declare_parameter<double>("jump_threshold", 0.0);
  const auto min_fraction = node->has_parameter("min_fraction")
                                ? node->get_parameter("min_fraction").as_double()
                                : node->declare_parameter<double>("min_fraction", 0.65);
  const auto velocity_scaling = node->has_parameter("velocity_scaling")
                                    ? node->get_parameter("velocity_scaling").as_double()
                                    : node->declare_parameter<double>("velocity_scaling", 0.2);
  const auto accel_scaling = node->has_parameter("accel_scaling")
                                 ? node->get_parameter("accel_scaling").as_double()
                                 : node->declare_parameter<double>("accel_scaling", 0.2);
  const auto planning_mode = node->has_parameter("planning_mode")
                                 ? node->get_parameter("planning_mode").as_string()
                                 : node->declare_parameter<std::string>("planning_mode", "planning");
  const auto planner_id = node->has_parameter("planner_id")
                                 ? node->get_parameter("planner_id").as_string()
                                 : node->declare_parameter<std::string>("planner_id", "");
  const auto planning_pipeline = node->has_parameter("planning_pipeline")
                                     ? node->get_parameter("planning_pipeline").as_string()
                                     : node->declare_parameter<std::string>("planning_pipeline", "");
  const auto use_joint_constraints = node->has_parameter("use_joint_constraints")
      ? node->get_parameter("use_joint_constraints").as_bool()
      : node->declare_parameter<bool>("use_joint_constraints", true);
  const auto joint_constraint_tolerance = node->has_parameter("joint_constraint_tolerance")
      ? node->get_parameter("joint_constraint_tolerance").as_double()
      : node->declare_parameter<double>("joint_constraint_tolerance", 0.35);
  const auto lock_orientation = node->has_parameter("lock_orientation")
      ? node->get_parameter("lock_orientation").as_bool()
      : node->declare_parameter<bool>("lock_orientation", false);
  const auto attention_status_topic = node->has_parameter("attention_status_topic")
      ? node->get_parameter("attention_status_topic").as_string()
      : node->declare_parameter<std::string>("attention_status_topic", "/engagement/status");
  const auto pause_when_not_attention = node->has_parameter("pause_when_not_attention")
      ? node->get_parameter("pause_when_not_attention").as_bool()
      : node->declare_parameter<bool>("pause_when_not_attention", true);
  const auto attention_poll_ms = node->has_parameter("attention_poll_ms")
      ? node->get_parameter("attention_poll_ms").as_int()
      : node->declare_parameter<int64_t>("attention_poll_ms", 50);
  const auto attention_ok_values = node->has_parameter("attention_ok_values")
      ? node->get_parameter("attention_ok_values").as_string_array()
      : node->declare_parameter<std::vector<std::string>>(
            "attention_ok_values", {"1", "true", "paying_attention"});

  moveit::planning_interface::MoveGroupInterface move_group(node, move_group_name);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(accel_scaling);

  std::set<std::string> attention_ok_set;
  for (const auto& v : attention_ok_values) {
    attention_ok_set.insert(normalize_label(v));
  }
  if (attention_ok_set.empty()) {
    attention_ok_set.insert("1");
    attention_ok_set.insert("true");
    attention_ok_set.insert("paying_attention");
  }

  auto poll_period = std::chrono::milliseconds(std::max<int64_t>(1, attention_poll_ms));
  auto attention_ok = std::make_shared<std::atomic<bool>>(true);
  auto trajectory_busy = std::make_shared<std::atomic<bool>>(false);

  auto attention_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto trajectory_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions attention_sub_options;
  attention_sub_options.callback_group = attention_cb_group;
  auto attention_sub = node->create_subscription<std_msgs::msg::String>(
      attention_status_topic,
      rclcpp::SystemDefaultsQoS(),
      [node, attention_ok, attention_ok_set](const std_msgs::msg::String::SharedPtr msg) {
        const auto normalized = normalize_label(msg->data);
        const bool is_ok = attention_ok_set.find(normalized) != attention_ok_set.end();
        attention_ok->store(is_ok);
      },
      attention_sub_options);

  // Added: Publisher to notify the Python node when a chunk finishes executing or fails
  auto status_pub = node->create_publisher<std_msgs::msg::Bool>("trajectory_done", 10);

  rclcpp::SubscriptionOptions trajectory_sub_options;
  trajectory_sub_options.callback_group = trajectory_cb_group;
  
  // Note: status_pub is now added to the capture list
  auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "cartesian_trajectory", rclcpp::SystemDefaultsQoS(),
      [node, &move_group, eef_step, jump_threshold, min_fraction, planning_mode,
       use_joint_constraints, joint_constraint_tolerance, planner_id, planning_pipeline,
       lock_orientation, pause_when_not_attention, poll_period,
       attention_ok, trajectory_busy, status_pub](geometry_msgs::msg::PoseArray::SharedPtr msg) {
        
        if (msg->poses.empty()) {
          RCLCPP_WARN(node->get_logger(), "Received empty PoseArray on cartesian_trajectory");
          return;
        }

        // We do not want to publish "done" here, because the previous chunk is still running
        if (trajectory_busy->exchange(true)) {
          RCLCPP_WARN(node->get_logger(), "Trajectory already executing. Ignoring new command.");
          return;
        }

        // Helper function to safely unlock and notify the Python node
        auto release_lock = [trajectory_busy, status_pub]() {
          std_msgs::msg::Bool done_msg;
          done_msg.data = true; 
          status_pub->publish(done_msg);
          trajectory_busy->store(false);
        };

        auto current_pose = move_group.getCurrentPose().pose;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.reserve(msg->poses.size());

        for (const auto& p : msg->poses) {
          geometry_msgs::msg::Pose pose = p;
          if (lock_orientation || is_zero_orientation(pose.orientation)) {
            pose.orientation = current_pose.orientation;
          }
          waypoints.push_back(pose);
        }

        wait_for_attention(node, *attention_ok, pause_when_not_attention, poll_period);
        if (!rclcpp::ok()) {
          release_lock();
          return;
        }

        if (planning_mode == "planning") {
          for (size_t waypoint_idx = 0; waypoint_idx < waypoints.size(); ++waypoint_idx) {
            const auto& pose = waypoints[waypoint_idx];
            bool reached_target = false;
            while (rclcpp::ok() && !reached_target) {
              wait_for_attention(node, *attention_ok, pause_when_not_attention, poll_period);
              if (!rclcpp::ok()) {
                move_group.clearPoseTargets();
                move_group.clearPathConstraints();
                release_lock();
                return;
              }

              if (use_joint_constraints) {
                moveit_msgs::msg::Constraints constraints;
                constraints.name = "keep_close_to_current_joints";
                const auto joint_names = move_group.getJointNames();
                const auto current_values = move_group.getCurrentJointValues();
                for (size_t i = 0; i < joint_names.size() && i < current_values.size(); ++i) {
                  moveit_msgs::msg::JointConstraint jc;
                  jc.joint_name = joint_names[i];
                  jc.position = current_values[i];
                  jc.tolerance_above = joint_constraint_tolerance;
                  jc.tolerance_below = joint_constraint_tolerance;
                  jc.weight = 1.0;
                  constraints.joint_constraints.push_back(jc);
                }
                move_group.setPathConstraints(constraints);
              } else if (lock_orientation) {
                moveit_msgs::msg::Constraints constraints;
                moveit_msgs::msg::OrientationConstraint oc;
                oc.link_name = move_group.getEndEffectorLink();
                oc.header.frame_id = move_group.getPlanningFrame();
                oc.orientation = move_group.getCurrentPose().pose.orientation;
                oc.absolute_x_axis_tolerance = 0.05;
                oc.absolute_y_axis_tolerance = 0.05;
                oc.absolute_z_axis_tolerance = 0.05;
                oc.weight = 1.0;
                constraints.orientation_constraints.push_back(oc);
                move_group.setPathConstraints(constraints);
              }

              if (!planner_id.empty()) {
                move_group.setPlannerId(planner_id);
              }
              if (!planning_pipeline.empty()) {
                move_group.setPlanningPipelineId(planning_pipeline);
              }
              move_group.setPoseTarget(pose);
              moveit::planning_interface::MoveGroupInterface::Plan plan;
              auto plan_result = move_group.plan(plan);
              if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(node->get_logger(), "Planning failed with code %d", plan_result.val);
                move_group.clearPoseTargets();
                move_group.clearPathConstraints();
                release_lock();
                return;
              }

              const auto segment_desc =
                  "planned waypoint " + std::to_string(waypoint_idx + 1) + "/" +
                  std::to_string(waypoints.size());
              const bool executed = execute_with_attention_pause(
                  node, move_group, plan, *attention_ok, pause_when_not_attention,
                  poll_period, segment_desc);
              move_group.clearPoseTargets();
              move_group.clearPathConstraints();

              if (!executed) {
                if (pause_when_not_attention && !attention_ok->load()) {
                  continue;
                }
                release_lock();
                return;
              }
              reached_target = true;
            }
          }
          RCLCPP_INFO(node->get_logger(), "Planned and executed %zu waypoint(s).", waypoints.size());
        } else {
          wait_for_attention(node, *attention_ok, pause_when_not_attention, poll_period);
          if (!rclcpp::ok()) {
            release_lock();
            return;
          }

          moveit_msgs::msg::RobotTrajectory trajectory;
          
          const double fraction = move_group.computeCartesianPath(
              waypoints, eef_step, jump_threshold, trajectory, true);
              
          if (fraction < min_fraction) {
            RCLCPP_WARN(node->get_logger(),
                        "Cartesian path fraction %.3f below threshold %.3f.",
                        fraction, min_fraction);
            release_lock();
            return;
          }

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          plan.trajectory_ = trajectory;

          const std::string segment_desc = "continuous cartesian path";
          
          const bool executed = execute_with_attention_pause(
              node, move_group, plan, *attention_ok, pause_when_not_attention,
              poll_period, segment_desc);

          if (!executed) {
            RCLCPP_WARN(node->get_logger(), "Continuous trajectory execution interrupted or failed.");
          } else {
            RCLCPP_INFO(node->get_logger(), "Cartesian trajectory executed successfully.");
          }
        }

        // Final unlock and notify upon successful completion
        release_lock();
      });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}