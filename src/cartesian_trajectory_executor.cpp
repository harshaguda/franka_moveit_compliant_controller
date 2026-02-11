#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {
bool is_zero_orientation(const geometry_msgs::msg::Quaternion& q) {
  return q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0;
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
                    ? node->get_parameter("joint_constraint_tolerance")
                      .as_double()
                    : node->declare_parameter<double>(
                      "joint_constraint_tolerance", 0.35);
    const auto lock_orientation = node->has_parameter("lock_orientation")
                    ? node->get_parameter("lock_orientation").as_bool()
                  : node->declare_parameter<bool>("lock_orientation", false);

  moveit::planning_interface::MoveGroupInterface move_group(node, move_group_name);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(accel_scaling);
    auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "cartesian_trajectory", rclcpp::SystemDefaultsQoS(),
      [node, &move_group, eef_step, jump_threshold, min_fraction, planning_mode,
       use_joint_constraints, joint_constraint_tolerance,
       planner_id, planning_pipeline, lock_orientation](
          geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.empty()) {
          RCLCPP_WARN(node->get_logger(), "Received empty PoseArray on cartesian_trajectory");
          return;
        }

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

        if (planning_mode == "planning") {
          for (const auto& pose : waypoints) {
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
                oc.orientation = current_pose.orientation;
                oc.absolute_x_axis_tolerance = 0.05; // Tight tolerance for dressing task
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
              return;
            }
            auto exec_result = move_group.execute(plan);
            if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_WARN(node->get_logger(), "Execution failed with code %d", exec_result.val);
              move_group.clearPoseTargets();
              move_group.clearPathConstraints();
              return;
            }
          }
          move_group.clearPoseTargets();
          move_group.clearPathConstraints();
          RCLCPP_INFO(node->get_logger(), "Planned and executed %zu waypoint(s).", waypoints.size());
        } else {
          moveit_msgs::msg::RobotTrajectory trajectory;
          double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold,
                                                            trajectory, true);
          if (fraction < min_fraction) {
            RCLCPP_WARN(node->get_logger(),
                        "Cartesian path fraction %.3f below threshold %.3f. Not executing.",
                        fraction, min_fraction);
            return;
          }

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          plan.trajectory_ = trajectory;

          auto result = move_group.execute(plan);
          if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(node->get_logger(), "Execution failed with code %d", result.val);
          } else {
            RCLCPP_INFO(node->get_logger(), "Trajectory executed successfully.");
          }
        }
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
