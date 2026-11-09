#include <memory>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
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
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

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

double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::sqrt(
      (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// The part of the path still ahead of the arm: everything after the nearest waypoint, so
// resuming or replacing a path never sends the arm back. The last waypoint always stays.
std::vector<geometry_msgs::msg::Pose> waypoints_ahead(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const geometry_msgs::msg::Point& current) {
  size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const double d = distance(waypoints[i].position, current);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  const size_t start = std::min(nearest + 1, waypoints.size() - 1);
  return {waypoints.begin() + static_cast<std::ptrdiff_t>(start), waypoints.end()};
}

double path_length(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const geometry_msgs::msg::Point& start) {
  double length = 0.0;
  geometry_msgs::msg::Point previous = start;
  for (const auto& pose : waypoints) {
    length += distance(previous, pose.position);
    previous = pose.position;
  }
  return length;
}

// Stretch a trajectory so it averages speed_m_s. It only ever slows down, so the planner's
// velocity and acceleration limits still hold whatever speed is asked for.
double slow_to_speed(
    moveit_msgs::msg::RobotTrajectory& trajectory, double length_m, double speed_m_s) {
  auto& points = trajectory.joint_trajectory.points;
  if (points.empty() || speed_m_s <= 0.0) {
    return 1.0;
  }
  const auto& last = points.back().time_from_start;
  const double duration = last.sec + last.nanosec * 1e-9;
  if (duration <= 0.0) {
    return 1.0;
  }
  const double factor = length_m / speed_m_s / duration;
  if (factor <= 1.0) {
    return 1.0;
  }
  for (auto& point : points) {
    const double t = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) * factor;
    point.time_from_start.sec = static_cast<int32_t>(std::floor(t));
    point.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
    for (auto& v : point.velocities) {
      v /= factor;
    }
    for (auto& a : point.accelerations) {
      a /= factor * factor;
    }
  }
  return factor;
}

// One piece of the commanded path. Between two waypoints MoveIt puts the hand on the straight
// line and the orientation on the slerp, both at the same fraction t.
struct PathSegment {
  Eigen::Vector3d p0, p1;
  Eigen::Quaterniond q0, q1;
};

Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

// How far a hand pose is from the commanded path, in multiples of the allowed deviation.
// Above 1 the hand is off the path.
double path_deviation(
    const Eigen::Isometry3d& pose, const std::vector<PathSegment>& segments,
    double max_position, double max_angle, double& position_off, double& angle_off) {
  const Eigen::Vector3d p = pose.translation();
  const Eigen::Quaterniond q(pose.rotation());
  double best = std::numeric_limits<double>::max();
  constexpr int kSamples = 50;
  for (const auto& s : segments) {
    const Eigen::Vector3d d = s.p1 - s.p0;
    const double len2 = d.squaredNorm();
    const double projected = len2 > 1e-12 ? std::clamp((p - s.p0).dot(d) / len2, 0.0, 1.0) : 0.0;
    for (int k = -1; k <= kSamples; ++k) {
      const double t = k < 0 ? projected : static_cast<double>(k) / kSamples;
      const double position_error = (p - (s.p0 + t * d)).norm() / max_position;
      const double angle_error = q.angularDistance(s.q0.slerp(t, s.q1)) / max_angle;
      const double deviation = std::max(position_error, angle_error);
      if (deviation < best) {
        best = deviation;
        position_off = position_error * max_position;
        angle_off = angle_error * max_angle;
      }
    }
  }
  return best;
}

// Hand pose in the planning frame, which is the frame the waypoints are given in.
Eigen::Isometry3d hand_pose(
    moveit::core::RobotState& state, const std::string& planning_frame, const std::string& ee_link,
    const trajectory_msgs::msg::JointTrajectory& trajectory, const std::vector<double>& positions) {
  state.setVariablePositions(trajectory.joint_names, positions);
  state.update();
  return state.getFrameTransform(planning_frame).inverse() * state.getGlobalLinkTransform(ee_link);
}

// Number of leading points of the trajectory over which the hand stays on the commanded path,
// also checked between points the way the controller interpolates the joints. Equal to the
// number of points when the whole trajectory is on the path.
size_t points_on_path(
    const moveit::core::RobotModelConstPtr& model, const std::string& planning_frame,
    const std::string& ee_link, const trajectory_msgs::msg::JointTrajectory& trajectory,
    const std::vector<geometry_msgs::msg::Pose>& waypoints, double max_position,
    double max_angle, double& position_off, double& angle_off) {
  const auto& points = trajectory.points;
  position_off = 0.0;
  angle_off = 0.0;
  if (points.empty()) {
    return 0;
  }
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  const Eigen::Isometry3d start = hand_pose(state, planning_frame, ee_link, trajectory, points[0].positions);
  std::vector<PathSegment> segments;
  Eigen::Vector3d p_prev = start.translation();
  Eigen::Quaterniond q_prev(start.rotation());
  for (const auto& w : waypoints) {
    const Eigen::Vector3d p(w.position.x, w.position.y, w.position.z);
    const Eigen::Quaterniond q = to_eigen(w.orientation);
    segments.push_back({p_prev, p, q_prev, q});
    p_prev = p;
    q_prev = q;
  }

  constexpr int kSubsteps = 10;
  std::vector<double> positions(points[0].positions.size());
  for (size_t i = 1; i < points.size(); ++i) {
    const auto& a = points[i - 1].positions;
    const auto& b = points[i].positions;
    for (int k = 1; k <= kSubsteps; ++k) {
      const double alpha = static_cast<double>(k) / kSubsteps;
      for (size_t j = 0; j < positions.size(); ++j) {
        positions[j] = a[j] + alpha * (b[j] - a[j]);
      }
      const double deviation = path_deviation(
          hand_pose(state, planning_frame, ee_link, trajectory, positions), segments,
          max_position, max_angle, position_off, angle_off);
      if (deviation > 1.0) {
        return i;
      }
    }
  }
  return points.size();
}

// Distance the hand travels over the first `count` points.
double hand_path_length(
    const moveit::core::RobotModelConstPtr& model, const std::string& planning_frame,
    const std::string& ee_link, const trajectory_msgs::msg::JointTrajectory& trajectory,
    size_t count) {
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  double length = 0.0;
  Eigen::Vector3d previous = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < count && i < trajectory.points.size(); ++i) {
    const Eigen::Vector3d position =
        hand_pose(state, planning_frame, ee_link, trajectory, trajectory.points[i].positions)
            .translation();
    if (i > 0) {
      length += (position - previous).norm();
    }
    previous = position;
  }
  return length;
}

// computeCartesianPath solves IK separately for every eef_step. When the solver cannot converge
// from its seed it restarts from a random one, and can return another arm configuration for the
// same hand pose. The joints are then interpolated between the two, the hand swings off the
// path and comes back, and the garment is pulled off the tool. So the returned trajectory is
// checked by forward kinematics: the hand must stay within max_position (m) and max_angle (rad)
// of the commanded straight-line path, including between trajectory points. The elbow may move
// as it likes. The trajectory is cut before the first point that leaves the path.
// Returns the fraction of the path that is kept.
double compute_on_path_cartesian_path(
    rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double eef_step, double jump_threshold, double max_position, double max_angle,
    double velocity_scaling, double accel_scaling,
    moveit_msgs::msg::RobotTrajectory& trajectory) {
  const double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory, true);
  if (max_position <= 0.0 || max_angle <= 0.0 || fraction <= 0.0) {
    return fraction;
  }
  auto& joint_trajectory = trajectory.joint_trajectory;
  const auto model = move_group.getRobotModel();
  const auto planning_frame = move_group.getPlanningFrame();
  std::string ee_link = move_group.getEndEffectorLink();
  if (ee_link.empty()) {
    ee_link = move_group.getLinkNames().back();
  }

  double position_off = 0.0;
  double angle_off = 0.0;
  const size_t total = joint_trajectory.points.size();
  const size_t kept = points_on_path(
      model, planning_frame, ee_link, joint_trajectory, waypoints, max_position, max_angle, position_off,
      angle_off);
  if (kept == total) {
    return fraction;
  }

  const double full_length = hand_path_length(model, planning_frame, ee_link, joint_trajectory, total);
  const double kept_fraction = (kept < 2 || full_length <= 0.0)
      ? 0.0
      : fraction * hand_path_length(model, planning_frame, ee_link, joint_trajectory, kept) / full_length;
  RCLCPP_WARN(
      node->get_logger(),
      "The hand would leave the path after point %zu/%zu (usually IK switching arm "
      "configuration, e.g. at a joint limit): "
      "%.4f m and %.3f rad off (allowed %.4f m, %.3f rad). Cutting the path there: %.3f of "
      "it is kept.",
      kept, total, position_off, angle_off, max_position, max_angle, kept_fraction);
  if (kept < 2) {
    joint_trajectory.points.clear();
    return 0.0;
  }

  // Timing was computed for the whole path, so the cut point would still be moving: retime.
  joint_trajectory.points.resize(kept);
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.setVariablePositions(joint_trajectory.joint_names, joint_trajectory.points.front().positions);
  start.update();
  robot_trajectory::RobotTrajectory cut(model, move_group.getName());
  cut.setRobotTrajectoryMsg(start, trajectory);
  // Small path tolerance: the kept points are already checked, do not blend corners off them.
  trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.001, 0.1);
  if (!totg.computeTimeStamps(cut, velocity_scaling, accel_scaling)) {
    RCLCPP_WARN(node->get_logger(), "Could not retime the cut path; dropping it.");
    joint_trajectory.points.clear();
    return 0.0;
  }
  cut.getRobotTrajectoryMsg(trajectory);
  if (points_on_path(model, planning_frame, ee_link, joint_trajectory, waypoints, max_position,
                     max_angle, position_off, angle_off) != joint_trajectory.points.size()) {
    RCLCPP_WARN(node->get_logger(), "The retimed cut path leaves the path; dropping it.");
    joint_trajectory.points.clear();
    return 0.0;
  }
  return kept_fraction;
}

// Latest message on joint_states, kept by this node's own subscription. MoveGroupInterface's
// state monitor never delivered a state in this node (getCurrentPose() logged "Failed to fetch
// current robot state" and returned the identity pose, which turns the hand upside down).
struct JointStateCache {
  std::mutex mutex;
  sensor_msgs::msg::JointState::SharedPtr msg;
  rclcpp::Time received;
};

// Current arm state from the cache. False when there is no joint state from the last second or
// it lacks a joint of the group.
bool current_state(
    rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group,
    JointStateCache& cache, moveit::core::RobotState& state) {
  sensor_msgs::msg::JointState::SharedPtr msg;
  rclcpp::Time received;
  {
    std::lock_guard<std::mutex> lock(cache.mutex);
    msg = cache.msg;
    received = cache.received;
  }
  if (!msg || (node->now() - received).seconds() > 1.0) {
    return false;
  }
  state.setToDefaultValues();
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (state.getRobotModel()->hasJointModel(msg->name[i])) {
      state.setVariablePosition(msg->name[i], msg->position[i]);
    }
  }
  for (const auto& name : move_group.getActiveJoints()) {
    if (std::find(msg->name.begin(), msg->name.end(), name) == msg->name.end()) {
      return false;
    }
  }
  state.update();
  return true;
}

// Current hand pose in the planning frame, the frame the waypoints are given in.
bool current_hand_pose(
    rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group,
    JointStateCache& cache, geometry_msgs::msg::Pose& pose) {
  moveit::core::RobotState state(move_group.getRobotModel());
  if (!current_state(node, move_group, cache, state)) {
    return false;
  }
  const Eigen::Isometry3d hand =
      state.getFrameTransform(move_group.getPlanningFrame()).inverse() *
      state.getGlobalLinkTransform(move_group.getEndEffectorLink());
  const Eigen::Quaterniond q(hand.rotation());
  pose.position.x = hand.translation().x();
  pose.position.y = hand.translation().y();
  pose.position.z = hand.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return true;
}

enum class ExecutionEnd { kFinished, kAttentionLost, kNewPath, kFailed };

ExecutionEnd execute_until_interrupted(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::atomic<bool>& attention_ok,
    bool pause_when_not_attention,
    const std::atomic<uint64_t>& latest_path,
    uint64_t running_path,
    const std::chrono::milliseconds& poll_period) {
  std::atomic<bool> execute_done{false};
  moveit::core::MoveItErrorCode execute_result = moveit::core::MoveItErrorCode::FAILURE;
  std::thread exec_thread([&move_group, &plan, &execute_done, &execute_result]() {
    execute_result = move_group.execute(plan);
    execute_done.store(true);
  });

  auto end = ExecutionEnd::kFinished;
  while (rclcpp::ok() && !execute_done.load()) {
    if (pause_when_not_attention && !attention_ok.load()) {
      end = ExecutionEnd::kAttentionLost;
    } else if (latest_path.load() != running_path) {
      end = ExecutionEnd::kNewPath;
    } else {
      std::this_thread::sleep_for(poll_period);
      continue;
    }
    move_group.stop();
    break;
  }
  exec_thread.join();

  if (end == ExecutionEnd::kFinished && execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    return ExecutionEnd::kFailed;
  }
  return end;
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
  // How far the hand may be from the commanded straight-line path, in position (m) and
  // orientation (rad). A Cartesian path is cut before it leaves. 0 turns the check off.
  const auto max_path_deviation = node->has_parameter("max_path_deviation")
      ? node->get_parameter("max_path_deviation").as_double()
      : node->declare_parameter<double>("max_path_deviation", 0.01);
  const auto max_orientation_deviation = node->has_parameter("max_orientation_deviation")
      ? node->get_parameter("max_orientation_deviation").as_double()
      : node->declare_parameter<double>("max_orientation_deviation", 0.05);
  if (max_path_deviation <= 0.0 || max_orientation_deviation <= 0.0) {
    RCLCPP_WARN(node->get_logger(),
                "Path deviation check is off: if IK switches arm configuration mid-path, the "
                "hand swings off the path.");
  }
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

  const auto continuous_execution = node->has_parameter("continuous_execution")
      ? node->get_parameter("continuous_execution").as_bool()
      : node->declare_parameter<bool>("continuous_execution", false);
  auto cartesian_speed = node->has_parameter("cartesian_speed")
      ? node->get_parameter("cartesian_speed").as_double()
      : node->declare_parameter<double>("cartesian_speed", 0.03);
  const auto execution_status_topic = node->has_parameter("execution_status_topic")
      ? node->get_parameter("execution_status_topic").as_string()
      : node->declare_parameter<std::string>(
            "execution_status_topic", "/cartesian_trajectory_status");
  if (continuous_execution && cartesian_speed <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "cartesian_speed must be above 0, using 0.03 m/s.");
    cartesian_speed = 0.03;
  }

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

  auto joint_state_cache = std::make_shared<JointStateCache>();
  rclcpp::SubscriptionOptions joint_state_sub_options;
  joint_state_sub_options.callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [node, joint_state_cache](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_cache->mutex);
        joint_state_cache->msg = msg;
        joint_state_cache->received = node->now();
      },
      joint_state_sub_options);

  // Continuous mode: the callback only stores the newest path; a worker thread runs it as
  // one motion and can stop it for a pause or a replacement path.
  std::mutex latest_waypoints_mutex;
  std::vector<geometry_msgs::msg::Pose> latest_waypoints;
  std::atomic<uint64_t> latest_path{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_status_pub;
  if (continuous_execution) {
    execution_status_pub = node->create_publisher<std_msgs::msg::String>(
        execution_status_topic, rclcpp::QoS(10).reliable());
    RCLCPP_INFO(
        node->get_logger(), "Continuous execution on, at up to %.3f m/s.", cartesian_speed);
  }

  rclcpp::SubscriptionOptions trajectory_sub_options;
  trajectory_sub_options.callback_group = trajectory_cb_group;
  auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "cartesian_trajectory", rclcpp::SystemDefaultsQoS(),
      [node, &move_group, eef_step, jump_threshold, max_path_deviation, max_orientation_deviation,
       velocity_scaling, accel_scaling, min_fraction, planning_mode,
       use_joint_constraints, joint_constraint_tolerance, planner_id, planning_pipeline,
       lock_orientation, pause_when_not_attention, poll_period,
       attention_ok, trajectory_busy, continuous_execution, joint_state_cache, &latest_waypoints_mutex,
       &latest_waypoints, &latest_path](geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.empty()) {
          RCLCPP_WARN(node->get_logger(), "Received empty PoseArray on cartesian_trajectory");
          return;
        }

        if (continuous_execution) {
          std::lock_guard<std::mutex> lock(latest_waypoints_mutex);
          latest_waypoints = msg->poses;
          latest_path.fetch_add(1);
          return;
        }

        if (trajectory_busy->exchange(true)) {
          RCLCPP_WARN(node->get_logger(), "Trajectory already executing. Ignoring new command.");
          return;
        }

        geometry_msgs::msg::Pose current_pose;
        if (!current_hand_pose(node, move_group, *joint_state_cache, current_pose)) {
          RCLCPP_ERROR(node->get_logger(),
                       "No current joint state, so the hand pose is unknown. Ignoring the trajectory.");
          trajectory_busy->store(false);
          return;
        }
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
          trajectory_busy->store(false);
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
                trajectory_busy->store(false);
                return;
              }

              if (use_joint_constraints) {
                moveit_msgs::msg::Constraints constraints;
                constraints.name = "keep_close_to_current_joints";
                moveit::core::RobotState state(move_group.getRobotModel());
                if (!current_state(node, move_group, *joint_state_cache, state)) {
                  RCLCPP_ERROR(node->get_logger(), "No current joint state. Stopping the trajectory.");
                  trajectory_busy->store(false);
                  return;
                }
                const auto joint_names = move_group.getActiveJoints();
                for (size_t i = 0; i < joint_names.size(); ++i) {
                  moveit_msgs::msg::JointConstraint jc;
                  jc.joint_name = joint_names[i];
                  jc.position = state.getVariablePosition(joint_names[i]);
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
                trajectory_busy->store(false);
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
                trajectory_busy->store(false);
                return;
              }
              reached_target = true;
            }
          }
          RCLCPP_INFO(node->get_logger(), "Planned and executed %zu waypoint(s).", waypoints.size());
        } else {
          for (size_t waypoint_idx = 0; waypoint_idx < waypoints.size(); ++waypoint_idx) {
            const auto& target_pose = waypoints[waypoint_idx];
            bool reached_target = false;
            while (rclcpp::ok() && !reached_target) {
              wait_for_attention(node, *attention_ok, pause_when_not_attention, poll_period);
              if (!rclcpp::ok()) {
                trajectory_busy->store(false);
                return;
              }

              std::vector<geometry_msgs::msg::Pose> segment_waypoints;
              segment_waypoints.push_back(target_pose);

              moveit_msgs::msg::RobotTrajectory trajectory;
              const double fraction = compute_on_path_cartesian_path(
                  node, move_group, segment_waypoints, eef_step, jump_threshold, max_path_deviation,
                  max_orientation_deviation, velocity_scaling, accel_scaling, trajectory);
              if (fraction < min_fraction) {
                RCLCPP_WARN(node->get_logger(),
                            "Cartesian segment fraction %.3f below threshold %.3f at waypoint %zu/%zu.",
                            fraction, min_fraction, waypoint_idx + 1, waypoints.size());
                trajectory_busy->store(false);
                return;
              }

              moveit::planning_interface::MoveGroupInterface::Plan plan;
              plan.trajectory_ = trajectory;

              const auto segment_desc =
                  "cartesian waypoint " + std::to_string(waypoint_idx + 1) + "/" +
                  std::to_string(waypoints.size());
              const bool executed = execute_with_attention_pause(
                  node, move_group, plan, *attention_ok, pause_when_not_attention,
                  poll_period, segment_desc);

              if (!executed) {
                if (pause_when_not_attention && !attention_ok->load()) {
                  continue;
                }
                trajectory_busy->store(false);
                return;
              }
              reached_target = true;
            }
          }
          RCLCPP_INFO(node->get_logger(), "Cartesian trajectory executed successfully.");
        }

        trajectory_busy->store(false);
      });

  std::thread continuous_worker;
  if (continuous_execution) {
    continuous_worker = std::thread(
        [node, &move_group, eef_step, jump_threshold, max_path_deviation, max_orientation_deviation,
         velocity_scaling, accel_scaling, min_fraction, lock_orientation,
         pause_when_not_attention, poll_period, attention_ok, cartesian_speed,
         execution_status_pub, joint_state_cache, &latest_waypoints_mutex, &latest_waypoints,
         &latest_path]() {
          const auto report = [&execution_status_pub](const std::string& status) {
            std_msgs::msg::String msg;
            msg.data = status;
            execution_status_pub->publish(msg);
          };
          uint64_t running_path = 0;
          bool has_work = false;
          std::vector<geometry_msgs::msg::Pose> waypoints;

          while (rclcpp::ok()) {
            if (latest_path.load() != running_path) {
              std::lock_guard<std::mutex> lock(latest_waypoints_mutex);
              waypoints = latest_waypoints;
              running_path = latest_path.load();
              has_work = true;
            }
            if (!has_work) {
              std::this_thread::sleep_for(poll_period);
              continue;
            }

            wait_for_attention(node, *attention_ok, pause_when_not_attention, poll_period);
            if (!rclcpp::ok()) {
              break;
            }
            if (latest_path.load() != running_path) {
              continue;
            }

            geometry_msgs::msg::Pose current_pose;
            if (!current_hand_pose(node, move_group, *joint_state_cache, current_pose)) {
              RCLCPP_ERROR(node->get_logger(),
                           "No current joint state, so the hand pose is unknown. Dropping the path.");
              report("failed");
              has_work = false;
              continue;
            }
            auto ahead = waypoints_ahead(waypoints, current_pose.position);
            if (lock_orientation) {
              for (auto& pose : ahead) {
                pose.orientation = current_pose.orientation;
              }
            }
            for (auto& pose : ahead) {
              if (is_zero_orientation(pose.orientation)) {
                pose.orientation = current_pose.orientation;
              }
            }

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double fraction = compute_on_path_cartesian_path(
                node, move_group, ahead, eef_step, jump_threshold, max_path_deviation,
                max_orientation_deviation, velocity_scaling, accel_scaling, trajectory);
            if (fraction < min_fraction) {
              RCLCPP_WARN(node->get_logger(),
                          "Cartesian path fraction %.3f below threshold %.3f. Dropping the path.",
                          fraction, min_fraction);
              report("failed");
              has_work = false;
              continue;
            }
            if (fraction < 1.0) {
              RCLCPP_WARN(node->get_logger(),
                          "Cartesian path covers only %.3f of the path; the arm will stop short.",
                          fraction);
            }

            const double length = path_length(ahead, current_pose.position) * fraction;
            const double factor = slow_to_speed(trajectory, length, cartesian_speed);
            RCLCPP_INFO(node->get_logger(),
                        "Running %zu waypoint(s), %.3f m, slowed down by %.2f.",
                        ahead.size(), length, factor);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            const auto end = execute_until_interrupted(
                move_group, plan, *attention_ok, pause_when_not_attention, latest_path,
                running_path, poll_period);
            if (end == ExecutionEnd::kFinished) {
              report("finished");
              has_work = false;
            } else if (end == ExecutionEnd::kFailed) {
              RCLCPP_WARN(node->get_logger(), "Execution of the continuous path failed.");
              report("failed");
              has_work = false;
            }
          }
        });
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  if (continuous_worker.joinable()) {
    continuous_worker.join();
  }
  return 0;
}
