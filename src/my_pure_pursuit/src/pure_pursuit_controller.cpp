#include "my_pure_pursuit/pure_pursuit_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav2_util/node_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Đăng ký plugin với pluginlib
PLUGINLIB_EXPORT_CLASS(
  my_pure_pursuit::PurePursuitController,
  nav2_core::Controller)

namespace my_pure_pursuit
{

// ── configure ──────────────────────────────────────────────────────────────
void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  m_node        = parent;
  m_plugin_name = name;
  m_tf          = tf;

  auto node = parent.lock();
  m_logger  = node->get_logger();

  // Đọc tham số từ nav2_params.yaml (namespace = plugin name)
  nav2_util::declare_parameter_if_not_declared(
    node, m_plugin_name + ".lookahead_dist",  rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, m_plugin_name + ".linear_vel",      rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, m_plugin_name + ".angular_gain",    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, m_plugin_name + ".goal_tolerance",  rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, m_plugin_name + ".max_angular_vel", rclcpp::ParameterValue(1.0));

  node->get_parameter(m_plugin_name + ".lookahead_dist",  m_lookahead_dist);
  node->get_parameter(m_plugin_name + ".linear_vel",      m_linear_vel);
  node->get_parameter(m_plugin_name + ".angular_gain",    m_angular_gain);
  node->get_parameter(m_plugin_name + ".goal_tolerance",  m_goal_tolerance);
  node->get_parameter(m_plugin_name + ".max_angular_vel", m_max_angular_vel);

  RCLCPP_INFO(m_logger,
    "PurePursuitController configured: lookahead=%.2f m, linear_vel=%.2f m/s, "
    "angular_gain=%.2f, goal_tol=%.2f m",
    m_lookahead_dist, m_linear_vel, m_angular_gain, m_goal_tolerance);
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(m_logger, "PurePursuitController cleaned up.");
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(m_logger, "PurePursuitController activated.");
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(m_logger, "PurePursuitController deactivated.");
}

// ── setPlan ────────────────────────────────────────────────────────────────
void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  m_current_plan = path;
  RCLCPP_INFO(m_logger, "New plan received: %zu poses.", path.poses.size());
}

// ── setSpeedLimit ──────────────────────────────────────────────────────────
void PurePursuitController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    m_linear_vel *= (speed_limit / 100.0);
  } else {
    m_linear_vel = speed_limit;
  }
  RCLCPP_INFO(m_logger, "Speed limit applied: linear_vel = %.3f m/s", m_linear_vel);
}

// ── computeVelocityCommands ────────────────────────────────────────────────
geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist       & /*velocity*/,
  nav2_core::GoalChecker               * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;

  if (m_current_plan.poses.empty()) {
    RCLCPP_WARN(m_logger, "Plan is empty — sending zero velocity.");
    return cmd;
  }

  double robot_x   = pose.pose.position.x;
  double robot_y   = pose.pose.position.y;
  double robot_yaw = tf2::getYaw(pose.pose.orientation);

  // ── Kiểm tra đã đến đích chưa ────────────────────────────────────────
  const auto & goal_pose = m_current_plan.poses.back().pose;
  double dist_to_goal = Dist(robot_x, robot_y,
                             goal_pose.position.x, goal_pose.position.y);

  if (dist_to_goal < m_goal_tolerance) {
    RCLCPP_INFO(m_logger, "Goal reached (dist=%.3f m). Stopping.", dist_to_goal);
    // Zero velocity → controller_server sẽ báo goal succeeded
    return cmd;
  }

  // ── Tìm look-ahead point ─────────────────────────────────────────────
  size_t la_idx = FindLookAheadIndex(robot_x, robot_y);
  double la_x   = m_current_plan.poses[la_idx].pose.position.x;
  double la_y   = m_current_plan.poses[la_idx].pose.position.y;

  // ── Tính heading error (alpha) ────────────────────────────────────────
  //
  //  alpha = atan2(dy, dx) - robot_yaw
  //
  //  Nếu alpha > 0 → look-ahead point ở bên TRÁI robot → quay trái (+z)
  //  Nếu alpha < 0 → look-ahead point ở bên PHẢI robot → quay phải (-z)
  //
  double alpha = HeadingError(robot_x, robot_y, robot_yaw, la_x, la_y);

  // ── Tính lệnh vận tốc ─────────────────────────────────────────────────
  double angular_z = m_angular_gain * alpha;

  // Clamp angular velocity
  angular_z = std::clamp(angular_z, -m_max_angular_vel, m_max_angular_vel);

  // Giảm tốc độ tuyến tính khi cần xoay nhiều (smooth turning)
  double speed_scale = std::cos(alpha);
  speed_scale        = std::max(0.0, speed_scale);   // không cho lùi
  double linear_x    = m_linear_vel * speed_scale;

  cmd.twist.linear.x  = linear_x;
  cmd.twist.angular.z = angular_z;

  RCLCPP_DEBUG(m_logger,
    "robot=(%.2f,%.2f,yaw=%.2f) la=(%.2f,%.2f) alpha=%.3f rad → "
    "linear=%.3f m/s angular=%.3f rad/s",
    robot_x, robot_y, robot_yaw, la_x, la_y, alpha,
    linear_x, angular_z);

  return cmd;
}

// ── FindLookAheadIndex ────────────────────────────────────────────────────
size_t PurePursuitController::FindLookAheadIndex(double rx, double ry) const
{
  const auto & poses = m_current_plan.poses;
  size_t n = poses.size();

  // Tìm điểm gần nhất trên path với robot
  size_t nearest_idx = 0;
  double min_dist    = std::numeric_limits<double>::max();
  for (size_t i = 0; i < n; ++i) {
    double d = Dist(rx, ry,
                    poses[i].pose.position.x,
                    poses[i].pose.position.y);
    if (d < min_dist) {
      min_dist    = d;
      nearest_idx = i;
    }
  }

  // Đi tiếp từ nearest_idx cho đến khi tích lũy đủ lookahead_dist
  double arc = 0.0;
  for (size_t i = nearest_idx + 1; i < n; ++i) {
    double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
    double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
    arc += std::hypot(dx, dy);
    if (arc >= m_lookahead_dist) {
      return i;
    }
  }

  // Nếu không đủ arc-length → trả về điểm cuối (goal)
  return n - 1;
}

// ── HeadingError ──────────────────────────────────────────────────────────
double PurePursuitController::HeadingError(
  double rx, double ry, double r_yaw,
  double tx, double ty)
{
  double desired_heading = std::atan2(ty - ry, tx - rx);
  double alpha           = desired_heading - r_yaw;

  // Chuẩn hóa về [-pi, pi]
  while (alpha >  M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  return alpha;
}

}  // namespace my_pure_pursuit
