#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <cmath>
#include <string>
#include <memory>

namespace my_pure_pursuit
{

/**
 * @brief Simple Pure Pursuit Controller plugin for NAV2.
 *
 * Algorithm:
 *   1. Find the "look-ahead point" on the global path that is
 *      approximately `lookahead_dist` ahead of the robot.
 *   2. Compute the heading error (alpha) to that point.
 *   3. Set angular.z = gain * alpha, linear.x = constant speed.
 *   4. Return zero velocity when the robot is within goal_tolerance of
 *      the final waypoint.
 */
class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  // ── Lifecycle ──────────────────────────────────────────────────────────
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  // ── Core API ───────────────────────────────────────────────────────────
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist       & velocity,
    nav2_core::GoalChecker               * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  /**
   * @brief Find the index of the look-ahead point on the path.
   *
   * Searches forward from the nearest point on the path until the
   * cumulative arc-length exceeds `lookahead_dist`.
   *
   * @param robot_x  Robot x in map frame
   * @param robot_y  Robot y in map frame
   * @return Index into m_current_plan.poses
   */
  size_t FindLookAheadIndex(double robot_x, double robot_y) const;

  /**
   * @brief Compute the signed heading error from the robot's current yaw
   *        to the look-ahead point.
   *
   * alpha = atan2(dy, dx) - robot_yaw
   * Normalised to [-pi, pi].
   */
  static double HeadingError(double robot_x, double robot_y, double robot_yaw,
                             double target_x,  double target_y);

  /**
   * @brief Euclidean distance helper.
   */
  static double Dist(double x1, double y1, double x2, double y2)
  {
    return std::hypot(x2 - x1, y2 - y1);
  }

  // ── Members ────────────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr m_node;
  std::shared_ptr<tf2_ros::Buffer>         m_tf;
  std::string                              m_plugin_name;

  nav_msgs::msg::Path m_current_plan;

  double m_lookahead_dist{0.5};   // metres — how far ahead to aim
  double m_linear_vel{0.15};      // m/s   — constant forward speed
  double m_angular_gain{1.5};     // rad/s per rad of heading error
  double m_goal_tolerance{0.25};  // metres — stop when this close to goal
  double m_max_angular_vel{1.0};  // rad/s  — safety clamp

  rclcpp::Logger m_logger{rclcpp::get_logger("PurePursuitController")};
};

}  // namespace my_pure_pursuit
