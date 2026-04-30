#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tuple>
#include <vector>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class PatrolNode : public rclcpp::Node
{
public:
  PatrolNode() : Node("patrol_node"), m_current_index(0), m_is_running(true)
  {
    m_nav_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // 4 waypoint tuần tra trong turtlebot3_world: {x, y, yaw_rad}
    m_waypoints = {
      { 1.5, -0.5,  0.0},
      { 1.5,  1.5,  1.5708},   // ~90°
      {-1.0,  1.5,  3.1416},   // ~180°
      {-1.0, -0.5, -1.5708},   // ~-90°
    };

    // Graceful shutdown khi nhận Ctrl+C
    rclcpp::on_shutdown([this]() {
      m_is_running = false;
      RCLCPP_INFO(this->get_logger(), "Patrol node shutting down gracefully.");
    });

    RCLCPP_INFO(this->get_logger(),
      "PatrolNode started. %zu waypoints loaded. Waiting for action server...",
      m_waypoints.size());

    // Chờ action server sẵn sàng trước khi bắt tuần tra
    if (!m_nav_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /navigate_to_pose not available. Exiting.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Action server ready. Starting patrol...");
    SendNextGoal();
  }

private:
  // ── Gửi goal đến waypoint tiếp theo trong danh sách ─────────────────────
  void SendNextGoal()
  {
    if (!m_is_running || !rclcpp::ok()) return;

    auto [x, y, yaw] = m_waypoints[m_current_index];

    RCLCPP_INFO(this->get_logger(),
      "── Navigating to waypoint [%zu/%zu]: (%.2f, %.2f, yaw=%.2f)",
      m_current_index + 1, m_waypoints.size(), x, y, yaw);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.pose.pose.position.x  = x;
    goal_msg.pose.pose.position.y  = y;
    goal_msg.pose.pose.position.z  = 0.0;

    // Chuyển yaw → quaternion (chỉ xoay quanh trục z)
    goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & goal_handle)
      { GoalResponseCallback(goal_handle); };

    send_goal_options.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      { FeedbackCallback(feedback); };

    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result)
      { ResultCallback(result); };

    m_nav_client->async_send_goal(goal_msg, send_goal_options);
  }

  // ── Goal chấp nhận / bị từ chối ─────────────────────────────────────────
  void GoalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
        "Goal to waypoint [%zu] was REJECTED by action server!", m_current_index + 1);
      AdvanceAndSend();
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Goal to waypoint [%zu] ACCEPTED.", m_current_index + 1);
    }
  }

  // ── Feedback: in vị trí hiện tại và ETA ─────────────────────────────────
  void FeedbackCallback(
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    auto & pos = feedback->current_pose.pose.position;
    double eta = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
    RCLCPP_INFO(this->get_logger(),
      "  ↳ Current pos: (%.2f, %.2f) | ETA: %.1f s",
      pos.x, pos.y, eta);
  }

  // ── Kết quả: SUCCEEDED / ABORTED / CANCELED ─────────────────────────────
  void ResultCallback(const GoalHandleNav::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(),
          "✓ Arrived at waypoint [%zu]! Waiting 3 seconds...",
          m_current_index + 1);
        // Dừng 3 giây tại waypoint rồi mới chuyển điểm tiếp
        rclcpp::sleep_for(std::chrono::seconds(3));
        AdvanceAndSend();
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(),
          "✗ Goal ABORTED at waypoint [%zu]. Skipping to next waypoint.",
          m_current_index + 1);
        AdvanceAndSend();
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(),
          "✗ Goal CANCELED at waypoint [%zu]. Skipping to next waypoint.",
          m_current_index + 1);
        AdvanceAndSend();
        break;

      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code. Skipping.");
        AdvanceAndSend();
        break;
    }
  }

  // ── Tăng index theo vòng (wrap-around) và gửi goal tiếp theo ─────────────
  void AdvanceAndSend()
  {
    if (!m_is_running || !rclcpp::ok()) return;

    m_current_index = (m_current_index + 1) % m_waypoints.size();

    // Nếu vừa quay về index 0 → bắt đầu vòng mới
    if (m_current_index == 0) {
      RCLCPP_INFO(this->get_logger(),
        "════ Completed one full patrol loop! Starting again... ════");
    }

    SendNextGoal();
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr m_nav_client;
  std::vector<std::tuple<double, double, double>>  m_waypoints;
  size_t m_current_index;
  bool   m_is_running;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNode>());
  rclcpp::shutdown();
  return 0;
}
