#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include "distance_warning/action/check_distance.hpp"

using CheckDistance = distance_warning::action::CheckDistance;
using GoalHandle    = rclcpp_action::ServerGoalHandle<CheckDistance>;

class DistanceActionServer : public rclcpp::Node
{
public:
  DistanceActionServer() : Node("distance_action_server")
  {
    this->declare_parameter<double>("threshold", 0.5);

    // Tạo action server với 3 callback bắt buộc
    action_server_ = rclcpp_action::create_server<CheckDistance>(
      this,
      "check_distance",
      std::bind(&DistanceActionServer::handleGoal,     this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&DistanceActionServer::handleCancel,   this, std::placeholders::_1),
      std::bind(&DistanceActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action server '/check_distance' started.");
  }

private:
  // ── 1. Chấp nhận hoặc từ chối goal ────────────────────────────────────────
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CheckDistance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received goal: check %.3f m", goal->distance_to_check);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // ── 2. Chấp nhận yêu cầu huỷ goal ─────────────────────────────────────────
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_WARN(this->get_logger(), "Goal cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // ── 3. Chạy goal trong thread riêng để không block executor ───────────────
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&DistanceActionServer::execute, this, goal_handle)}.detach();
  }

  // ── 4. Logic thực thi goal ─────────────────────────────────────────────────
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    float distance = goal_handle->get_goal()->distance_to_check;

    auto feedback = std::make_shared<CheckDistance::Feedback>();
    auto result   = std::make_shared<CheckDistance::Result>();

    // 5 bước xử lý giả lập
    const std::vector<std::string> steps = {
      "Receiving distance value...",
      "Fetching threshold parameter...",
      "Comparing values...",
      "Generating result...",
      "Done."
    };
    const int total_steps = static_cast<int>(steps.size());

    for (int i = 0; i < total_steps; ++i) {
      // Kiểm tra nếu goal bị cancel
      if (goal_handle->is_canceling()) {
        result->is_safe        = false;
        result->result_message = "Goal was cancelled.";
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "Goal cancelled during execution.");
        return;
      }

      // Gửi feedback bước hiện tại
      feedback->step         = i + 1;
      feedback->total_steps  = total_steps;
      feedback->feedback_msg = steps[i];
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(),
        "Feedback [%d/%d]: %s", i + 1, total_steps, steps[i].c_str());

      // Delay 500ms mỗi bước
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Lấy threshold hiện tại và so sánh
    double threshold   = this->get_parameter("threshold").as_double();
    result->is_safe    = (distance >= static_cast<float>(threshold));

    if (result->is_safe) {
      result->result_message = "SAFE: " + std::to_string(distance).substr(0, 5)
                               + " m >= threshold " + std::to_string(threshold).substr(0, 4) + " m";
      RCLCPP_INFO(this->get_logger(), "Result: SAFE (%.3f >= %.2f)", distance, threshold);
    } else {
      result->result_message = "NOT SAFE: " + std::to_string(distance).substr(0, 5)
                               + " m < threshold " + std::to_string(threshold).substr(0, 4) + " m";
      RCLCPP_WARN(this->get_logger(), "Result: NOT SAFE (%.3f < %.2f)", distance, threshold);
    }

    goal_handle->succeed(result);
  }

  rclcpp_action::Server<CheckDistance>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceActionServer>());
  rclcpp::shutdown();
  return 0;
}
