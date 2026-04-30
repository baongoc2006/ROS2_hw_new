#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "distance_warning/action/check_distance.hpp"

using CheckDistance = distance_warning::action::CheckDistance;
using GoalHandle    = rclcpp_action::ClientGoalHandle<CheckDistance>;

class DistanceActionClient : public rclcpp::Node
{
public:
  DistanceActionClient() : Node("distance_action_client")
  {
    client_ = rclcpp_action::create_client<CheckDistance>(this, "check_distance");
  }

  void sendGoal(float distance)
  {
    // Chờ action server sẵn sàng tối đa 5 giây
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after 5 seconds.");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = CheckDistance::Goal();
    goal_msg.distance_to_check = distance;
    RCLCPP_INFO(this->get_logger(), "Sending goal: check %.2f m", distance);

    auto send_goal_options = rclcpp_action::Client<CheckDistance>::SendGoalOptions();

    // ── Feedback callback: in từng bước nhận được ─────────────────────────
    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr /*gh*/,
             const std::shared_ptr<const CheckDistance::Feedback> feedback)
      {
        RCLCPP_INFO(this->get_logger(),
          "Feedback [%d/%d]: %s",
          feedback->step,
          feedback->total_steps,
          feedback->feedback_msg.c_str());
      };

    // ── Result callback: in kết quả SAFE hoặc NOT SAFE ────────────────────
    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult & wrapped_result)
      {
        switch (wrapped_result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            if (wrapped_result.result->is_safe) {
              RCLCPP_INFO(this->get_logger(),
                "RESULT: SAFE — %s", wrapped_result.result->result_message.c_str());
            } else {
              RCLCPP_WARN(this->get_logger(),
                "RESULT: NOT SAFE — %s", wrapped_result.result->result_message.c_str());
            }
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was cancelled.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<CheckDistance>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Đọc distance từ argv[1] nếu có, mặc định 0.3 m
  float distance = 0.3f;
  if (argc > 1) {
    try {
      distance = std::stof(argv[1]);
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("main"),
        "Invalid argument, using default distance 0.3 m");
    }
  }

  auto node = std::make_shared<DistanceActionClient>();
  node->sendGoal(distance);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
