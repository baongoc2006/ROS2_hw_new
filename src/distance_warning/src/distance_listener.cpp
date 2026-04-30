#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class DistanceListener : public rclcpp::Node
{
public:
  DistanceListener() : Node("distance_listener")
  {
    // Khai báo parameter threshold với giá trị mặc định 0.5 m
    this->declare_parameter<double>("threshold", 0.5);
    // Cho phép parameter được set từ bên ngoài
    this->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "distance_topic",
      10,
      std::bind(&DistanceListener::distanceCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "distance_listener started. Subscribing to /distance_topic");
  }

private:
  void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    double threshold = this->get_parameter("threshold").as_double();
    float distance   = msg->data;

    RCLCPP_INFO(this->get_logger(),
      "──────────────────────────────────────────");
    RCLCPP_INFO(this->get_logger(),
      "Received distance : %.3f m", distance);
    RCLCPP_INFO(this->get_logger(),
      "Current threshold : %.2f m", threshold);

    if (distance < static_cast<float>(threshold)) {
      RCLCPP_WARN(this->get_logger(), "Warning: Object too close!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Distance is safe.");
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceListener>());
  rclcpp::shutdown();
  return 0;
}
