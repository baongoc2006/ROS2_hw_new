#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>

class DistancePublisher : public rclcpp::Node
{
public:
  DistancePublisher()
  : Node("distance_publisher"),
    rng_(std::random_device{}()),
    dist_(0.1f, 1.5f)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("distance_topic", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DistancePublisher::publishDistance, this));

    RCLCPP_INFO(this->get_logger(), "distance_publisher started. Publishing on /distance_topic @ 1 Hz");
  }

private:
  void publishDistance()
  {
    auto msg = std_msgs::msg::Float32();
    msg.data = dist_(rng_);

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published distance: %.3f m", msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
  std::uniform_real_distribution<float> dist_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistancePublisher>());
  rclcpp::shutdown();
  return 0;
}
