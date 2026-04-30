#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>

class DistancePublisherQoS : public rclcpp::Node
{
public:
  DistancePublisherQoS() : Node("distance_publisher_qos")
  {
    auto qos_reliable = rclcpp::QoS(10)
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .history(rclcpp::HistoryPolicy::KeepLast);

    auto qos_best_effort = rclcpp::QoS(10)
      .reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .history(rclcpp::HistoryPolicy::KeepLast);

    // Publisher RELIABLE trên '/distance_reliable' — 1 Hz
    pub_reliable_ = this->create_publisher<std_msgs::msg::Float32>(
      "distance_reliable", qos_reliable);

    // Publisher BEST_EFFORT trên '/distance_best_effort' — 10 Hz
    pub_best_effort_ = this->create_publisher<std_msgs::msg::Float32>(
      "distance_best_effort", qos_best_effort);

    // Timer 1 Hz → publishReliable()
    timer_reliable_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DistancePublisherQoS::publishReliable, this));

    // Timer 100 ms → publishBestEffort()
    timer_best_effort_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DistancePublisherQoS::publishBestEffort, this));

    RCLCPP_INFO(this->get_logger(),
      "distance_publisher_qos started.\n"
      "  /distance_reliable    — RELIABLE  @ 1 Hz\n"
      "  /distance_best_effort — BEST_EFFORT @ 10 Hz");
  }

private:
  float randomDistance()
  {
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<float> dist(0.1f, 1.5f);
    return dist(rng);
  }

  void publishReliable()
  {
    auto msg  = std_msgs::msg::Float32();
    msg.data  = randomDistance();
    pub_reliable_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
      "[RELIABLE    1Hz] Published: %.3f m", msg.data);
  }

  void publishBestEffort()
  {
    auto msg  = std_msgs::msg::Float32();
    msg.data  = randomDistance();
    pub_best_effort_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
      "[BEST_EFFORT 10Hz] Published: %.3f m", msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_reliable_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_best_effort_;
  rclcpp::TimerBase::SharedPtr                         timer_reliable_;
  rclcpp::TimerBase::SharedPtr                         timer_best_effort_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistancePublisherQoS>());
  rclcpp::shutdown();
  return 0;
}
