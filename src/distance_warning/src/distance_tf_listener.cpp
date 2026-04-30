#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "distance_warning/srv/set_threshold.hpp"
#include <cmath>

using SetThreshold = distance_warning::srv::SetThreshold;

class DistanceTfListener : public rclcpp::Node
{
public:
  DistanceTfListener() : Node("distance_tf_listener")
  {
    this->declare_parameter<double>("tf_threshold", 1.0);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    threshold_client_ = this->create_client<SetThreshold>("set_threshold");

    // Timer 1 Hz → lookupAndCheck()
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DistanceTfListener::lookupAndCheck, this));

    RCLCPP_INFO(this->get_logger(),
      "distance_tf_listener started. Looking up world->sensor_link @ 1 Hz");
  }

private:
  void lookupAndCheck()
  {
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        "world", "sensor_link", tf2::TimePointZero);

      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double z = t.transform.translation.z;

      // Tính khoảng cách Euclidean từ world origin đến sensor_link
      double dist = std::sqrt(x * x + y * y + z * z);

      RCLCPP_INFO(this->get_logger(),
        "TF distance world->sensor_link: %.2f m", dist);

      double tf_threshold = this->get_parameter("tf_threshold").as_double();

      if (dist > tf_threshold) {
        RCLCPP_WARN(this->get_logger(),
          "Sensor out of range! %.2f m > tf_threshold %.2f m", dist, tf_threshold);
        callSetThreshold(false);  // Tự động giảm threshold
      }

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  void callSetThreshold(bool increase)
  {
    if (!threshold_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(this->get_logger(), "set_threshold service not available");
      return;
    }
    auto request = std::make_shared<SetThreshold::Request>();
    request->increase = increase;

    threshold_client_->async_send_request(request,
      [this](rclcpp::Client<SetThreshold>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(),
            "Auto-adjusted threshold to %.2f m", response->new_threshold);
        }
      });
  }

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<SetThreshold>::SharedPtr     threshold_client_;
  rclcpp::TimerBase::SharedPtr                timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceTfListener>());
  rclcpp::shutdown();
  return 0;
}
