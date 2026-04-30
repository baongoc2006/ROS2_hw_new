#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class DistanceTfBroadcaster : public rclcpp::Node
{
public:
  DistanceTfBroadcaster() : Node("distance_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "distance_topic",
      10,
      std::bind(&DistanceTfBroadcaster::broadcastCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "distance_tf_broadcaster started. Subscribing to /distance_topic");
  }

private:
  void broadcastCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    auto now = this->get_clock()->now();

    // --- world → base_link (cố định tại gốc) ---
    geometry_msgs::msg::TransformStamped t_base;
    t_base.header.stamp    = now;
    t_base.header.frame_id = "world";
    t_base.child_frame_id  = "base_link";
    t_base.transform.translation.x = 0.0;
    t_base.transform.translation.y = 0.0;
    t_base.transform.translation.z = 0.0;
    t_base.transform.rotation.w    = 1.0;
    tf_broadcaster_->sendTransform(t_base);

    // --- base_link → sensor_link (x = distance đo được) ---
    geometry_msgs::msg::TransformStamped t_sensor;
    t_sensor.header.stamp    = now;
    t_sensor.header.frame_id = "base_link";
    t_sensor.child_frame_id  = "sensor_link";
    t_sensor.transform.translation.x = static_cast<double>(msg->data);
    t_sensor.transform.translation.y = 0.0;
    t_sensor.transform.translation.z = 0.0;
    t_sensor.transform.rotation.w    = 1.0;
    tf_broadcaster_->sendTransform(t_sensor);

    RCLCPP_INFO(this->get_logger(),
      "Broadcasting sensor_link at x=%.2f m", msg->data);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
