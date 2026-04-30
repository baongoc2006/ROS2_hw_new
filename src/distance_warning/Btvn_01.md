# Bài Tập ROS2 — `distance_warning`

> Language: C++ (rclcpp) | ROS2 Humble

---

## 1. Cấu trúc Package

```
distance_warning/
├── src/
│   ├── distance_publisher.cpp
│   ├── distance_listener.cpp
│   ├── set_threshold_service.cpp
│   ├── distance_action_server.cpp
│   └── distance_action_client.cpp
├── action/
│   └── CheckDistance.action
├── srv/
│   └── SetThreshold.srv
├── launch/
│   └── distance_warning.launch.py
├── CMakeLists.txt
└── package.xml
```

### Tạo package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake distance_warning \
  --dependencies rclcpp std_msgs rclcpp_action

mkdir distance_warning/src distance_warning/action \
      distance_warning/srv distance_warning/launch
```

---

## 2. Custom Interfaces

### `srv/SetThreshold.srv`

```
# Request
bool increase   # true = tăng, false = giảm
---
# Response
bool success
float32 new_threshold
string message
```

### `action/CheckDistance.action`

```
# Goal
float32 distance_to_check
---
# Result
bool is_safe
string result_message
---
# Feedback
int32 step
int32 total_steps
string feedback_msg
```

### `package.xml`

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>std_msgs</depend>
```

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(distance_warning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetThreshold.srv"
  "action/CheckDistance.action"
)

# TODO: Thêm executable cho từng node (xem hướng dẫn từng phần bên dưới)

ament_package()
```

---

## 3. Parameters

Trong ROS2, **không có global parameter server** như ROS1. Mỗi node tự khai báo và quản lý parameter của mình. Để đồng bộ `threshold` runtime, dùng service để các node thông báo cho nhau.

### Khai báo trong C++

```cpp
class DistanceListener : public rclcpp::Node
{
public:
  DistanceListener() : Node("distance_listener")
  {
    this->declare_parameter<double>("threshold", 0.5);
  }

private:
  double get_threshold()
  {
    return this->get_parameter("threshold").as_double();
  }
};
```

> Kiểm tra parameter đang chạy: `ros2 param get /distance_listener threshold`

---

## 4. Publisher & Subscriber

### 4.1 Node `distance_publisher`

**File:** `src/distance_publisher.cpp`

**Yêu cầu:**
- Publish `/distance_topic` kiểu `std_msgs/msg/Float32`
- Giá trị ngẫu nhiên từ `0.1` đến `1.5` mét
- Tần suất: **1 Hz**

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>

class DistancePublisher : public rclcpp::Node
{
public:
  DistancePublisher() : Node("distance_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("distance_topic", 10);
    // TODO: Tạo timer gọi timerCallback mỗi 1 giây
    // timer_ = this->create_wall_timer(...);
  }

private:
  void timerCallback()
  {
    auto msg = std_msgs::msg::Float32();
    // TODO: Gán giá trị ngẫu nhiên từ 0.1 đến 1.5
    // Gợi ý: dùng std::uniform_real_distribution
    // msg.data = ...;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: %.2f m", msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistancePublisher>());
  rclcpp::shutdown();
  return 0;
}
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_publisher src/distance_publisher.cpp)
ament_target_dependencies(distance_publisher rclcpp std_msgs)
install(TARGETS distance_publisher DESTINATION lib/${PROJECT_NAME})
```

---

### 4.2 Node `distance_listener`

**File:** `src/distance_listener.cpp`

**Yêu cầu:**
- Subscribe `/distance_topic`
- Lấy `threshold` từ parameter của chính node
- Khi `distance < threshold`: in cảnh báo kèm giá trị và threshold

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class DistanceListener : public rclcpp::Node
{
public:
  DistanceListener() : Node("distance_listener")
  {
    this->declare_parameter<double>("threshold", 0.5);
    // TODO: Tạo subscriber lắng nghe 'distance_topic'
    // subscription_ = this->create_subscription<std_msgs::msg::Float32>(...);
  }

private:
  void listenerCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    double distance = msg->data;
    double threshold = this->get_parameter("threshold").as_double();
    // TODO: In khoảng cách nhận được
    // TODO: Nếu distance < threshold, in cảnh báo bằng RCLCPP_WARN
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
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_listener src/distance_listener.cpp)
ament_target_dependencies(distance_listener rclcpp std_msgs)
install(TARGETS distance_listener DESTINATION lib/${PROJECT_NAME})
```

**Expected output:**

```
[INFO] [distance_listener]: Distance received: 0.82 m
[INFO] [distance_listener]: Distance received: 0.34 m
[WARN] [distance_listener]: Warning: Object too close! (0.34 m < threshold: 0.50 m)
```

---

## 5. Service

### 5.1 Node `set_threshold_service` (Server)

**File:** `src/set_threshold_service.cpp`

**Yêu cầu:**
- Tạo service server `/set_threshold`
- `request->increase = true` → tăng `0.1` (tối đa `1.5`)
- `request->increase = false` → giảm `0.1` (tối thiểu `0.1`)
- Cập nhật parameter `threshold` của node
- Trả về `success`, `new_threshold`, `message`

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include "distance_warning/srv/set_threshold.hpp"

using SetThreshold = distance_warning::srv::SetThreshold;

class SetThresholdService : public rclcpp::Node
{
public:
  SetThresholdService() : Node("set_threshold_service")
  {
    this->declare_parameter<double>("threshold", 0.5);
    // TODO: Tạo service server
    // service_ = this->create_service<SetThreshold>(
    //   "set_threshold",
    //   std::bind(&SetThresholdService::handleSetThreshold, this,
    //             std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleSetThreshold(
    const SetThreshold::Request::SharedPtr request,
    SetThreshold::Response::SharedPtr response)
  {
    double current = this->get_parameter("threshold").as_double();
    double new_threshold = current;

    // TODO: Tăng hoặc giảm new_threshold theo request->increase
    // TODO: Clamp giá trị trong khoảng [0.1, 1.5]
    // TODO: Cập nhật parameter bằng this->set_parameter(...)
    // TODO: Set response->success, new_threshold, message

    RCLCPP_INFO(this->get_logger(), "Threshold updated: %.2f -> %.2f m",
                current, new_threshold);
  }

  rclcpp::Service<SetThreshold>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetThresholdService>());
  rclcpp::shutdown();
  return 0;
}
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(set_threshold_service src/set_threshold_service.cpp)
ament_target_dependencies(set_threshold_service rclcpp)
rosidl_target_interfaces(set_threshold_service ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS set_threshold_service DESTINATION lib/${PROJECT_NAME})
```

### 5.2 Test bằng CLI

```bash
# Tăng threshold
ros2 service call /set_threshold distance_warning/srv/SetThreshold '{increase: true}'

# Giảm threshold
ros2 service call /set_threshold distance_warning/srv/SetThreshold '{increase: false}'
```

**Expected output:**

```
[INFO] [set_threshold_service]: Threshold updated: 0.50 -> 0.60 m

# Response:
# success: true
# new_threshold: 0.6000000238418579
# message: Threshold increased to 0.60 m
```

---

## 6. Action

### 6.1 Node `distance_action_server` (Server)

**File:** `src/distance_action_server.cpp`

**Yêu cầu:**
- Tạo action server `/check_distance`
- Nhận goal `distance_to_check: float32`
- Xử lý qua **5 bước**, mỗi bước delay **500ms**, gửi feedback từng bước

| Bước | `feedback_msg` |
|------|----------------|
| 1/5 | `Receiving distance value...` |
| 2/5 | `Fetching threshold parameter...` |
| 3/5 | `Comparing values...` |
| 4/5 | `Generating result...` |
| 5/5 | `Done.` |

- Trả về `is_safe = true` nếu `distance >= threshold`

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include "distance_warning/action/check_distance.hpp"

using CheckDistance = distance_warning::action::CheckDistance;
using GoalHandle = rclcpp_action::ServerGoalHandle<CheckDistance>;

class DistanceActionServer : public rclcpp::Node
{
public:
  DistanceActionServer() : Node("distance_action_server")
  {
    this->declare_parameter<double>("threshold", 0.5);
    // TODO: Tạo action server
    // action_server_ = rclcpp_action::create_server<CheckDistance>(
    //   this, "check_distance",
    //   std::bind(&DistanceActionServer::handleGoal, this, ...),
    //   std::bind(&DistanceActionServer::handleCancel, this, ...),
    //   std::bind(&DistanceActionServer::handleAccepted, this, ...));
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CheckDistance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: check %.2f m", goal->distance_to_check);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&DistanceActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    float distance = goal_handle->get_goal()->distance_to_check;
    auto feedback = std::make_shared<CheckDistance::Feedback>();
    auto result = std::make_shared<CheckDistance::Result>();

    std::vector<std::string> steps = {
      "Receiving distance value...",
      "Fetching threshold parameter...",
      "Comparing values...",
      "Generating result...",
      "Done."
    };

    // TODO: Loop qua 5 bước:
    //   - Set feedback->step, total_steps, feedback_msg
    //   - goal_handle->publish_feedback(feedback)
    //   - std::this_thread::sleep_for(std::chrono::milliseconds(500))

    // TODO: Lấy threshold, so sánh với distance
    // TODO: Set result->is_safe và result->result_message
    // TODO: goal_handle->succeed(result)
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
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_action_server src/distance_action_server.cpp)
ament_target_dependencies(distance_action_server rclcpp rclcpp_action)
rosidl_target_interfaces(distance_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS distance_action_server DESTINATION lib/${PROJECT_NAME})
```

---

### 6.2 Node `distance_action_client` (Client)

**File:** `src/distance_action_client.cpp`

**Yêu cầu:**
- Gửi goal với `distance_to_check` (hardcode hoặc từ `argv`)
- In feedback mỗi bước nhận được
- In kết quả cuối cùng rõ ràng

**Code khung:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "distance_warning/action/check_distance.hpp"

using CheckDistance = distance_warning::action::CheckDistance;
using GoalHandle = rclcpp_action::ClientGoalHandle<CheckDistance>;

class DistanceActionClient : public rclcpp::Node
{
public:
  DistanceActionClient() : Node("distance_action_client")
  {
    client_ = rclcpp_action::create_client<CheckDistance>(this, "check_distance");
  }

  void sendGoal(float distance)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = CheckDistance::Goal();
    goal_msg.distance_to_check = distance;
    RCLCPP_INFO(this->get_logger(), "Sending goal: check %.2f m", distance);

    auto send_goal_options = rclcpp_action::Client<CheckDistance>::SendGoalOptions();

    // TODO: Set send_goal_options.feedback_callback
    //   - In từng bước: [step/total_steps]: feedback_msg

    // TODO: Set send_goal_options.result_callback
    //   - In SAFE hoặc NOT SAFE theo result->is_safe

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<CheckDistance>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistanceActionClient>();
  node->sendGoal(0.3f);  // TODO: Thay bằng argv nếu muốn
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

**Thêm vào `CMakeLists.txt`:**

```cmake
add_executable(distance_action_client src/distance_action_client.cpp)
ament_target_dependencies(distance_action_client rclcpp rclcpp_action)
rosidl_target_interfaces(distance_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS distance_action_client DESTINATION lib/${PROJECT_NAME})
```

**Expected output:**

```
[INFO] [distance_action_client]: Sending goal: check 0.30 m
[INFO] [distance_action_client]: Feedback [1/5]: Receiving distance value...
[INFO] [distance_action_client]: Feedback [2/5]: Fetching threshold parameter...
[INFO] [distance_action_client]: Feedback [3/5]: Comparing values...
[INFO] [distance_action_client]: Feedback [4/5]: Generating result...
[INFO] [distance_action_client]: Feedback [5/5]: Done.
[WARN] [distance_action_client]: RESULT: NOT SAFE — 0.30 m < threshold 0.50 m
```

---

## 7. Launch File

**File:** `launch/distance_warning.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.5',
        description='Distance warning threshold in meters'
    )
    threshold = LaunchConfiguration('threshold')

    return LaunchDescription([
        threshold_arg,
        Node(
            package='distance_warning',
            executable='distance_publisher',
            name='distance_publisher'
        ),
        Node(
            package='distance_warning',
            executable='distance_listener',
            name='distance_listener',
            parameters=[{'threshold': threshold}]
        ),
        Node(
            package='distance_warning',
            executable='set_threshold_service',
            name='set_threshold_service',
            parameters=[{'threshold': threshold}]
        ),
        Node(
            package='distance_warning',
            executable='distance_action_server',
            name='distance_action_server',
            parameters=[{'threshold': threshold}]
        ),
    ])
```

**Thêm vào `CMakeLists.txt`:**

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

**Build và chạy:**

```bash
cd ~/ros2_ws
colcon build --packages-select distance_warning
source install/setup.bash

# Chạy với threshold mặc định
ros2 launch distance_warning distance_warning.launch.py

# Chạy với threshold tùy chỉnh
ros2 launch distance_warning distance_warning.launch.py threshold:=0.8
```

---

## 8. Tiêu chí chấp nhận

| # | Tiêu chí | Kiểm tra bằng |
|---|----------|---------------|
| 1 | Package build không có warning | `colcon build` |
| 2 | `distance_publisher` publish đúng 1 Hz | `ros2 topic hz /distance_topic` |
| 3 | `distance_listener` in cảnh báo khi `distance < threshold` | Quan sát log |
| 4 | Service thay đổi được `threshold` runtime | `ros2 service call ...` |
| 5 | Sau khi gọi service, listener dùng threshold mới | Quan sát log |
| 6 | Action server gửi đủ 5 bước feedback | Chạy action client |
| 7 | Action trả về `is_safe` đúng theo threshold hiện tại | Test nhiều giá trị |
| 8 | Launch file khởi động toàn bộ hệ thống một lệnh | `ros2 launch ...` |
| 9 | `rqt_graph` hiển thị đủ node và topic | `rqt_graph` |
