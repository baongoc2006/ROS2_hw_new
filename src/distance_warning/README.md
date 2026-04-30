# distance_warning — ROS 2 Package (C++)

Hệ thống giả lập đo khoảng cách và cảnh báo khi vật thể quá gần.
Viết bằng C++ với `rclcpp` và `rclcpp_action`, build bằng `colcon`.

---

## Cấu trúc thư mục

```
distance_warning/
├── action/
│   └── CheckDistance.action          # Goal / Result / Feedback
├── srv/
│   └── SetThreshold.srv              # Request / Response
├── src/
│   ├── distance_publisher.cpp        # Node: phát khoảng cách ngẫu nhiên
│   ├── distance_listener.cpp         # Node: subscribe + cảnh báo
│   ├── set_threshold_service.cpp     # Node: service server
│   ├── distance_action_server.cpp    # Node: action server
│   └── distance_action_client.cpp   # Node: action client
├── launch/
│   └── distance_warning.launch.py   # Launch file Python
├── CMakeLists.txt
└── package.xml
```

---

## Cài đặt & Build

```bash
# 1. Copy package vào workspace
cp -r distance_warning ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select distance_warning

# 3. Source
source install/setup.bash
```

---

## Chạy hệ thống

### Khởi động toàn bộ (publisher + listener + service + action server)

```bash
ros2 launch distance_warning distance_warning.launch.py

# Với threshold tùy chỉnh
ros2 launch distance_warning distance_warning.launch.py threshold:=0.8
```

### Chạy action client (terminal riêng)

```bash
# Kiểm tra khoảng cách 0.3 m (mặc định)
ros2 run distance_warning distance_action_client

# Kiểm tra khoảng cách tùy chỉnh
ros2 run distance_warning distance_action_client 0.7
```

---

## Tương tác runtime

### Xem topic

```bash
ros2 topic echo /distance_topic
ros2 topic hz /distance_topic         # → phải ra ~1.0 Hz
```

### Gọi service thay đổi threshold

```bash
# Tăng threshold 0.1 m
ros2 service call /set_threshold distance_warning/srv/SetThreshold '{increase: true}'

# Giảm threshold 0.1 m
ros2 service call /set_threshold distance_warning/srv/SetThreshold '{increase: false}'
```

**Expected output:**
```
[INFO] [set_threshold_service]: Threshold updated: 0.50 -> 0.60 m

# Response:
# success: true
# new_threshold: 0.6
# message: Threshold increased to 0.60 m
```

### Kiểm tra graph

```bash
rqt_graph
```

---

## Interfaces

### `srv/SetThreshold.srv`

```
bool increase        # true = tăng, false = giảm
---
bool success
float64 new_threshold
string message
```

### `action/CheckDistance.action`

```
float32 distance_to_check
---
bool is_safe
string result_message
---
int32 step
int32 total_steps
string feedback_msg
```

---

## Expected output — action client

```
[INFO] [distance_action_client]: Sending goal: check 0.30 m
[INFO] [distance_action_client]: Feedback [1/5]: Receiving distance value...
[INFO] [distance_action_client]: Feedback [2/5]: Fetching threshold parameter...
[INFO] [distance_action_client]: Feedback [3/5]: Comparing values...
[INFO] [distance_action_client]: Feedback [4/5]: Generating result...
[INFO] [distance_action_client]: Feedback [5/5]: Done.
[WARN] [distance_action_client]: RESULT: NOT SAFE — NOT SAFE: 0.300 m < threshold 0.50 m
```

---

## Dependencies

- ROS 2 Humble (hoặc Foxy/Iron)
- `rclcpp`, `rclcpp_action`, `std_msgs`
- `rosidl_default_generators`, `rosidl_default_runtime`
