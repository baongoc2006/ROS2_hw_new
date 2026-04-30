# ═══════════════════════════════════════════════════════════════════════════
# CMakeLists.txt cho package nav2_goal_sender (Bài 1 + Bài 2)
# ═══════════════════════════════════════════════════════════════════════════
cmake_minimum_required(VERSION 3.8)
project(nav2_goal_sender)

find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs     REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Bài 1: patrol_node ──────────────────────────────────────────────────────
add_executable(patrol_node src/patrol_node.cpp)
ament_target_dependencies(patrol_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs)
install(TARGETS patrol_node DESTINATION lib/${PROJECT_NAME})

# ── Bài 2: planner_benchmark ────────────────────────────────────────────────
add_executable(planner_benchmark src/planner_benchmark.cpp)
ament_target_dependencies(planner_benchmark
  rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs)
install(TARGETS planner_benchmark DESTINATION lib/${PROJECT_NAME})

ament_package()


# ═══════════════════════════════════════════════════════════════════════════
# package.xml cho nav2_goal_sender
# ═══════════════════════════════════════════════════════════════════════════
# <?xml version="1.0"?>
# <package format="3">
#   <name>nav2_goal_sender</name>
#   <version>0.0.1</version>
#   <description>NAV2 goal sender nodes (BTVN_03)</description>
#   <maintainer email="student@example.com">student</maintainer>
#   <license>Apache-2.0</license>
#   <buildtool_depend>ament_cmake</buildtool_depend>
#   <depend>rclcpp</depend>
#   <depend>rclcpp_action</depend>
#   <depend>nav2_msgs</depend>
#   <depend>nav_msgs</depend>
#   <depend>geometry_msgs</depend>
#   <export><build_type>ament_cmake</build_type></export>
# </package>


# ═══════════════════════════════════════════════════════════════════════════
# Cấu hình nav2_params.yaml cho my_pure_pursuit (thêm vào controller_server)
# ═══════════════════════════════════════════════════════════════════════════
#
# controller_server:
#   ros__parameters:
#     controller_plugins: ["FollowPath"]
#     FollowPath:
#       plugin: "my_pure_pursuit/PurePursuitController"
#       lookahead_dist:  0.5
#       linear_vel:      0.15
#       angular_gain:    1.5
#       goal_tolerance:  0.25
#       max_angular_vel: 1.0


# ═══════════════════════════════════════════════════════════════════════════
# Build & chạy
# ═══════════════════════════════════════════════════════════════════════════
#
# cd ~/ros2_ws
# colcon build --packages-select nav2_goal_sender my_pure_pursuit
# source install/setup.bash
#
# # Bài 1 — Patrol
# ros2 run nav2_goal_sender patrol_node
#
# # Bài 2 — Benchmark (chạy khi NAV2 đã sẵn sàng)
# ros2 run nav2_goal_sender planner_benchmark
#
# # Bài 3 — Dùng plugin trong NAV2 (khai báo trong nav2_params.yaml)
# ros2 launch turtlebot3_navigation2 navigation2.launch.py \
#   use_sim_time:=True \
#   map:=$HOME/maps/turtlebot3_world.yaml \
#   params_file:=$HOME/my_nav2_params.yaml