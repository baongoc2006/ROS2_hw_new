cmake_minimum_required(VERSION 3.8)
project(distance_warning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Generate interfaces (srv + action) ────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetThreshold.srv"
  "action/CheckDistance.action"
  DEPENDENCIES std_msgs
)

# Helper to link generated interfaces to a target
# (replaces the old rosidl_target_interfaces macro in Humble+)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── distance_publisher ────────────────────────────────────────────────────────
add_executable(distance_publisher src/distance_publisher.cpp)
ament_target_dependencies(distance_publisher rclcpp std_msgs)
install(TARGETS distance_publisher DESTINATION lib/${PROJECT_NAME})

# ── distance_listener ─────────────────────────────────────────────────────────
add_executable(distance_listener src/distance_listener.cpp)
ament_target_dependencies(distance_listener rclcpp std_msgs)
install(TARGETS distance_listener DESTINATION lib/${PROJECT_NAME})

# ── set_threshold_service ─────────────────────────────────────────────────────
add_executable(set_threshold_service src/set_threshold_service.cpp)
ament_target_dependencies(set_threshold_service rclcpp)
target_link_libraries(set_threshold_service "${cpp_typesupport_target}")
install(TARGETS set_threshold_service DESTINATION lib/${PROJECT_NAME})

# ── distance_action_server ────────────────────────────────────────────────────
add_executable(distance_action_server src/distance_action_server.cpp)
ament_target_dependencies(distance_action_server rclcpp rclcpp_action)
target_link_libraries(distance_action_server "${cpp_typesupport_target}")
install(TARGETS distance_action_server DESTINATION lib/${PROJECT_NAME})

# ── distance_action_client ────────────────────────────────────────────────────
add_executable(distance_action_client src/distance_action_client.cpp)
ament_target_dependencies(distance_action_client rclcpp rclcpp_action)
target_link_libraries(distance_action_client "${cpp_typesupport_target}")
install(TARGETS distance_action_client DESTINATION lib/${PROJECT_NAME})

add_executable(distance_tf_broadcaster src/distance_tf_broadcaster.cpp)
ament_target_dependencies(distance_tf_broadcaster
  rclcpp std_msgs geometry_msgs tf2 tf2_ros)
install(TARGETS distance_tf_broadcaster DESTINATION lib/${PROJECT_NAME})
 
add_executable(distance_tf_listener src/distance_tf_listener.cpp)
ament_target_dependencies(distance_tf_listener
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
rosidl_target_interfaces(distance_tf_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS distance_tf_listener DESTINATION lib/${PROJECT_NAME})
 
add_executable(distance_publisher_qos src/distance_publisher_qos.cpp)
ament_target_dependencies(distance_publisher_qos rclcpp std_msgs)
install(TARGETS distance_publisher_qos DESTINATION lib/${PROJECT_NAME})
 
add_executable(distance_listener_qos src/distance_listener_qos.cpp)
ament_target_dependencies(distance_listener_qos rclcpp std_msgs)
install(TARGETS distance_listener_qos DESTINATION lib/${PROJECT_NAME})

# ── Launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
