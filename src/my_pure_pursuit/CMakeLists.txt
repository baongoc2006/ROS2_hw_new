cmake_minimum_required(VERSION 3.8)
project(my_pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_lifecycle   REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(nav2_core          REQUIRED)
find_package(nav2_costmap_2d    REQUIRED)
find_package(nav2_util          REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(pluginlib          REQUIRED)

# ── Plugin shared library ────────────────────────────────────────────────────
add_library(my_pure_pursuit SHARED
  src/pure_pursuit_controller.cpp
)

target_include_directories(my_pure_pursuit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_pure_pursuit
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

# Đăng ký plugin với pluginlib
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS my_pure_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(my_pure_pursuit)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
  nav2_core nav2_costmap_2d nav2_util
  tf2 tf2_ros tf2_geometry_msgs pluginlib
)

ament_package()