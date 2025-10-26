cmake_minimum_required(VERSION 3.8)
project(navigation2_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wheeltec_mic_msg REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(autolocalization_node src/autolocalization.cpp) 
ament_target_dependencies(autolocalization_node rclcpp geometry_msgs std_srvs)

add_executable(cmd_nav_node src/cmd_nav.cpp )
ament_target_dependencies(cmd_nav_node rclcpp std_msgs geometry_msgs rclcpp_action nav2_msgs tf2_ros tf2_geometry_msgs wheeltec_mic_msg visualization_msgs)


install(
  DIRECTORY launch param maps src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  autolocalization_node
  cmd_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
