cmake_minimum_required(VERSION 3.8)
project(custom_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/usr/include/gz/sim7)
include_directories(/usr/include/gz/sim/components)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz-sim8 REQUIRED)

add_executable(cmd_vel_to_rpm src/diff_drive_controller.cpp)
add_library(custom_diff_drive SHARED src/new_plugin.cpp)
target_link_libraries(custom_diff_drive PRIVATE gz-sim8::gz-sim8)

ament_target_dependencies(cmd_vel_to_rpm rclcpp geometry_msgs std_msgs)

install(
  DIRECTORY launch urdf worlds config params
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS cmd_vel_to_rpm DESTINATION lib/${PROJECT_NAME})
install(TARGETS custom_diff_drive DESTINATION lib)



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

ament_package()
