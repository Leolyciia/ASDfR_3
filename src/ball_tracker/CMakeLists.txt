cmake_minimum_required(VERSION 3.22)
project(ball_tracker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Change the executable name from ball_tracker_node to ball_tracker
add_executable(ball_tracker src/ball_tracker.cpp)

ament_target_dependencies(ball_tracker
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

install(TARGETS ball_tracker
  DESTINATION lib/${PROJECT_NAME})

ament_package()

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
