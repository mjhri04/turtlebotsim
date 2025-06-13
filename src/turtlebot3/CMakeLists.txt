cmake_minimum_required(VERSION 3.8)
project(turtlebot3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch models maps params
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/map_to_stl_node.cpp
  nodes/map_to_world_node.cpp
  nodes/goal_pose_to_nav2.cpp
  nodes/map_to_stl_topic.cpp
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(map_to_stl_node nodes/map_to_stl_node.cpp)
# ament_target_dependencies(map_to_stl_node rclcpp nav_msgs)
ament_target_dependencies(map_to_stl_node rclcpp
  nav_msgs
  yaml-cpp
  )
target_link_libraries(map_to_stl_node ${OpenCV_LIBS} yaml-cpp)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(map_to_world_node nodes/map_to_world_node.cpp)
ament_target_dependencies(map_to_world_node rclcpp)

add_executable(map_to_stl_topic nodes/map_to_stl_topic.cpp)
ament_target_dependencies(map_to_stl_topic 
  rclcpp 
  sensor_msgs
  std_msgs
  nav_msgs
)

add_executable(cmd_vel_halver nodes/cmd_vel_halver.cpp)
ament_target_dependencies(cmd_vel_halver 
  rclcpp 
  geometry_msgs
  nav_msgs
  std_msgs
)
add_executable(goal_pose_to_nav2 nodes/goal_pose_to_nav2.cpp)
ament_target_dependencies(goal_pose_to_nav2
  rclcpp
  std_msgs
  geometry_msgs
  nav2_msgs
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  tf2
  )

install(TARGETS map_to_stl_topic
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  cmd_vel_halver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS map_to_stl_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS map_to_world_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS goal_pose_to_nav2 DESTINATION lib/${PROJECT_NAME})
install(TARGETS map_to_stl_topic DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  goal_pose_to_nav2
  DESTINATION lib/${PROJECT_NAME}
)
set(CMAKE_CXX_STANDARD 17)


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
