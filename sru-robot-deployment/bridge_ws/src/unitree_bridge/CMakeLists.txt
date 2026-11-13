cmake_minimum_required(VERSION 3.8)
project(unitree_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/common include/nlohmann)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)

add_executable(unitree_bridge src/unitree_bridge.cpp)
ament_target_dependencies(unitree_bridge
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  unitree_api
  unitree_go
)

add_executable(sport_mode_subscriber src/sport_mode_subscriber.cpp)
ament_target_dependencies(sport_mode_subscriber
  rclcpp
  unitree_go
)

add_executable(g1_cmd_vel_bridge src/g1_cmd_vel_bridge.cpp)
target_compile_features(g1_cmd_vel_bridge PRIVATE cxx_std_20)
ament_target_dependencies(g1_cmd_vel_bridge
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
  unitree_hg
)

install(TARGETS
  unitree_bridge
  sport_mode_subscriber
  g1_cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 