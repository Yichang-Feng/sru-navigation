cmake_minimum_required(VERSION 3.8)
project(rl_nav_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ============================================================================
# C++ executable: depth_preprocess_node
# ============================================================================
add_executable(depth_preprocess_node
  src/depth_preprocess.cpp
)

ament_target_dependencies(depth_preprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(depth_preprocess_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS depth_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Python module
# ============================================================================
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  rl_nav_controller/rl_nav_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rl_nav_controller
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install deployment policies (ONNX models)
install(DIRECTORY deployment_policies/
  DESTINATION share/${PROJECT_NAME}/deployment_policies
)

ament_package()
