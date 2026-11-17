cmake_minimum_required(VERSION 3.8)
project(local_planner)

# 本包同时生成自定义消息并编译规划核心，不依赖原 scan_planner 中的功能包。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)
set(local_planner_dependencies
  cv_bridge geometry_msgs message_filters nav_msgs pcl_conversions rclcpp
  sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
foreach(dependency IN LISTS local_planner_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)

add_library(local_planner_core
  src/plan_env/grid_map.cpp
  src/plan_env/raycast.cpp
  src/path_searching/dyn_a_star.cpp
  src/bspline_opt/uniform_bspline.cpp
  src/bspline_opt/bspline_optimizer.cpp
  src/bspline_opt/gradient_descent_optimizer.cpp
  src/traj_utils/planning_visualization.cpp
  src/traj_utils/polynomial_traj.cpp
  src/plan_manage/planner_manager.cpp)
target_include_directories(local_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(local_planner_core ${local_planner_dependencies})
target_link_libraries(local_planner_core Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(local_planner_node
  src/plan_manage/local_planner_node.cpp
  src/plan_manage/local_replan_fsm.cpp)
target_link_libraries(local_planner_node local_planner_core)
ament_target_dependencies(local_planner_node ${local_planner_dependencies})
# Foxy 中先生成本包消息头，再编译使用这些消息的规划节点。
rosidl_target_interfaces(local_planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 直接按时间执行本包 B 样条，发布机体 Twist，并把朝向对齐冻结状态反馈给规划器。
add_executable(closed_loop_controller src/plan_manage/closed_loop_controller.cpp)
target_link_libraries(closed_loop_controller local_planner_core)
ament_target_dependencies(closed_loop_controller ${local_planner_dependencies})
rosidl_target_interfaces(closed_loop_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS local_planner_core local_planner_node closed_loop_controller
  EXPORT export_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_local_planner HAS_LIBRARY_TARGET)
ament_export_dependencies(${local_planner_dependencies} Eigen3 PCL OpenCV rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # 算法单测不依赖机器人；接口测试使用独立 ROS 域，避免影响运行中的导航节点。
  foreach(test_name reference_path_utils reference_frame_utils reference_height uniform_bspline bspline_path initial_path_sampling)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} local_planner_core)
  endforeach()
  add_launch_test(test/test_global_path_launch.py TIMEOUT 40)
  add_launch_test(test/test_dynamic_reference_tf_launch.py TIMEOUT 35)
  add_launch_test(test/test_replan_limits_launch.py TIMEOUT 30)
  add_launch_test(test/test_closed_loop_controller_launch.py TIMEOUT 45)
  add_launch_test(test/test_local_controller_launch.py TIMEOUT 35)
  set_tests_properties(test_test_global_path_launch.py test_test_dynamic_reference_tf_launch.py
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=174")
  set_tests_properties(test_test_replan_limits_launch.py
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=176")
  set_tests_properties(test_test_closed_loop_controller_launch.py
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=177")
  set_tests_properties(test_test_local_controller_launch.py
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=178")
endif()

ament_package()
