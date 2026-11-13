cmake_minimum_required(VERSION 3.8)
project(rviz_multi_goals_manager_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# rover_msgs 为自定义消息包，如不存在可注释掉并相应修改源码
find_package(rover_msgs QUIET)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/multi_navi_goal_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
)

set(DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  action_msgs
  rviz2
  rviz_common
  pluginlib
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

if(rover_msgs_FOUND)
  list(APPEND DEPS rover_msgs)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_ROVER_MSGS)
endif()

ament_target_dependencies(${PROJECT_NAME} ${DEPS})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY image/
  DESTINATION share/${PROJECT_NAME}/image
)

ament_package()
