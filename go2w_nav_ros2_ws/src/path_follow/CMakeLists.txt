cmake_minimum_required(VERSION 3.5)
project(path_follow)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
# Keep the original executable as a compatibility entry point.
install(PROGRAMS path_follow/path_follow_py_node.py
  RENAME path_follow_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS path_follow/path_follow_py_node.py
  RENAME path_follow_py_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Do not load unrelated user-installed pytest plugins in this isolated suite.
  set(saved_pytest_autoload "$ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD}")
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(test_controller test/test_controller.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  if(saved_pytest_autoload STREQUAL "")
    unset(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD})
  else()
    set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "${saved_pytest_autoload}")
  endif()
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_path_follow_launch.py TIMEOUT 90)
  # Isolate synthetic paths and commands from any running robot ROS graph.
  set_tests_properties(test_test_path_follow_launch.py PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=187")
endif()
ament_package()
