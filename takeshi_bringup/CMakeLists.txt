cmake_minimum_required(VERSION 3.8)
project(takeshi_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python executables
install(
  PROGRAMS
    scripts/cloud_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()