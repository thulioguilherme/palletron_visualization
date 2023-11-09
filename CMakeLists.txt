# Set minimum required version of cmake, project name and compile options
cmake_minimum_required(VERSION 3.8)
project(palletron_visualization)

# Find ament packages and libraries for ament and system dependencies
find_package(ament_cmake REQUIRED)

# Install
install(
  DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Macro for ament package
ament_package()