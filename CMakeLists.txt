cmake_minimum_required(VERSION 3.8)
project(agilex_piper_arm_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install config, launch, urdf, and test files
install(
  DIRECTORY
    config
    launch
    urdf
    test
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
