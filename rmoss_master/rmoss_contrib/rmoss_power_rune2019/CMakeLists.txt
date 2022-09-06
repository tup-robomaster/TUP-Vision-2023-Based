cmake_minimum_required(VERSION 3.5)
project(rmoss_power_rune2019)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(rmoss_util REQUIRED)
find_package(rmoss_cam REQUIRED)
find_package(rmoss_projectile_motion REQUIRED)
find_package(OpenCV REQUIRED)

# Include
include_directories(include)

# Create rmoss_power_rune2019 lib
add_library(${PROJECT_NAME} SHARED
  src/power_rune_algo/rune_detector.cpp
  src/power_rune_algo/rune_prediction_tool.cpp
  src/power_rune_algo/rune_state_machine.cpp
  src/power_rune_algo/simple_power_rune_algo.cpp
  src/power_rune_node.cpp
)

set(dependencies
  rclcpp 
  rmoss_interfaces
  rmoss_util
  rmoss_cam
  rmoss_projectile_motion
  OpenCV
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Create executable nodes
add_executable(power_rune src/power_rune_main.cpp)
target_link_libraries(power_rune ${PROJECT_NAME})


# Install launch res directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executable nodes
install(TARGETS power_rune 
  DESTINATION lib/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS 
        scripts/client.py
    DESTINATION lib/${PROJECT_NAME})
    
#export rmoss_auto_aim dependency
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()