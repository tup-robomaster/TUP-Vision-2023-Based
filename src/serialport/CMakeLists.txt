cmake_minimum_required(VERSION 3.8)
project(serialport)

## Use C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
# SET(CMAKE_BUILD_TYPE "Debug")  
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# By adding -Wall and -Werror, the compiler does not ignore warnings anymore, enforcing cleaner code.
# add_definitions(-Wall -Werror)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(OpenCV REQUIRED)
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
  
set(node_plugins "")
  
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/crc_check.cpp 
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
  src/data_processor/data_transform.cpp
  src/${PROJECT_NAME}_node.cpp
)
  
set(dependencies 
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  OpenCV
  global_interface
  global_user
  message_filters 
  robot_state_publisher 
  # yaml-cpp
  Eigen3
  Ceres
  tf2_geometry_msgs
  tf2_ros 
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}/crc_check.cpp
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
  src/data_processor/data_transform.cpp
  src/${PROJECT_NAME}_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node 
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_node 
  yaml-cpp
)

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "serialport::SerialPortNode"
  EXECUTABLE ${PROJECT_NAME}_node  
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
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# install inlude directories
install(
    DIRECTORY include/
    DESTINATION include
)

# install launch resource config directory
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()