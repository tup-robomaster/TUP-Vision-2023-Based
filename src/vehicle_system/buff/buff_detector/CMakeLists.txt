cmake_minimum_required(VERSION 3.8)
project(buff_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
  
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(filter REQUIRED)
find_package(global_user REQUIRED)
find_package(global_interface REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include 
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENVINO_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp 
  rclcpp_components
  OpenCV
  OpenVINO
  geometry_msgs
  Ceres 
  Eigen3 
  rclpy 
  message_filters
  image_transport
  cv_bridge
  global_user
  global_interface
  filter
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME} SHARED
  src/fan_tracker/fan_tracker.cpp
  src/inference/inference_api2.cpp
  src/buff_detector/buff_detector.cpp
  src/buff_detector_node.cpp

  # test/src/buff_detector/buff_detector.cpp
  # test/src/buff_detector_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  openvino::runtime
)

add_executable(${PROJECT_NAME}_node 
  src/fan_tracker/fan_tracker.cpp
  src/inference/inference_api2.cpp 
  src/buff_detector/buff_detector.cpp 
  src/buff_detector_node.cpp
  
  # test/src/buff_detector/buff_detector.cpp
  # test/src/buff_detector_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}_node 
  openvino::runtime 
  yaml-cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "buff_detector::BuffDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node  
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS 
  ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  model
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
