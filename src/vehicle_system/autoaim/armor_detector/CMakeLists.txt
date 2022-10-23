cmake_minimum_required(VERSION 3.8)
project(armor_detector)

# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_PREFIX_PATH /opt/intel/openvino_2021/deployment_tools/inference_engine/share)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(InferenceEngine REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(Eigen3 REQUIRED)

find_package(cv_bridge REQUIRED)

# custom package
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

set(dependencies 
  rclcpp
  rclpy
  OpenCV 
  message_filters 
  image_transport
  sensor_msgs
  # InferenceEngine 
  OpenVINO
  Ceres
  Eigen3
  cv_bridge
  
  global_interface
  global_user
  
  yaml-cpp 
  fmt
)
  
include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${InferenceEngine_INCLUDE_DIRS}
  ${OpenVINO_INCLUDE_DIRS}
)
  
include_directories(include)
# add_library(${PROJECT_NAME}
  # ./include/armor_detector/armor_tracker.h 
  # ./include/armor_detector/inference.h 
  # ./include/armor_detector/detector.hpp
  # ./include/armor_detector/detector_node.hpp
  # ./src/armor_detector/inference.cpp
  # src/armor_detector/armor_tracker.cpp 
  # src/armor_detector/detector.cpp 
  # src/armor_detector/spinning_detector.cpp
# )
  
# ament_target_dependencies(${PROJECT_NAME}
#   ${dependencies}  
# )
  
  # set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  # target_compile_definitions(${PROJECT_NAME}
  # PRIVATE "COMPOSITION_BUILDING_DLL")
  
add_executable(armor_detector_node
  # ./include/armor_detector/inference.h 
  # ./include/armor_detector/spinning_detector.hpp
  src/armor_detector/armor_tracker.cpp 
  src/armor_detector/detector.cpp 
  src/armor_detector/spinning_detector.cpp
  include/armor_detector/inference_api2.hpp
  src/armor_detector/inference_api2.cpp
  src/armor_detector/detector_node.cpp 
)

# add_executable(testing
#   # ./include/armor_detector/inference.h
#   # ./src/armor_detector/inference.cpp
#   ./src/test/test.cpp 
# )

# rclcpp_components_register_nodes(${PROJECT_NAME}
#   PLUGIN "armor_detector::detector_node"
#   EXECUTABLE armor_detector_node
# )

ament_target_dependencies(armor_detector_node
  ${dependencies}
)

target_link_libraries(armor_detector_node
  # ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  # ${InferenceEngine_LIBRARIES}
  openvino::runtime 
  ${Eigen3_LIBRARIES}
  
  yaml-cpp.so 
  glog.so 
  fmt.so
)

# target_link_libraries(testing
#   ${PROJECT_NAME}
#   ${InferenceEngine_LIBRARIES}
# )

# install(TARGETS 
#   ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib
#   INCLUDES DESTINATION include
# )
  
install(TARGETS 
  armor_detector_node
  # testing 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  # config
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
