cmake_minimum_required(VERSION 3.8)
project(armor_detector  LANGUAGES CXX CUDA)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=16000000")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall -fopenmp")
set(CUDA_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CUDA_GEN_CODE "-gencode=arch=compute_75,code=sm_75")
set(CMAKE_CUDA_ARCHITECTURES 75)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++14 -O3 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CUDA REQUIRED)
find_package(CUDNN REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(yaml-cpp REQUIRED)
find_package(angles REQUIRED)
# custom package
find_package(global_user REQUIRED)
find_package(global_interface REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CUDA_INCLUDE_DIRS}
  ${CUDNN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp
  rclcpp_components
  rclpy
  OpenCV 
  message_filters 
  image_transport
  sensor_msgs
  CUDA
  CUDNN
  Eigen3
  angles
  cv_bridge
  yaml-cpp 
  global_interface
  global_user
)
  
cuda_add_library(${PROJECT_NAME} SHARED 
  src/armor_tracker/armor_tracker.cpp 
  src/spinning_detector/spinning_detector.cpp 
  src/armor_detector/armor_detector.cpp
  src/TRTInfer/Inference.cpp
  src/TRTInfer//preprocess.cu
  src/detector_node.cpp
)
  
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${CUDNN_LIBRARIES}
  nvinfer
  cudart
  nvonnxparser
  yaml-cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

# set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
# target_compile_definitions(${PROJECT_NAME}
# PRIVATE "COMPOSITION_BUILDING_DLL")
  
cuda_add_executable(${PROJECT_NAME}_node
  src/armor_tracker/armor_tracker.cpp 
  src/armor_detector/armor_detector.cpp 
  src/spinning_detector/spinning_detector.cpp
  src/TRTInfer/Inference.cpp
  src/TRTInfer//preprocess.cu
  src/detector_node.cpp 
)
set_target_properties(PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${CUDNN_LIBRARIES}
  nvinfer
  cudart
  nvonnxparser
  yaml-cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "armor_detector::DetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
