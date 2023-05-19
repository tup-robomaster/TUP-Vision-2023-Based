cmake_minimum_required(VERSION 3.8)
project(global_user)

set(CMAKE_BUILD_TYPE=Release)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(yaml-cpp REQUIRED)

set(dependencies
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  Eigen3
  Ceres
  yaml-cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 添加源文件，生成库
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/coordsolver.cpp
)

# 用于代替传统的target_link_libraries
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  yaml-cpp
)

# 添加头文件地址
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# 为了使下游文件可以访问
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   ${dependencies}
# )

# 注册 导出头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 注册 导出库文件
install(
  TARGETS ${PROJECT_NAME} # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  launch
  config 
  data
  DESTINATION share/${PROJECT_NAME}
)

# specific order: dependents before dependencies
#ament_export_targets(${PROJECT_NAME})
#ament_export_dependencies(${dependencies})        
        
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(test)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
      
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()