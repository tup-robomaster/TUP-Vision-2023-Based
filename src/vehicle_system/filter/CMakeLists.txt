cmake_minimum_required(VERSION 3.8)
project(filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(matplotlib_cpp REQUIRED)

set(dependencies
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  angles
  OpenCV
  Eigen3
  Ceres

  global_user
  global_interface
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
  

# 添加源文件，生成库
add_library(${PROJECT_NAME} SHARED
# Utils
  include/filter/filter.hpp
  
  # Base
  include/motion_model/kalman_filter_base.hpp
  
  # Models
  include/motion_model/measurement_model.hpp
  include/motion_model/system_model.hpp
  include/motion_model/linear_measurement_model.hpp
  include/motion_model/linear_system_model.hpp
  
  # KF
  include/kalman_filter.hpp
  src/kalman_filter.cpp 
  
  # EKF
  include/extend_kalman_filter.hpp
  
  # PF
  include/particle_filter.hpp
  src/particle_filter.cpp
  
  # UKF
)
  
target_link_libraries(${PROJECT_NAME}
  # ${dependencies}
  ${OpenCV_LIBRARIES}
  yaml-cpp.so
  fmt.so 
  glog.so
)

add_executable(testing 
  test/imm_system_model.hpp
  test/imm.hpp
  test/imm.cpp
  test/system_model.cpp 
  test/system_model.hpp
  test/measurement_model.cpp
  test/main.cpp
)

target_link_libraries(testing
  ${PROJECT_NAME}
  ${Eigen3_LIBRARIES}
  ceres.so
  matplotlib_cpp::matplotlib_cpp
)

add_executable(figure
  test/figure.cpp
)

target_link_libraries(figure
  matplotlib_cpp::matplotlib_cpp
)
      
# 添加头文件地址
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 为了使下游文件可以访问
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${dependencies}
)

# 注册 导出头文件
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  testing 
  figure
  DESTINATION lib/${PROJECT_NAME}
)

## TODO:
# install(DIRECTORY
#   # launch
#   # config 
#   DESTINATION share/${PROJECT_NAME}
# )

# 注册 导出库文件
install(
  TARGETS ${PROJECT_NAME} # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# specific order: dependents before dependencies
#ament_export_targets(${PROJECT_NAME})
#ament_export_dependencies(${dependencies})        
        
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