cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 添加-fPIC编译，即：装成一个动态库
# option(WITH_MVSSDK "Build with hikMVSSDK" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

find_package(global_user REQUIRED)
find_package(global_interface REQUIRED)

include_directories(include ${cv_bridge_INCLUDE_DIRS})
include_directories("dependencies/hik_sdk/include")
include_directories("dependencies/daheng_sdk/include")
link_directories(/opt/MVS/lib/64/)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  camera_info_manager
  image_transport
  camera_calibration_parsers
  tf2
  OpenCV
  Eigen3
  yaml-cpp
  global_user
  global_interface 
)

add_library(${PROJECT_NAME} SHARED
  src/usb_driver/usb_cam.cpp
  src/usb_driver/usb_cam_node.cpp
  src/hik_driver/hik_camera.cpp
  src/hik_driver/hik_cam_node.cpp
  src/daheng_driver/daheng_camera.cpp
  src/daheng_driver/daheng_cam_node.cpp
  src/camera_driver/camera_driver_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  MvCameraControl
  MVRender
  FormatConversion
  MediaProcess
  gxiapi
  yaml-cpp.so
)
# target_compile_definitions(${PROJECT_NAME}
#   PRIVATE "COMPOSITION_BUILDING_DLL"
# )

add_executable(usb_cam_driver_node src/usb_driver/usb_cam_node_main.cpp)
ament_target_dependencies(usb_cam_driver_node ${dependencies})
target_link_libraries(usb_cam_driver_node ${PROJECT_NAME} yaml-cpp.so)

add_executable(hik_cam_driver_node src/hik_driver/hik_cam_node_main.cpp)
ament_target_dependencies(hik_cam_driver_node ${dependencies})
target_link_libraries(hik_cam_driver_node ${PROJECT_NAME} yaml-cpp.so)

add_executable(daheng_cam_driver_node src/daheng_driver/daheng_cam_node_main.cpp)
ament_target_dependencies(daheng_cam_driver_node ${dependencies})
target_link_libraries(daheng_cam_driver_node ${PROJECT_NAME} gxiapi yaml-cpp.so)

rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "camera_driver::UsbCamNode"
  EXECUTABLE usb_cam_driver_node 
)
rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "camera_driver::HikCamNode"
  EXECUTABLE hik_cam_driver_node
)
rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "camera_driver::DahengCamNode"
  EXECUTABLE daheng_cam_driver_node
)
    
install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS 
  usb_cam_driver_node 
  hik_cam_driver_node
  daheng_cam_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# specific order: dependents before dependencies
# ament_export_targets(export_${PROJECT_NAME})
# ament_export_dependencies(${dependencies})

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
