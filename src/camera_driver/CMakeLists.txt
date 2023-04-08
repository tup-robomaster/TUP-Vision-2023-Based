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
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(fmt REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(global_user REQUIRED)
find_package(global_interface REQUIRED)

include_directories(include ${cv_bridge_INCLUDE_DIRS})
include_directories("dependencies/hik_sdk/include")
include_directories("dependencies/daheng_sdk/include")

# if(WITH_MVSSDK)
#   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    # set(MVS_LIB_DIR "dependencies/hik_sdk/MVS/lib/64")
    # link_directories(${MVS_LIB_DIR})
#   endif()
  # set(MVS_LIB_SO "${PROJECT_SOURCE_DIR}/${MVS_LIB_DIR}/libMvCameraControl")
# endif()
# elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#   target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/MVS/lib/64)
#   install(
  #     DIRECTORY hikSDK/lib/arm64/
#     DESTINATION lib
# )

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(/opt/MvCamCtrlSDK/lib/64/)

# set(GXI_LIB_SO usr/lib/libgxiapi)

# add_library(${PROJECT_NAME}
  # include/hik_camera.h
  # src/hik_camera.cpp
# )

set(dependencies 
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  camera_info_manager
  image_transport
  camera_calibration_parsers
  OpenCV
  Eigen3
  glog
  fmt 
  yaml-cpp
  global_user
  global_interface 
)

include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/usb_driver/usb_cam.cpp
  src/usb_driver/usb_cam_node.cpp
  src/hik_driver/hik_camera.cpp
  src/hik_driver/hik_cam_node.cpp
  src/daheng_driver/daheng_camera.cpp
  src/daheng_driver/daheng_cam_node.cpp
)
  
# # DaHeng相机
# if(WITH_MVSSDK)
#   target_link_libraries(${PROJECT_NAME} ${MVS_LIB_DIR})
# endif()

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

# target_include_directories(${PROJECT_NAME}
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  MvCameraControl
  MVRender
  FormatConversion
  MediaProcess
  gxiapi
  yaml-cpp
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# add_library(hik_cam_component SHARED
#   src/hik_cam_node.cpp)

# target_compile_definitions(hik_cam_component
#   PRIVATE "COMPOSITION_BUILDING_DLL")

# ament_target_dependencies(hik_cam_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs")


# install cam driver lib
# install(
#     FILES ${GXI_LIB_SO}
#     DESTINATION lib
# )

# add_library(daheng_cam_component SHARED
#   src/daheng_cam_node.cpp)
# target_compile_definitions(daheng_cam_component
#   PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(daheng_cam_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs")
# rclcpp_components_register_nodes(daheng_cam_component "camera_driver::daheng_cam_node")
# set(node_plugins "${node_plugins}camera_driver::daheng_cam_node;$<TARGET_FILE:daheng_cam_component>\n")

# link_directories(${MVS_LIB_DIR})
add_executable(usb_cam_driver_node
  src/usb_driver/usb_cam.cpp
  src/usb_driver/usb_cam_node.cpp
  src/usb_driver/usb_cam_node_main.cpp
)

add_executable(hik_cam_driver_node
  src/hik_driver/hik_camera.cpp
  src/hik_driver/hik_cam_node.cpp
  src/hik_driver/hik_cam_node_main.cpp
)

add_executable(daheng_cam_driver_node
  src/daheng_driver/daheng_camera.cpp
  src/daheng_driver/daheng_cam_node.cpp
  src/daheng_driver/daheng_cam_node_main.cpp
)

# target_include_directories(hik_cam_driver_node PUBLIC hik_sdk/include)
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
# target_link_directories(hik_cam_driver_node PUBLIC dependencies/hik_sdk/MVS/lib/64)
# install(
#   DIRECTORY dependencies/hik_sdk/MVS/lib/64/
#   DESTINATION lib
#   )
# endif()

ament_target_dependencies(usb_cam_driver_node
  ${dependencies}
)

ament_target_dependencies(hik_cam_driver_node
  ${dependencies}  
)

ament_target_dependencies(daheng_cam_driver_node
  ${dependencies}  
)

target_link_libraries(usb_cam_driver_node
  ${PROJECT_NAME}
  yaml-cpp
)
  
target_link_libraries(hik_cam_driver_node 
  ${PROJECT_NAME}
  yaml-cpp
)
  
target_link_libraries(daheng_cam_driver_node
  gxiapi
  # fmt
  yaml-cpp
)
    
rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "camera_driver::UsbCamNode"
  EXECUTABLE usb_cam_driver_node 
)
set(node_plugins 
  "${node_plugins}camera_driver::usb_cam_node;&<TARGET_FILE:usb_cam_node\n"
)
  
# rclcpp_components_register_nodes(${PROJECT_NAME} 
#   PLUGIN "camera_driver::HikCamNode"
#   EXECUTABLE hik_cam_driver_node
# )
# set(node_plugins 
#   "${node_plugins}camera_driver::hik_cam_node;$<TARGET_FILE:hik_cam_node>\n"
# )
  
rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "camera_driver::DahengCamNode"
  EXECUTABLE daheng_cam_driver_node
)
set(node_plugins 
  "${node_plugins}camera_driver::daheng_cam_node;$<TARGET_FILE:daheng_cam_node>\n"
)
    
# install cam driver lib
# install(
#   FILES ${MVS_LIB_SO}
#   DESTINATION lib
# )

# 包含海康相机动态链接库
# find_library(MVS_LIB_DIR ${MVS_LIB_SO})
# target_link_libraries(hik_cam_driver_node MvCameraControl)

# install(TARGETS
#   hik_cam_component
#   # daheng_cam_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


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
