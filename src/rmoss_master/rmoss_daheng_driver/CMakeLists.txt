cmake_minimum_required(VERSION 3.8)
project(rmoss_daheng_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(rmoss_cam REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 设置依赖列表
set(dependencies
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  rmoss_interfaces
  rmoss_cam
)

# options
option(WITH_GXISDK "Build with DaHengGxiSDK" ON)

message(STATUS "System architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# DaHengGxiSDK
if(WITH_GXISDK)
    include_directories("dependencies/DaHengGxiSDK/include")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(GXI_LIB_DIR "dependencies/DaHengGxiSDK/lib/x86_64")
        link_directories("dependencies/DaHengGxiSDK/lib/x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(GXI_LIB_DIR "dependencies/DaHengGxiSDK/lib/armv8")
        link_directories("dependencies/DaHengGxiSDK/lib/armv8")
    else()
        message(FATAL_ERROR "Unkown host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
    endif()
    set(GXI_LIB_SO "${PROJECT_SOURCE_DIR}/${GXI_LIB_DIR}/libgxiapi.so")
endif()

# Include
include_directories(include)

set(cam_drivers
  "src/cam_driver/daheng_cam.cpp"
)

# 创建shared lib
add_library(${PROJECT_NAME} SHARED
    ${cam_drivers}
    "src/daheng_cam_node.cpp"
)

# DaHeng相机
if(WITH_GXISDK)
    target_link_libraries(${PROJECT_NAME} gxiapi pthread)
endif()

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# DahengCamera可执行入口
add_executable(daheng_cam "src/daheng_cam_node_main.cpp")
target_link_libraries(daheng_cam ${PROJECT_NAME})

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "rmoss_cam::DaHengCamNode")

# install cam driver lib
install(
    FILES ${GXI_LIB_SO}
    DESTINATION lib
)

# install inlude directories
install(
    DIRECTORY include/
    DESTINATION include
)

# install libraries
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# install executable entry
install(
    TARGETS daheng_cam
    DESTINATION lib/${PROJECT_NAME}
)

# install launch resource config directory
install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()