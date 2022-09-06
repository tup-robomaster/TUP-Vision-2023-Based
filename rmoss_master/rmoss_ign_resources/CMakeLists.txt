cmake_minimum_required(VERSION 3.5)
project(rmoss_ign_resources)

# ros package
find_package(ament_cmake REQUIRED)

#install
install( DIRECTORY
     resource
  DESTINATION share/${PROJECT_NAME}
)

#environment hooks, ROS 2 >=Eloquent
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ignition.dsv.in")

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
