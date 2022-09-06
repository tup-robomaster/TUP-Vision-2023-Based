cmake_minimum_required(VERSION 3.15)

project(test)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")

add_executable(test test.cpp
            ${PROJECT_SOURCE_DIR}/DxImageProc.h
            ${PROJECT_SOURCE_DIR}/GxIAPI.h
)

target_link_libraries(test gxiapi)
# target_link_libraries(test "/home/dishierweidu/Documents/auto/rmoss/src/rmoss_daheng_driver/dependencies/DaHengGxiSDK/lib/x86_64/libgxiapi.so")
