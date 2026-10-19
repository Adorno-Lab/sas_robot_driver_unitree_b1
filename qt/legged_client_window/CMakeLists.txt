# Qt application to test a legged robot driver of sas_tools (e.g. the Unitree B1, optionally with the Z1) through
# LeggedRobotDriverClient. It is not a colcon package: build it with Qt Creator, or with CMake, in a terminal in which
# ROS 2 and the workspace that contains sas_tools are sourced.
cmake_minimum_required(VERSION 3.19)
project(legged_client_window LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_robot_driver REQUIRED)
find_package(sas_tools REQUIRED)

qt_standard_project_setup()

qt_add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.h
)

# The keyword signature, since qt_add_executable() uses it.
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    Eigen3
    sas_core
    sas_common
    sas_robot_driver
    sas_tools
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Widgets
    dqrobotics
)
