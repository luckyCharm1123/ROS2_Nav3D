cmake_minimum_required(VERSION 3.8)
project(rviz_3d_nav_goal_tool)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(
  include
)

set(dependencies
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  tf2
  tf2_geometry_msgs
  pluginlib
  Qt5
)

add_library(rviz_3d_nav_goal_tool SHARED
  src/goal_tool.cpp
  src/pose_tool.cpp
  include/rviz_3d_nav_goal_tool/goal_tool.hpp
  include/rviz_3d_nav_goal_tool/pose_tool.hpp
)

target_include_directories(rviz_3d_nav_goal_tool PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rviz_3d_nav_goal_tool
  ${dependencies}
)

# 注册插件定义文件
pluginlib_export_plugin_description_file(rviz_common rviz_plugin.xml)

install(TARGETS rviz_3d_nav_goal_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES rviz_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()