cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 添加可执行文件
add_executable(waypoint_generator 
  src/waypoint_generator.cpp
  src/sample_waypoints.hpp
)

# 3. 包含头文件目录
# 注意：我们将 sample_waypoints.hpp 放在了 src 下，所以包含 CMAKE_CURRENT_SOURCE_DIR/src
target_include_directories(waypoint_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${Eigen3_INCLUDE_DIRS}
)

# 4. 链接依赖
ament_target_dependencies(waypoint_generator
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)

# 5. 安装规则 (让 ros2 run 能找到它)
install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME})

# 拷贝 launch 文件夹（如果以后有的话）
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})

ament_package()