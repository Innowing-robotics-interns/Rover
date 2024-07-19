cmake_minimum_required(VERSION 3.8)
project(dstar_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# find include
include_directories(include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io) # 根据需要，你可以添加或删除特定的组件
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# launch setting
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 如果你有配置文件或参数文件也需要安装
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# add dstar_main executable
add_executable(dstar_main src/dstar_main.cpp) # Assuming the file extension should be .cpp
add_executable(dwa_main src/dwa_main.cpp) # Assuming the file extension should be .cpp
add_executable(scan2grid src/scan2grid.cpp) # Assuming the file extension should be .cpp
add_executable(map2grid src/map2grid.cpp) # Assuming the file extension should be .cpp
# 安装可执行文件
install(TARGETS
  dstar_main
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  dwa_main
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  scan2grid
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  map2grid
  DESTINATION lib/${PROJECT_NAME}
)
# link libraries for dstar_main executable
ament_target_dependencies(dstar_main
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)
target_link_libraries(dstar_main
  ${PCL_LIBRARIES}
)

# link libraries for dstar_main executable
ament_target_dependencies(dwa_main
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)
target_link_libraries(dwa_main
  ${PCL_LIBRARIES}
)
# link libraries for dstar_main executable
ament_target_dependencies(scan2grid
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)
target_link_libraries(scan2grid
  ${PCL_LIBRARIES}
)
ament_target_dependencies(map2grid
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
)
target_link_libraries(map2grid
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
