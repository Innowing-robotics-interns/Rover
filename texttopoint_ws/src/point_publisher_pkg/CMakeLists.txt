cmake_minimum_required(VERSION 3.5)
project(point_publisher_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare a C++ executable
add_executable(point_publisher src/point_publisher.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(point_publisher
  rclcpp
  geometry_msgs
)

# Install the executable
install(TARGETS
  point_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()