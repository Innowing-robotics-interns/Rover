cmake_minimum_required(VERSION 3.5)
project(pcl_filter)

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
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters segmentation search) # Added filters, segmentation, and search components

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add definitions (optional, depending on PCL configuration)
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ executable
add_executable(point_cloud_filter_node src/PointCloudFilter.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(point_cloud_filter_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(point_cloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions # Ensure pcl_conversions is linked
)

# Link against PCL components if necessary
# This step is sometimes needed if PCL components are not automatically linked by ${PCL_LIBRARIES}
target_link_libraries(point_cloud_filter_node
  pcl_common
  pcl_io
  pcl_filters
  pcl_segmentation
  pcl_search
)

# Install the executable
install(TARGETS
  point_cloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs pcl_conversions)

ament_package()