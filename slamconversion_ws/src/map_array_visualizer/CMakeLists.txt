cmake_minimum_required(VERSION 3.8)
project(map_array_visualizer)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Declare a C++ executable
add_executable(map_array_visualizer src/map_array_visualizer.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(map_array_visualizer
  rclcpp
  nav_msgs
)

# Install the executable
install(TARGETS
  map_array_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE) # Skip cpplint
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()