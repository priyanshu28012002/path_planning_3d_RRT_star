cmake_minimum_required(VERSION 3.8)
project(path_planning)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Install headers and launch files
install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

# Create shared library
add_library(RRT SHARED src/rrt_star.cpp)

# Add dependencies to the library
ament_target_dependencies(RRT
  rclcpp
  std_msgs
  geometry_msgs
)

# Create executable
add_executable(2d_rviz_node src/2d_rviz.cpp)
target_link_libraries(2d_rviz_node RRT)
ament_target_dependencies(2d_rviz_node rclcpp std_msgs geometry_msgs visualization_msgs)

add_executable(3d_rviz_node src/3d_rviz.cpp)
target_link_libraries(3d_rviz_node RRT)

# Add dependencies to the executable
ament_target_dependencies(3d_rviz_node rclcpp std_msgs geometry_msgs visualization_msgs)

add_executable(2d_main_node src/2d_main.cpp)
target_link_libraries(2d_main_node RRT)
ament_target_dependencies(2d_main_node rclcpp std_msgs geometry_msgs visualization_msgs)


add_executable(3d_main_node src/3d_main.cpp)
target_link_libraries(3d_main_node RRT)
ament_target_dependencies(3d_main_node rclcpp std_msgs geometry_msgs visualization_msgs)





# Install executable
# Install executable and shared library
install(TARGETS
  2d_rviz_node
  3d_main_node
  2d_main_node
  3d_rviz_node
  RRT
  DESTINATION lib/${PROJECT_NAME}
)
# Linting and testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
