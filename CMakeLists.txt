cmake_minimum_required(VERSION 3.5)
project(ros2_pinocchio_demo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# Build the library
add_library(pinocchio_demo_lib
  src/pinocchio_node.cpp
)
target_include_directories(pinocchio_demo_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(pinocchio_demo_lib
  rclcpp std_msgs urdf pinocchio
)

# Create executable
add_executable(pinocchio_node src/pinocchio_node.cpp)
target_link_libraries(pinocchio_node pinocchio_demo_lib)

# Install targets
install(TARGETS pinocchio_demo_lib pinocchio_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()