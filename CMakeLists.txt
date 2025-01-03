cmake_minimum_required(VERSION 3.5)
project(robot_urdf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED) # Add rclpy for Python nodes

# Install Python nodes
install(PROGRAMS
  scripts/move_robot_client.py
  scripts/move_robot_node.py  # Add your Python node here
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch, urdf, config, worlds)
install(DIRECTORY
  launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment the following lines to skip specific linters
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
