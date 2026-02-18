cmake_minimum_required(VERSION 3.8)
project(relative_pose_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  sim/launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install models directory
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds directory
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/leader_controller.py
  scripts/follower_controller.py
  scripts/data_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
