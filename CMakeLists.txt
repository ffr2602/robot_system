cmake_minimum_required(VERSION 3.8)
project(robot_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch scripts description map world rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joy_control.py
  scripts/omni_controller.py
  scripts/robot_control_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
