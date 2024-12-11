cmake_minimum_required(VERSION 3.5)
project(moveit_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

install(PROGRAMS
  moveit_controller/controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller
)
install(PROGRAMS
  moveit_controller/part_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME part_manager
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_python_install_package(${PROJECT_NAME})

ament_package()