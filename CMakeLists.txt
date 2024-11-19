cmake_minimum_required(VERSION 3.5)
project(moveit_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Link directories
link_directories(/usr/local/lib)  # Path to OpENer library

# Create Cpp executable
add_executable(ethernetip_utils src/ethernetip_utils.cpp)
ament_target_dependencies(ethernetip_utils rclcpp)

target_link_libraries(ethernetip_utils
  CIP  # CIP library
  EIPScanner  # EIP Scanner library
  ENET_ENCAP  # ENET ENCAP library
  NVDATA  # NVDATA library
  PLATFORM_GENERIC  # PLATFORM GENERIC library
  POSIXPLATFORM  # POSIX PLATFORM library
  SAMPLE_APP  # SAMPLE APP library
  Utils
  pthread  # For threading support
)

# Install Cpp executables
install(TARGETS
  ethernetip_utils
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  moveit_controller/controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()