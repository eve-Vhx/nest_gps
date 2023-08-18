cmake_minimum_required(VERSION 3.5)
project(nest_gps)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# This line installs Python script as an executable
install(PROGRAMS
  nest_gps/nest_gps_t.py
  nest_gps/nest_sync.py
  nest_gps/nest_sync_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
