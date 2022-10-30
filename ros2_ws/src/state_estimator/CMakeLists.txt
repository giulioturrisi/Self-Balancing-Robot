cmake_minimum_required(VERSION 3.5)
project(state_estimator)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)



add_executable(complementary_filter src/complementary_filter.cpp)

ament_target_dependencies(complementary_filter rclcpp sensor_msgs tf2_msgs geometry_msgs tf2_ros)


install(TARGETS
  complementary_filter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
