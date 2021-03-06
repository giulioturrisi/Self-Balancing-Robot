cmake_minimum_required(VERSION 3.5)
project(controllers)

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


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)


add_executable(lqr src/lqr.cpp)
add_executable(pd src/pd.cpp)
ament_target_dependencies(lqr rclcpp tf2_msgs tf2 std_msgs)
ament_target_dependencies(pd rclcpp tf2_msgs tf2 std_msgs)


install(TARGETS
  lqr
  pd
  DESTINATION lib/${PROJECT_NAME})



ament_package()
