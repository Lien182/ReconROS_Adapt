cmake_minimum_required(VERSION 3.5)
project(reference_application_periodic_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mnist_msgs REQUIRED)
find_package(sorter_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 QUIET)
find_package(cv_bridge REQUIRED)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(periodic_client src/main.cpp)

ament_target_dependencies(periodic_client rclcpp mnist_msgs sorter_msgs sensor_msgs OpenCV cv_bridge)
ament_export_include_directories(include ${OpenCV_INCLUDE_DIRS})

install(TARGETS periodic_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
