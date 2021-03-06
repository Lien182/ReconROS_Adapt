cmake_minimum_required(VERSION 3.5)
project(reference_application)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(mnist_msgs REQUIRED)
find_package(sorter_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(standard_executor src/main.cpp src/inverse/inverse.c src/sort/sort.c src/mnist/mnist.cpp src/mnist/pooling.cpp src/mnist/LeNet.cpp src/mnist/fullyconnected.cpp src/mnist/convolution.cpp src/mnist/activation.cpp src/sobel/sobel.c src/periodic/periodic.c)
ament_target_dependencies(standard_executor rclcpp mnist_msgs sorter_msgs sensor_msgs)
ament_export_include_directories(include)

install(TARGETS standard_executor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
