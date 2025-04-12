cmake_minimum_required(VERSION 3.8)
project(pb2roscpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

set(RCLCPP_LOCAL_BINARY_NAME pb2roscpp)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
  src/pb2roscpp.cpp
  src/floatarray.pb.c
  src/pb_common.c
  src/pb_decode.c
  src/pb_encode.c)

ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME} rclcpp std_msgs geometry_msgs serial)

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
  DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
