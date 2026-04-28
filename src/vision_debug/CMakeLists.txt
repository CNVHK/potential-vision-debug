cmake_minimum_required(VERSION 3.8)
project(vision_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找构建依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(cv_bridge REQUIRED)   
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

add_executable(vision_node
  src/ros_main.cpp
  src/auto_aim_pipeline.cpp
  src/vision_input_buffer.cpp
  src/vision_ros_publisher.cpp
  src/aimer.cpp
  src/armor.cpp
  src/classifier.cpp
  src/detector.cpp
  src/light_corner_corrector.cpp
  src/shooter.cpp
  src/solver.cpp
  src/target.cpp
  src/tracker.cpp
  src/tool/extended_kalman_filter.cpp
  src/tool/math_tools.cpp
  src/tool/trajectory.cpp
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS vision_node
  DESTINATION lib/${PROJECT_NAME}
)
add_compile_options(-g -fno-omit-frame-pointer)
ament_target_dependencies(vision_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  rm_interfaces
  geometry_msgs
  OpenCV
  visualization_msgs
)

target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/tool>
)
target_link_libraries(vision_node
  Eigen3::Eigen  
  yaml-cpp
  fmt::fmt
  tbb
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 当前源码版权声明不统一，暂时跳过版权检查。
  set(ament_cmake_copyright_FOUND TRUE)
  # 当前仍处于接手重构阶段，暂时跳过 cpplint。
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
