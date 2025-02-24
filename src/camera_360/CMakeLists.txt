cmake_minimum_required(VERSION 3.8)
project(camera_360)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Define executable
add_executable(ricoh_theta_z1 src/ricoh_theta_z1.cpp)

# Include directories
target_include_directories(ricoh_theta_z1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Compile features
target_compile_features(ricoh_theta_z1 PUBLIC c_std_99 cxx_std_17)
target_compile_options(ricoh_theta_z1 PUBLIC -O3)

# Link dependencies
ament_target_dependencies(
  ricoh_theta_z1
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)

# Install executable
install(TARGETS ricoh_theta_z1
  DESTINATION lib/${PROJECT_NAME})

# Test dependencies (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
