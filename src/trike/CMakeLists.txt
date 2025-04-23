cmake_minimum_required(VERSION 3.8)
project(trike)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables
add_executable(servo_controller src/embedded/servo_controller.cpp)
add_executable(steering_controller src/embedded/steering_controller.cpp)

# Link dependencies
ament_target_dependencies(servo_controller rclcpp std_msgs)
ament_target_dependencies(steering_controller rclcpp std_msgs)

# Include directories
foreach(target servo_controller steering_controller)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
endforeach()

# Install targets
install(TARGETS servo_controller steering_controller
  DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.urdf"
  PATTERN "*.xacro"
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.yaml"
)

install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
  # has yaml and png files
  PATTERN "*.yaml"
  PATTERN "*.png"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
