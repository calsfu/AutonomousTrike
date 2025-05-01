cmake_minimum_required(VERSION 3.8)
project(trike)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define the list of C++ executable targets
set(CPP_TARGET_NAMES
  servo_controller
  steering_controller
  data_manager
  twist_to_steer
)

# Add executables
add_executable(servo_controller src/embedded/servo_controller.cpp)
add_executable(steering_controller src/embedded/steering_controller.cpp)
add_executable(data_manager src/data/data_manager.cpp)
add_executable(twist_to_steer src/data/twist_to_steer.cpp)


# Apply common properties in a loop
foreach(target IN LISTS CPP_TARGET_NAMES)
  ament_target_dependencies(${target} rclcpp std_msgs geometry_msgs)

  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)
endforeach()

# C++ install targets using the list
install(TARGETS ${CPP_TARGET_NAMES}
  DESTINATION lib/${PROJECT_NAME})

# Other install directives
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(
  ${PROJECT_NAME}
  PACKAGE_DIR src/audio
  PACKAGE_DIR src/navigation
  PACKAGE_DIR src/controller
  PACKAGE_DIR src/perception
  PACKAGE_DIR src/key_op
  PACKAGE_DIR src/networking
)

# Install Python scripts/executables
install(PROGRAMS
  src/audio/audio_player.py
  src/audio/speech_to_text.py
  src/navigation/controller_line.py
  src/controller/controller.py
  src/controller/mode_manager.py
  src/controller/controller_line.py
  src/perception/emergency_stop.py
  src/perception/image_segment.py
  src/key_op/key_op.py
  src/key_op/key_op2.py
  src/networking/ros_get_from_docker.py
  src/networking/ros_send_into_docker.py
  DESTINATION lib/${PROJECT_NAME}
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

install(DIRECTORY audio
  DESTINATION share/${PROJECT_NAME}
  # has yaml and png files
  PATTERN "*.wav"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
