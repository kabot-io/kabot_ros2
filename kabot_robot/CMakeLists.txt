cmake_minimum_required(VERSION 3.16)
project(kabot_robot LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(kabot_calibrated_motion SHARED src/calibrated_motion_controller.cpp)
target_compile_features(kabot_calibrated_motion PUBLIC cxx_std_17)
target_include_directories(kabot_calibrated_motion PRIVATE include)
target_link_libraries(kabot_calibrated_motion PRIVATE controller_interface::controller_interface
  hardware_interface::hardware_interface pluginlib::pluginlib realtime_tools::realtime_tools
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle ${control_msgs_TARGETS} ${sensor_msgs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface bringup/calibrated_motion_plugin.xml)
install(TARGETS kabot_calibrated_motion LIBRARY DESTINATION lib)

install(
  DIRECTORY description/launch description/urdf description/rviz description/gazebo
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES description/ros2_control/kabot_mock.ros2_control.xacro
  description/ros2_control/kabot_gazebo.ros2_control.xacro
  DESTINATION share/${PROJECT_NAME}/ros2_control
)
install(FILES bringup/launch/mock.launch.py bringup/launch/gazebo.launch.py
  bringup/launch/native_sim.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(FILES bringup/config/kabot_mock_controllers.yaml bringup/config/kabot_gazebo_controllers.yaml
  bringup/config/kabot_native_sim_controllers.yaml
  bringup/config/kabot_calibrated_controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY bringup/worlds DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_motion_calibration test/test_motion_calibration.cpp)
  target_include_directories(test_motion_calibration PRIVATE include)
  target_compile_features(test_motion_calibration PRIVATE cxx_std_17)
  target_compile_options(test_motion_calibration PRIVATE -UNDEBUG)
  add_test(NAME kabot_motion_calibration COMMAND test_motion_calibration)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(kabot_urdf_xacro test/test_urdf_xacro.py
    ENV KABOT_TEST_INSTALLED=1
  )
  ament_add_pytest_test(kabot_calibrate_motion test/test_calibrate_motion.py)
  ament_add_pytest_test(kabot_drive_loop test/test_drive_loop.py)

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_calibrated_motion.py
    TARGET kabot_calibrated_motion_chain
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    TIMEOUT 60
  )
  add_launch_test(test/test_native_pid.py
    TARGET kabot_native_pid
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_CONTROL_MODE=pid
    TIMEOUT 45
  )
  add_launch_test(test/test_native_pid.py
    TARGET kabot_native_open_loop
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_CONTROL_MODE=open_loop
    TIMEOUT 45
  )
  ament_add_pytest_test(kabot_view_robot test/test_view_robot_launch.py)
  add_launch_test(test/test_mock_launch.py
    TARGET kabot_mock
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_PREFIX=
    TIMEOUT 60
  )
  add_launch_test(test/test_mock_launch.py
    TARGET kabot_mock_prefixed
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_PREFIX=kabot_
    TIMEOUT 60
  )
  add_launch_test(test/test_gazebo_launch.py
    TARGET kabot_gazebo
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_PREFIX=
    TIMEOUT 150
  )
  add_launch_test(test/test_gazebo_launch.py
    TARGET kabot_gazebo_prefixed
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_PREFIX=kabot_
    TIMEOUT 150
  )
  add_launch_test(test/test_gazebo_launch.py
    TARGET kabot_gazebo_foreign_description
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_PREFIX= KABOT_TEST_FOREIGN_DESCRIPTION=1
    TIMEOUT 150
  )
endif()

ament_package()
