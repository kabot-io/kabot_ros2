cmake_minimum_required(VERSION 3.16)
project(kabot_robot LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

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
  DESTINATION share/${PROJECT_NAME}/launch)
install(FILES bringup/config/kabot_mock_controllers.yaml bringup/config/kabot_gazebo_controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY bringup/worlds DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(kabot_urdf_xacro test/test_urdf_xacro.py
    ENV KABOT_TEST_INSTALLED=1
  )

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_view_robot_launch.py
    TARGET kabot_view_robot
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_PREFIX=
    TIMEOUT 45
  )
  add_launch_test(test/test_view_robot_launch.py
    TARGET kabot_view_robot_prefixed
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV KABOT_TEST_PREFIX=kabot_
    TIMEOUT 45
  )
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
