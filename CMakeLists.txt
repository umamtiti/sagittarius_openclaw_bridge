cmake_minimum_required(VERSION 2.8.3)
project(sagittarius_openclaw_bridge)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cv_bridge
  message_generation
  rospy
  sensor_msgs
  sagittarius_object_color_detector
)

add_service_files(
  FILES
  RunCommand.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    cv_bridge
    message_runtime
    rospy
    sensor_msgs
    sagittarius_object_color_detector
)

catkin_install_python(PROGRAMS
  scripts/command_bridge.py
  scripts/openclaw_cmd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
