cmake_minimum_required(VERSION 3.0.2)
project(Sony_FDR-X3000)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES Sony_FDR-X3000
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  script/sony_stream.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)