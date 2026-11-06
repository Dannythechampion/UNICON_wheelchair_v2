cmake_minimum_required(VERSION 3.0.2)
project(pid)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  diagnostic_msgs
  std_srvs
  tf
  rosgraph
  static_livox_localization
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    diagnostic_msgs
    std_srvs
    tf
    rosgraph
    static_livox_localization
)

catkin_install_python(
  PROGRAMS
    scripts/ff_pid_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    scripts/ff_pid_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)