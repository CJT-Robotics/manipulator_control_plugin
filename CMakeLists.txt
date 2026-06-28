cmake_minimum_required(VERSION 3.0.2)
project(manipulator_control_plugin)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
)

catkin_python_setup()

catkin_package()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch res
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)