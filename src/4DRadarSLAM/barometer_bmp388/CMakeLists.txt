cmake_minimum_required(VERSION 2.8.3)
project(barometer_bmp388)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  )


add_message_files(
  FILES
  Barometer.msg
)
#
generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(DEPENDS CATKIN DEPENDS roscpp rospy std_msgs message_runtime)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
	script/barometer.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
