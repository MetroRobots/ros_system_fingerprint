cmake_minimum_required(VERSION 3.0.2)
project(system_fingerprint)

find_package(catkin REQUIRED COMPONENTS roslib rosnode rospy rosservice)
catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS roslib rosnode rospy rosservice
)

include_directories()

catkin_install_python(PROGRAMS src/imprint
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
