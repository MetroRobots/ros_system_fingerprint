cmake_minimum_required(VERSION 3.10.2)
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


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/basic.test)
endif()
