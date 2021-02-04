cmake_minimum_required(VERSION 3.0.2)
project(multi_signals_annotator)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/multi_signals_annotator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
