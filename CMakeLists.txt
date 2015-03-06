cmake_minimum_required(VERSION 2.8.3)
project(semap)

# Package also depends on:
#python-sqlalchemy
#geoalchemy2

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  tf
  geometry_msgs
  shape_msgs
  visualization_msgs
  semap_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  rospy
  roslib
  tf
  geometry_msgs
  shape_msgs
  visualization_msgs
  semap_msgs
  DEPENDS
  python-sqlalchemy
  geoalchemy2
)

catkin_python_setup()

catkin_install_python(PROGRAMS

  src/test/test_imports.py
  src/test/test_db_interaction.py
  src/test/test_transformation_tree.py

DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


