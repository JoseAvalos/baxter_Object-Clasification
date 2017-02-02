cmake_minimum_required(VERSION 2.8.3)
project(baxter_obj_classification)

## Find catkin and any catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg visualization_msgs)

###########################################################################

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(OSIK_CONTROL REQUIRED osik-control)
PKG_CHECK_MODULES(RBDL REQUIRED rbdl)
PKG_CHECK_MODULES(QPOASES REQUIRED qpoases)


IF (RBDL_FOUND)
  SET(RBDL_URDFReader_LIBRARIES rbdl_urdfreader)
ENDIF (RBDL_FOUND)


## Find catkin macros and libraries
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospack
  roslib
  tf
  visualization_msgs
  geometry_msgs
  tf2_msgs
)

CATKIN_PACKAGE()

###########
## Build ##
###########

INCLUDE_DIRECTORIES(
  include
  ${RBDL_INCLUDE_DIRS}
  ${OSIK_CONTROL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${RBDL_LIBRARY_DIRS}
  ${OSIK_CONTROL_LIBRARY_DIRS}
  ${QPOASES_LIBRARY_DIRS}
)

ADD_LIBRARY(baxter_obj_classification SHARED
  src/markers.cpp
  src/tools.cpp
  )

SET(ALL_LIBRARIES
  baxter_obj_classification
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARIES}
  ${RBDL_URDFReader_LIBRARIES}
  ${OSIK_CONTROL_LIBRARIES}
  ${QPOASES_LIBRARIES}
)


add_executable(left_OSIK_baxter left_baxter.cpp)
TARGET_LINK_LIBRARIES( left_OSIK_baxter
${ALL_LIBRARIES}
)

add_executable(right_OSIK_baxter right_baxter.cpp)
TARGET_LINK_LIBRARIES( right_OSIK_baxter
${ALL_LIBRARIES}
)


