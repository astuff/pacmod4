cmake_minimum_required(VERSION 2.8.3)
project(pacmod4)

add_definitions(-std=c++14)

# For static library
add_compile_options(-fPIC)

set(catkin_deps
  nodelet
  roscpp
  std_msgs
  pacmod4_msgs
  can_msgs
)

find_package(catkin REQUIRED ${catkin_deps} roslint)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${catkin_deps}
  LIBRARIES ${PROJECT_NAME}_nodelet
)

###########
## Build ##
###########

include_directories(
  include
  pacmod4_common/include
  ${catkin_INCLUDE_DIRS}
)

# Common Library
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_library(pacmod4_common STATIC
  pacmod4_common/src/pacmod4_dbc_ros_api.cpp
  pacmod4_common/src/pacmod4_dbc13_ros_api.cpp
  pacmod4_common/src/autogen/pacmod13.c
)

# nodelets
add_library(${PROJECT_NAME}_nodelet
  src/pacmod4_nodelet.cpp
  src/pacmod4_ros_msg_handler.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  pacmod4_common
  ${catkin_LIBRARIES}
)

# nodes
add_executable(${PROJECT_NAME}_node
  src/pacmod4_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# TODO: Re-enable after auto-formatting
# set(ROSLINT_CPP_OPTS "--filter=-build/include_subdir")
# roslint_cpp(
#   src/pacmod4_node.cpp
#   src/pacmod4_nodelet.cpp
#   src/pacmod4_ros_msg_handler.cpp
#   include/pacmod4/pacmod3_nodelet.h
#   include/pacmod4/pacmod3_ros_msg_handler.h
#   )

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
