cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(rvo)


find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  std_msgs
  genmsg
)

catkin_package(
   CATKIN_DEPENDS mavros roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(RVO2-3D/src)
add_executable(rvo RvoNode.cpp
)
target_link_libraries(rvo RVO ${catkin_LIBRARIES})
add_dependencies(rvo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
