cmake_minimum_required(VERSION 3.0.2)
project(tf)



find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    tf2_ros
    roslib
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS

  CATKIN_DEPENDS geometry_msgs

  DEPENDS
)

set( SOURCE
        src/main.cpp
        )


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tf_node ${SOURCE})
target_link_libraries(tf_node ${catkin_LIBRARIES})




