cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(wrobot_gazebo)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp std_msgs)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    roscpp
    std_msgs
    wrobot_comm
)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(wrobot_plugin SHARED resource/wrobot_plugin.cc)
target_link_libraries(wrobot_plugin ${GAZEBO_libraries} ${catkin_LIBRARIES})
add_dependencies(wrobot_plugin ${catkin_EXPORTED_TARGETS})

install(TARGETS wrobot_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

