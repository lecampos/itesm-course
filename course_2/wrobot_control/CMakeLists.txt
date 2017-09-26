cmake_minimum_required(VERSION 2.8.3)
project(wrobot_control)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS 
  wrobot_comm
  roscpp)

include_directories(
	include
	 ${catkin_INCLUDE_DIRS} )

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

add_executable(
  wrobot_simple_controller
  src/rosNode.cpp
  src/controller.cpp
)
target_link_libraries(wrobot_simple_controller ${catkin_LIBRARIES})

install(TARGETS wrobot_simple_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

