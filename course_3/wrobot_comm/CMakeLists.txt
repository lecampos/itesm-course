cmake_minimum_required(VERSION 2.8.3)
project(wrobot_comm)
find_package(catkin REQUIRED
  COMPONENTS geometry_msgs message_generation std_msgs)

# Declare the message files to be built
add_message_files(
  FILES
  WheelVelocity.msg
  RobotPosition.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
