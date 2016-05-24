cmake_minimum_required(VERSION 2.8.3)
project(unr_manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

# add_service_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

# add_action_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES
  # CATKIN_DEPENDS
  # DPENDS
)

# determine which rosdistro is being used
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
)

