cmake_minimum_required(VERSION 2.8.3)
project(unr_manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  moveit_core
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
  CATKIN_DEPENDS
    moveit_core
  # DPENDS
)

# determine which rosdistro is being used
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
############################ Build Main Library API ############################
################################################################################
add_library(grasp_server_library
  src/grasp_server_library/grasp_server.cc
)

target_link_libraries(grasp_server_library
  ${catkin_LIBRARIES}
)
################################################################################
################################################################################