cmake_minimum_required(VERSION 2.8.3)
project(unr_manipulation)

include(cmake/CPlusPlusCompiler.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  moveit_core
)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
# add_service_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

# add_action_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES
  CATKIN_DEPENDS
    moveit_core
  # DPENDS
)

# cmake options
option(LOGGING_ENABLED "Enable Log info printing" OFF)
if(${LOGGING_ENABLED})
  set(LOG_INFO_PRINTING ON)
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  set(DEBUG ON)
endif()


# Setup configuration files
include(cmake/ManipulationGenHeaders.cmake)

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

add_executable(grasp_server_test
  src/grasp_server_library/grasp_server_test.cc
)

target_link_libraries(grasp_server_test
  grasp_server_library
  ${GTEST_LIBRARIES}
  pthread
)

enable_testing()
add_test(
  NAME server_test
  COMMAND grasp_server_test
)