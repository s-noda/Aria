cmake_minimum_required(VERSION 2.8.3)
project(aria_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES utils
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ../aria_2ndparty/src/common
  ../aria_2ndparty/src/sensors
  ../aria_2ndparty/src/actuators
  ../aria_2ndparty/src/utils
)

add_executable(interactive_pose_maker src/interactive_pose_maker.cc ../aria_2ndparty/src/common/ssb_common_of.cc ../aria_2ndparty/src/common/ssb_common_vec.cc ../aria_2ndparty/src/utils/ssb_utils_model.cc)

target_link_libraries(interactive_pose_maker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

