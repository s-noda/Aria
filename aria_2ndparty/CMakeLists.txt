cmake_minimum_required(VERSION 2.8.3)
project(aria_2ndparty)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)


find_package(OpenCV)


################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ssb_tutorials
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  includes/kduino
  includes/dynamixel_msgs
  src/common
  src/sensors
  src/actuators
  src/utils
)

add_executable(second_party_controller src/second_party_controller.cc src/common/ssb_common_of.cc src/common/ssb_common_vec.cc src/utils/ssb_utils_model.cc src/utils/ssb_utils_interpolation.cc)
add_executable(second_party_core src/second_party_core.cc src/common/ssb_common_of.cc src/common/ssb_common_vec.cc src/utils/ssb_utils_model.cc src/utils/ssb_utils_interpolation.cc)


target_link_libraries(second_party_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(second_party_core
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

