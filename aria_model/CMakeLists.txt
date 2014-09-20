project(aria_model)

cmake_minimum_required(VERSION 2.4.6)

catkin_package()

#execute_process(COMMAND sh ${PROJECT_SOURCE_DIR}/gen-aria.sh ${PROJECT_SOURCE_DIR})
##set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
 )

add_executable(state_publisher src/state_publisher.cc)

target_link_libraries(state_publisher
  ${catkin_LIBRARIES}
)

install(TARGETS state_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
