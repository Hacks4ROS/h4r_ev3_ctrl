cmake_minimum_required(VERSION 2.8.3)
project(h4r_ev3_sensor_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
)


include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS controller_interface hardware_interface sensor_msgs realtime_tools
  INCLUDE_DIRS include
  )


add_library(ev3_ultrasonic_range_controller
  src/ev3_ultrasonic_range_controller.cpp include/h4r_ev3_sensor_control/ev3_ultrasonic_range_controller.h)
target_link_libraries(ev3_ultrasonic_range_controller ${catkin_LIBRARIES})



# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  
install(TARGETS ev3_ultrasonic_range_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ev3_sensor_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})