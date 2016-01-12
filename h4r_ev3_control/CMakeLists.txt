cmake_minimum_required(VERSION 2.8.3)
project(h4r_ev3_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  h4r_ev3_msgs
)

add_definitions(-std=c++11)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS controller_interface hardware_interface sensor_msgs realtime_tools h4r_ev3_msgs
  INCLUDE_DIRS include
  LIBRARIES ev3_control
  )

add_library(ev3_control
    src/${PROJECT_NAME}/Ev3Strings.cpp
    src/${PROJECT_NAME}/H4REv3Port.cpp
    src/${PROJECT_NAME}/syshelpers.cpp
)
target_link_libraries(ev3_control ${catkin_LIBRARIES})

add_library(ev3_controllers
  src/${PROJECT_NAME}/ev3_ultrasonic_controller.cpp include/${PROJECT_NAME}/ev3_ultrasonic_controller.h
  src/${PROJECT_NAME}/ev3_gyro_controller.cpp       include/${PROJECT_NAME}/ev3_gyro_controller.h  
  src/${PROJECT_NAME}/ev3_touch_controller.cpp      include/${PROJECT_NAME}/ev3_touch_controller.h
  src/${PROJECT_NAME}/ev3_color_controller.cpp      include/${PROJECT_NAME}/ev3_color_controller.h  
  src/${PROJECT_NAME}/ev3_infrared_controller.cpp      include/${PROJECT_NAME}/ev3_infrared_controller.h  
)
target_link_libraries(ev3_controllers ${catkin_LIBRARIES})



# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  
install(TARGETS ev3_control ev3_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ev3_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})