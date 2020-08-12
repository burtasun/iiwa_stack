cmake_minimum_required(VERSION 3.5)
project(iiwa_hwextaxis)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ximc)

find_package(catkin REQUIRED COMPONENTS
  combined_robot_hw
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  iiwa_msgs
  iiwa_ros
  iiwa_hw
  pluginlib
  controller_manager_msgs
  hardware_interface
  roscpp
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
  CATKIN_DEPENDS
      combined_robot_hw
      control_toolbox
      controller_interface
      controller_manager
      hardware_interface
      realtime_tools
      joint_limits_interface
      iiwa_msgs
      iiwa_ros
      iiwa_hw
      pluginlib
      controller_manager_msgs
      hardware_interface
      roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/home/ros-industrial/Desktop/ximc-2.12.1/ximc" #eje externo
  "/home/ros-industrial/Desktop/ximc-2.12.1/ximc/macosx"
)

add_library(${PROJECT_NAME}
    ../iiwa_hw/src/iiwa_hw.cpp
    src/extaxis_hw.cpp
)
target_link_libraries(
  ${PROJECT_NAME} ${catkin_LIBRARIES}
  "/usr/lib/libjximc.so"
  "/usr/lib/libbindy.so"
  "/usr/lib/libximc.so.7"
  "/usr/lib/libjximc.so.7"
)



add_executable(${PROJECT_NAME}-bin 
  src/main_extaxis.cpp
)
target_link_libraries(
  ${PROJECT_NAME}-bin ${catkin_LIBRARIES} 
  "/usr/lib/libjximc.so"
  "/usr/lib/libbindy.so"
  "/usr/lib/libximc.so.7"
  "/usr/lib/libjximc.so.7"
)


#target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})

#linkado api eje externo
SET(EXTAXIS_FLAG  "-lximc")
add_definitions(${EXTAXIS_FLAG})
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lximc")