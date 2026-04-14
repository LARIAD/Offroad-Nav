cmake_minimum_required(VERSION 3.0.2)
project(axis_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED 
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-rtsp-server-1.0
)

catkin_python_setup()

catkin_package(
  LIBRARIES opencv_gst
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_gst_cpp src/stream_to_topic.cpp)

add_dependencies(${PROJECT_NAME}_gst_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_gst_cpp
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GST_LIBRARIES}
)

catkin_install_python(PROGRAMS
  nodes/axis_camera_node
  nodes/axis_teleop_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
