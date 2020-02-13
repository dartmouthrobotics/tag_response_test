cmake_minimum_required(VERSION 2.8.3)
project(tag_response_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  image_transport
  cv_bridge
  nodelet
  ar_track_alvar_msgs
)
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES tag_response_test 
  CATKIN_DEPENDS roscpp nodelet std_msgs image_transport ar_track_alvar_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/fake_video_stream_nodelet.cpp
  src/marker_subscriber_nodelet.cpp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_NAME}.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
