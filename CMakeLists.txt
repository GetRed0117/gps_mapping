cmake_minimum_required(VERSION 2.8.3)
project(gps_mapping)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_filters
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  gl8_msgs
  tf)

add_message_files(  
  FILES  
  GPTRA_MSG.msg
  Heading.msg 
) 

generate_messages(DEPENDENCIES 
std_msgs
sensor_msgs)


#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS}
#	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime message_filters
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)




add_executable(scanRegistration2 src/scanRegistration.cpp)
target_link_libraries(scanRegistration2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(laserOdometry src/laserOdometry.cpp)
# target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(gpsMapping src/gpsMapping.cpp)
target_link_libraries(gpsMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(transformMaintenance src/transformMaintenance.cpp)
# target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(keyboardCtrl src/keyboardCtrl.cpp)
target_link_libraries(keyboardCtrl ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # TODO: Download test data
  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
    MD5 3d5194e6981975588b7a93caebf79ba4)
  add_custom_target(${PROJECT_NAME}_test_data
    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
  configure_file(tests/loam.test.in
                 ${PROJECT_BINARY_DIR}/test/loam.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
    DEPENDENCIES
      ${PROJECT_NAME}_test_data
      scanRegistration
      laserOdometry
      laserMapping
      transformMaintenance)
endif()


