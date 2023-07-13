cmake_minimum_required(VERSION 3.10)
project(conversion)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  sensor_msgs
  dvs_msgs
  image_meta_msgs)

find_package(HDF5 REQUIRED COMPONENTS CXX)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS})

set(HEADERS
  include/h5_io/H5UtilityFunctions.h
  include/h5_io/h5_to_ros/H5ToRosMessageWriter.h
  include/h5_io/h5_to_ros/H5ToRosImageWriter.h
  include/h5_io/h5_to_ros/H5ToRosEventsWriter.h
  include/h5_io/h5_to_ros/H5ToRosImageMetaWriter.h
  include/h5_io/h5_to_ros/H5ToRosCameraDiagnosticsWriter.h
  include/h5_io/h5_to_ros/H5ToRosConfigurationWriter.h
  include/h5_io/h5_to_ros/H5ToRosPoseStampedWriter.h
  include/h5_io/ros_to_h5/RosMessageToH5Writer.h
  include/h5_io/ros_to_h5/RosImageToH5Writer.h
  include/h5_io/ros_to_h5/RosEventsToH5Writer.h
  include/h5_io/ros_to_h5/RosImageMetaToH5Writer.h
  include/h5_io/ros_to_h5/RosCameraDiagnosticsToH5Writer.h
  include/h5_io/ros_to_h5/RosConfigurationToH5Writer.h
  include/h5_io/ros_to_h5/RosPoseStampedToH5Writer.h
  include/h5_io/ros_to_h5/compression/CompressionMethod.h
  include/h5_io/ros_to_h5/compression/GzipCompressionMethod.h)

set(SOURCES
  src/h5_io/H5UtilityFunctions.cpp
  src/h5_io/h5_to_ros/H5ToRosMessageWriter.cpp
  src/h5_io/h5_to_ros/H5ToRosImageWriter.cpp
  src/h5_io/h5_to_ros/H5ToRosEventsWriter.cpp
  src/h5_io/h5_to_ros/H5ToRosImageMetaWriter.cpp
  src/h5_io/h5_to_ros/H5ToRosCameraDiagnosticsWriter.cpp
  src/h5_io/h5_to_ros/H5ToRosConfigurationWriter.cpp
  src/h5_io/h5_to_ros/H5ToRosPoseStampedWriter.cpp
  src/h5_io/ros_to_h5/RosMessageToH5Writer.cpp
  src/h5_io/ros_to_h5/RosImageToH5Writer.cpp
  src/h5_io/ros_to_h5/RosEventsToH5Writer.cpp
  src/h5_io/ros_to_h5/RosImageMetaToH5Writer.cpp
  src/h5_io/ros_to_h5/RosCameraDiagnosticsToH5Writer.cpp
  src/h5_io/ros_to_h5/RosConfigurationToH5Writer.cpp
  src/h5_io/ros_to_h5/RosPoseStampedToH5Writer.cpp
  src/h5_io/ros_to_h5/compression/GzipCompressionMethod.cpp)

catkin_package(
  CATKIN_DEPENDS rosbag roscpp rospy sensor_msgs dvs_msgs image_meta_msgs
  INCLUDE_DIRS include)

add_library(${PROJECT_NAME}_LIB ${SOURCES} ${HEADERS})

add_executable(rosbag_to_h5 src/RosbagToH5.cpp)
target_link_libraries(rosbag_to_h5 ${PROJECT_NAME}_LIB ${catkin_LIBRARIES} ${HDF5_LIBRARIES})

add_executable(h5_combine src/H5Combine.cpp)
target_link_libraries(h5_combine ${PROJECT_NAME}_LIB ${catkin_LIBRARIES} ${HDF5_LIBRARIES})

add_executable(h5_to_rosbag src/H5ToRosbag.cpp)
target_link_libraries(h5_to_rosbag ${PROJECT_NAME}_LIB ${catkin_LIBRARIES} ${HDF5_LIBRARIES})
