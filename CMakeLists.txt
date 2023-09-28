cmake_minimum_required(VERSION 3.0.2)
project(lidar_point_to_panorama)

# Compile as C++17
add_definitions(-std=c++17)

# Find necessary packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge 
  image_transport 
  pcl_conversions 
  pcl_ros roscpp 
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/${PROJECT_NAME}/
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Create the panorama_node executable
add_executable(panorama_node nodes/panorama_node.cpp src/PointCloudToPanorama.cpp src/pointfilter.cpp src/rotatePointAroundZ.cpp)
target_link_libraries(panorama_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Create the filter_node executable
add_executable(filter_node nodes/filter_node.cpp src/pointfilter.cpp)
target_link_libraries(filter_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Create the rotate_node executable
add_executable(rotate_node nodes/rotate_node.cpp src/rotatePointAroundZ.cpp)
target_link_libraries(rotate_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Create the convert_node executable
add_executable(convert_node nodes/convert_node.cpp src/PointCloudToPanorama.cpp)
target_link_libraries(convert_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
