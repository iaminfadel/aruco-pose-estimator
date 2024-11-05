cmake_minimum_required(VERSION 3.0.2)
project(aruco_pose_estimator)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_pose_processor
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs
    visualization_msgs
    cv_bridge 
    image_transport
    camera_info_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(aruco_pose_processor src/aruco_pose_processor.cpp)
target_link_libraries(aruco_pose_processor 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

add_executable(aruco_pose_node src/aruco_pose_node.cpp)
target_link_libraries(aruco_pose_node 
  aruco_pose_processor 
  ${catkin_LIBRARIES}
)