cmake_minimum_required(VERSION 2.8.3)
project(floam)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL GTSAM
  INCLUDE_DIRS include
)

add_library(${PROJECT_NAME}
  src/laserMappingClass.cpp
  src/odomEstimationClass.cpp
  src/laserProcessingClass.cpp
  src/lidar.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

add_executable(floam_laser_processing_node src/laserProcessingNode.cpp)
target_link_libraries(floam_laser_processing_node ${PROJECT_NAME})

add_executable(floam_odom_estimation_node src/odomEstimationNode.cpp)
target_link_libraries(floam_odom_estimation_node  ${PROJECT_NAME})

add_executable(floam_laser_mapping_node src/laserMappingNode.cpp)
target_link_libraries(floam_laser_mapping_node  ${PROJECT_NAME})
