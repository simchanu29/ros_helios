cmake_minimum_required(VERSION 2.8.3)
project(leddartech)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  PointCloudConversion.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(leddartech src/leddartech.cpp)
add_executable(periodic_snapshotter src/periodic_snapshotter.cpp)
add_executable(point_cloud_conversion src/point_cloud_conversion.cpp)
target_link_libraries(leddartech ${catkin_LIBRARIES} LeddarTech Leddar LeddarC)
target_link_libraries(periodic_snapshotter ${catkin_LIBRARIES})
target_link_libraries(point_cloud_conversion ${catkin_LIBRARIES})



