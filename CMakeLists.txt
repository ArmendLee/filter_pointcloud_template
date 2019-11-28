cmake_minimum_required(VERSION 2.8.3)
project(filtpointcloud)


add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roscpp
  sensor_msgs
)

catkin_package(

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(filtpointcloud src/filtpointcloud.cpp)

target_link_libraries(filtpointcloud
  ${catkin_LIBRARIES}
 )

install(TARGETS filtpointcloud 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



