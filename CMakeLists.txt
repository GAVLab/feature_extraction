cmake_minimum_required(VERSION 2.8.3)
project(feature_extraction)

set(catkin_components pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS ${catkin_components})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES feature_extraction
  CATKIN_DEPENDS ${catkin_components}
  # DEPENDS Eigen3
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(feature_extraction
  src/feature_extraction_node.cpp
  include/feature_extraction/feature_extraction_node.h
)

# add_executable(narf_node src/narf_node.cpp)
# target_link_libraries(narf_node ${catkin_LIBRARIES})

# add_executable(sift_node src/sift_node.cpp)
# target_link_libraries(sift_node ${catkin_LIBRARIES})

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES})

add_executable(segmentation_node src/segmentation_node.cpp)
target_link_libraries(segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)


add_executable(pcl_tune src/pcl_tune.cpp)
target_link_libraries(pcl_tune ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so )

add_executable(feature_extraction_node src/feature_extraction_node.cpp include/${PROJECT_NAME}/feature_extraction_node.h)
target_link_libraries(feature_extraction_node ${catkin_LIBRARIES})

# add_executable(narf_node src/narf_node.cpp)
# target_link_libraries(narf_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_pcl_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)