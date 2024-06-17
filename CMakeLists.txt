cmake_minimum_required(VERSION 3.8)
project(icp_matching_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(icp_matching SHARED
  src/icp_matching.cpp
)
add_dependencies(icp_matching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable (test_pcd_creator src/demo/test_pcd_creator.cpp)
target_link_libraries(test_pcd_creator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(demo src/demo/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${PCL_LIBRARIES} icp_matching)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
