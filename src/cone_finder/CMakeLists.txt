cmake_minimum_required(VERSION 3.0.2)
project(cone_finder)

find_package(catkin REQUIRED COMPONENTS
	pcl_ros nodelet roscpp
)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES nodelet_test
   CATKIN_DEPENDS nodelet roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(cluster_extractor src/cluster_extractor.cpp)
add_dependencies(cluster_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cluster_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  cluster_extractor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/cone_finder.py
  scripts/cone_list_viz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


