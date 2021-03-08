cmake_minimum_required(VERSION 3.0)
project(mitre_fast_layered_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
    tf2_msgs
    tf2_sensor_msgs
    tf2_geometry_msgs
    tf2_eigen
    tf2_kdl
    grid_map_core
    grid_map_ros
    grid_map_msgs
    grid_map_loader
    grid_map_rviz_plugin
    grid_map_visualization
    sensor_msgs
    geometry_msgs
    filters
    pcl_ros
    pcl_conversions
    cv_bridge
    rostest
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED)
# It found eigen on its own for awhile and then stopped for some reason
set(CMAKE_PREFIX_PATH "/usr/lib/cmake/eigen3/Eigen3Config.cmake")
find_package(Eigen3 REQUIRED)

#set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mitre_fast_layered_map
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(filter_plugins
   src/filters/outlier_removal.cpp
   src/filters/ray_trace_2d.cpp
   src/filters/inflation.cpp
   src/filters/bayes_update.cpp
   src/filters/threshold_filter.cpp
)

add_library(${PROJECT_NAME}
  src/sensor_map.cpp
  src/test_map.cpp
  src/static_map.cpp
)

target_link_libraries(${PROJECT_NAME} filter_plugins ${catkin_LIBRARIES})

### Sensor Map Node
add_executable(sensor_map_node src/sensor_map_node.cpp)

### Static Map node
add_executable(static_map_node src/static_map_node)

### Specify libraries to link a library or executable target against
target_link_libraries(sensor_map_node
  ${PROJECT_NAME}
)

target_link_libraries(static_map_node
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
install(TARGETS 
  sensor_map_node 
  static_map_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME} filter_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS src/utils/publish_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES filter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  include_directories(
    test/include
  )

  add_rostest_gtest(test_filter_outlier_removal 
    test/launch/outlier_removal.launch 
    test/test_filter_outlier_removal.cpp
    test/test_mitre_fast_layered_map.cpp)
  target_link_libraries(test_filter_outlier_removal ${catkin_LIBRARIES})

  add_rostest_gtest(test_filter_ray_trace
    test/launch/ray_trace.launch
    test/test_filter_ray_trace.cpp
    test/test_mitre_fast_layered_map.cpp)
  target_link_libraries(test_filter_ray_trace ${catkin_LIBRARIES})

  add_rostest_gtest(test_filter_inflation
    test/launch/inflation.launch
    test/test_filter_inflation.cpp
    test/test_mitre_fast_layered_map.cpp)
  target_link_libraries(test_filter_inflation ${catkin_LIBRARIES})

  add_rostest_gtest(test_filter_bayes_update
    test/launch/bayes_update.launch
    test/test_filter_bayes_update.cpp
    test/test_mitre_fast_layered_map.cpp)
  target_link_libraries(test_filter_bayes_update ${catkin_LIBRARIES})

  add_rostest_gtest(test_filter_threshold
    test/launch/threshold.launch
    test/test_filter_threshold.cpp
    test/test_mitre_fast_layered_map.cpp)
  target_link_libraries(test_filter_threshold ${catkin_LIBRARIES})

  add_rostest_gtest(test_map 
    test/launch/map.launch 
    test/test_map.cpp 
    test/test_mitre_fast_layered_map.cpp
    test/init_vars.cpp)
  target_link_libraries(test_map ${PROJECT_NAME} filter_plugins ${catkin_LIBRARIES})

  #catkin_add_gtest(${PROJECT_NAME}-test 
  #  test/test_dart_gridmap.cpp
  #  test/test_filter_inflation.cpp
  #  test/test_filter_outlier_removal.cpp
  #  test/test_filter_ray_trace.cpp
  #  test/test_local_map.cpp
  #  test/test_map_controller.cpp
  #)

  #if(TARGET ${PROJECT_NAME}-test)
  # target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  #endif()
endif()

#target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

#target_link_libraries(${PROJECT_NAME}-test filter_plugins)

#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} filter_plugins)
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
