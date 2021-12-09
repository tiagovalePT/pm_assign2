cmake_minimum_required(VERSION 2.8.3)
project(object_3d_estimation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  gazebo_msgs
  tf
  cv_bridge
  image_transport
  message_generation
  std_msgs
  pcl_ros
  pcl_conversions
  darknet_ros
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


#add_message_files(
#    FILES
#    visual_markers.msg
#)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)


catkin_package(
    INCLUDE_DIRS include
#    LIBRARIES msg2tf
    CATKIN_DEPENDS nav_msgs sensor_msgs tf gazebo_msgs opencv message_generation std_msgs pcl_ros pcl_conversions darknet_ros
#    DEPENDS
)

###########
## Build ##
###########




## Declare a C++ executable
add_executable(object_3d_estimation src/object_3d_estimation.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(object_3d_estimation PROPERTIES OUTPUT_NAME object_3d_estimation PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(object_3d_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(object_3d_estimation
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

### Declare a C++ executable
#add_executable(object_visualization src/object_visualization.cpp)

### Rename C++ executable without prefix
### e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(object_visualization PROPERTIES OUTPUT_NAME object_visualization PREFIX "")

### Add cmake target dependencies of the executable
#add_dependencies(object_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

### Specify libraries to link a library or executable target against
#target_link_libraries(object_visualization
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS gazebo2odom_node lin_vel_control_node lidar_nearest
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
