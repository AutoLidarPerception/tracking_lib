##
# Copyright (C) 2019 by AutoSense Organization. All rights reserved.
# Gary Chan <chenshj35@mail2.sysu.edu.cn>
##

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS core
    roscpp
    # ROS messages
    std_msgs
    sensor_msgs
    geometry_msgs
    # ROS PCL
    #pcl_conversions
    #pcl_ros
    # 3rd modules for ROS, like Eigen
    cmake_modules
    # local packages
    common_lib
    segmenters_lib
    # needed for detection_node
    roi_filters_lib
    tracking_lib
    feature_extractors_lib
)


## System dependencies are found with CMake's conventions
### PCL
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

###########
## Build ##
###########


### Specify additional locations of header files
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

### Declare a C++ executable
add_executable(${PROJECT_NAME}_detection_node
    detection_node.cpp
)
add_executable(${PROJECT_NAME}_tracking_node
    tracking_node.cpp
)

### Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detection_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tracking_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# ...