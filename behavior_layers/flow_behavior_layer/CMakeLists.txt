cmake_minimum_required(VERSION 2.8.3)
project(flow_behavior_layer)

add_definitions(-Wall -Wunused -std=c++0x -pipe) # C++ 11 is required

set(PACKAGE_DEPENDENCIES
    costmap_2d
    pluginlib
    roscpp
    rospy
    spencer_tracking_msgs
    geometry_msgs
    tf
    behavior_functions
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# generate dynamic reconfigure options
generate_dynamic_reconfigure_options(cfg/FlowBehaviorLayer.cfg)


catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})


## Declare a cpp library
add_library(${PROJECT_NAME} src/flow_behavior_layer.cpp)
add_dependencies(${PROJECT_NAME} spencer_tracking_msgs)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS flow_behavior_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

