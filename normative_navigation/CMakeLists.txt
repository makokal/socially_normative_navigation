cmake_minimum_required(VERSION 2.8.3)
project(normative_navigation)

set(PACKAGE_DEPENDENCIES
    move_base
    nav_msgs
    roscpp
    rospy
    pedsim_simulator
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

###########
## Build ##
###########

# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


#############
## Install ##
#############



#############
## Testing ##
#############

