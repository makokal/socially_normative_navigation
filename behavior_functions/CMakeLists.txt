cmake_minimum_required(VERSION 2.8.3)
project(behavior_functions)

add_definitions(-Wall -Wunused -std=c++0x -pipe)

set(PACKAGE_DEPENDENCIES
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    behavior_utils
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    INCLUDE_DIRS include
)



###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


#############
## Install ##
#############


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    # add_subdirectory(test)
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_behavior_functions.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
