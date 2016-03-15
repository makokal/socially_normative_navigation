cmake_minimum_required(VERSION 2.8.3)

project(overtaking_planner)

add_definitions(-Wall -Wunused -std=c++0x -pipe)

set(PACKAGE_DEPENDENCIES
    geometry_msgs
    nav_msgs
    roscpp
    tf
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED)


catkin_package(
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    INCLUDE_DIRS include
)


###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(overtaking_planner
#   src/${PROJECT_NAME}/overtaking_planner.cpp
# )

## Declare a C++ executable
# add_executable(overtaking_planner_node src/overtaking_planner_node.cpp)


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

## Mark executables and/or libraries for installation
# install(TARGETS overtaking_planner overtaking_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_overtaking_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
