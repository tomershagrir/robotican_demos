cmake_minimum_required(VERSION 2.8.3)
project(robotican_demos)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        robotican_common
        roscpp
        rospy
        std_msgs
        move_base_msgs
        actionlib
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface

        cv_bridge
        pcl_conversions
        pcl_ros
        tf
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface

        pr2_controllers_msgs
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(robotican_common
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configrobotican_commonuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES robotican_demos
        CATKIN_DEPENDS roscpp rospy robotican_common std_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

#add_definitions(-DDEBUG_REACH_EVENT)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

add_executable(moveit_group_goal src/moveit_group_goal.cpp)
target_link_libraries(moveit_group_goal ${catkin_LIBRARIES})

add_executable(pan_tilt_api src/pan_tilt_api.cpp)
target_link_libraries(pan_tilt_api ${catkin_LIBRARIES})

add_executable(gripper_api src/gripper_api.cpp)
target_link_libraries(gripper_api ${catkin_LIBRARIES})

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES})

add_executable(drive_and_pick src/drive_and_pick.cpp)
add_dependencies(drive_and_pick ${robotican_common_EXPORTED_TARGETS})
target_link_libraries(drive_and_pick ${catkin_LIBRARIES})

add_executable(point_head src/point_head.cpp)
target_link_libraries(point_head ${catkin_LIBRARIES})

add_executable(arm_jta_client src/arm_jta_client.cpp)
target_link_libraries(arm_jta_client ${catkin_LIBRARIES})

add_executable(arm_trajectory_demo src/arm_trajectory_demo.cpp)
target_link_libraries(arm_trajectory_demo ${catkin_LIBRARIES})

add_executable(find_objects src/find_objects.cpp)
target_link_libraries(find_objects ${catkin_LIBRARIES})







#add_dependencies( demo_pick_node robotican_common_generate_messages_cpp )
## Declare a C++ library
# add_library(robotican_demos
#   src/${PROJECT_NAME}/robotican_demos.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(robotican_demos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(robotican_demos_node src/robotican_demos_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(robotican_demos_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(robotican_demos_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS robotican_demos robotican_demos_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robotican_demos.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
