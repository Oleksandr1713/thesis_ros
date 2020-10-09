cmake_minimum_required(VERSION 2.8.3)
project(simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR ~/Libraries/opencv-3.4.9/build)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        mongodb_store
        std_msgs
        message_generation
        grid_map_core
        grid_map_msgs
        grid_map_ros
        nav_msgs
        move_base_msgs
        actionlib
        tf
        OpenCV)
find_package(Qt5 COMPONENTS Quick Core REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   DatabaseEntryInsert.msg
   DatabaseEntryUpdate.msg
   DestinationPose.msg
   JobBriefInfo.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   AddObjectOnMapMsg.srv
   CleanCacheMsg.srv
   RemoveObjectFromMapMsg.srv
   ScheduleJobMsg.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

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
#  LIBRARIES simulation
  CATKIN_DEPENDS
        roscpp
        message_runtime
        mongodb_store
        grid_map_core
        grid_map_msgs
        grid_map_ros
        nav_msgs
        move_base_msgs
        actionlib
        tf
        qtbase5-dev
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/simulation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/simulation_node.cpp)
add_executable(sign_coordinates_node scripts/sign_coordinates_on_image.cpp)
add_executable(obstacle_dpc_node scripts/observation/obstacle_det_pos_calc.cpp)
add_executable(job_scheduler_node scripts/scheduler/job_scheduler.cpp)
add_executable(job_catcher_node scripts/catcher/job_catcher.cpp)
add_executable(database_node scripts/database/database.cpp)
add_executable(map_enrichment_node scripts/enrichment/map_enrichment.cpp)
add_executable(cmd_control_node scripts/control/cmd_control.cpp)

add_executable(cache scripts/observation/cache.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )
target_link_libraries(sign_coordinates_node ${catkin_LIBRARIES})
target_link_libraries(obstacle_dpc_node ${catkin_LIBRARIES} Qt5::Core)
target_link_libraries(job_scheduler_node ${catkin_LIBRARIES})
target_link_libraries(job_catcher_node ${catkin_LIBRARIES})
target_link_libraries(database_node ${catkin_LIBRARIES})
target_link_libraries(map_enrichment_node ${catkin_LIBRARIES})
target_link_libraries(cmd_control_node ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(sign_coordinates_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_dpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(job_scheduler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(job_catcher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(database_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(map_enrichment_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cmd_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})