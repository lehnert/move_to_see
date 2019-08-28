cmake_minimum_required(VERSION 2.8.3)
project(move_to_see)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    move_to_see_msgs
    rospy
    std_msgs
)


#add dynamic reconfigure api

#
# set(PCL_DIR "/usr/local/share/pcl-1.8")
# find_package(PCL 1.8 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   # LIBRARIES pcl_tools
   CATKIN_DEPENDS rospy std_msgs
#  CATKIN_DEPENDS baxter_core_msgs cv_bridge moveit_core roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(state_machine
#   src/${PROJECT_NAME}/state_machine.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(state_machine_node src/state_machine_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(state_machine_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(state_machine_node
#   ${catkin_LIBRARIES}
# )

execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

# include_directories(include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIR}
#   ${PCL_INCLUDE_DIRS}
#
# )

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executableharvey_msgs_generate_messages_cpp
# add_executable(point_cloud_repub src/point_cloud_repub.cpp)
#
# add_dependencies( point_cloud_repub pcl_tools)

## Specify libraries to link a library or executable target against
 # target_link_libraries(point_cloud_repub
 #   ${catkin_LIBRARIES}
 #   ${OpenCV_LIBS}
 #   ${PCL_LIBRARIES}
 #   ${pcl_tools_LIBRARIES}
 # )
