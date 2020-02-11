cmake_minimum_required(VERSION 2.8.3)
project(robot_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  urdf
#  tf
#  std_msgs
#  nav_msgs
#  sensor_msgs
)


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
#  INCLUDE_DIRS include
#  LIBRARIES robot1_description
#  CATKIN_DEPENDS roscpp std_msgs urdf xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
#add_executable(planner_node           src/planner_node.cpp      )
#add_executable(basic_planner_node     src/basic_planner_node.cpp)
#add_executable(mapper_node     src/mapper_node.cpp)
#add_executable(common_mapper_node     src/common_mapper_node.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot1_description_node robot1_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(planner_node          ${catkin_LIBRARIES})
#target_link_libraries(basic_planner_node    ${catkin_LIBRARIES})
#target_link_libraries(mapper_node    ${catkin_LIBRARIES})
#target_link_libraries(common_mapper_node    ${catkin_LIBRARIES})
