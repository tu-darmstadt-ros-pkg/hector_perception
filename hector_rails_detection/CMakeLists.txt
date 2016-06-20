cmake_minimum_required(VERSION 2.8.3)
project(hector_rails_detection)

if (NOT MSVC)
  # Enable the C++11 standard
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    octomap_msgs
    octomap_ros
    actionlib
    roscpp
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
    dynamic_reconfigure
    hector_perception_msgs
    grid_map_msgs
)

link_libraries(${OCTOMAP_LIBRARIES})

add_definitions(-DOCTOMAP_NODEBUGOUT)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS cv_bridge geometry_msgs octomap_msgs octomap_ros actionlib roscpp sensor_msgs std_msgs tf visualization_msgs hector_perception_msgs grid_map_msgs
   DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(${PROJECT_NAME}
   src/rails_detection.cpp
   src/rails_detection_node.cpp
   include/${PROJECT_NAME}/rails_detection.h
 )

add_dependencies(hector_rails_detection ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(hector_rails_detection_node src/rails_detection_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(hector_tutorial_node hector_tutorial_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(hector_rails_detection_node
   hector_rails_detection
   ${catkin_LIBRARIES}
   ${PCL_LIBRARY_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)


#common commands for building c++ executables and libraries
#add_executable(rails_detection src/rails_detection.cpp)
#target_link_libraries(rails_detection ${catkin_LIBRARIES})

