cmake_minimum_required(VERSION 3.0.2)
project(interactive_marker_controller)

find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    roscpp
    visualization_msgs
    tf
    pcl_conversions
    pcl_msgs
    pcl_ros
    rospy
    sensor_msgs
    std_msgs
    roslib
    pointcloud_ros_utilities
    geometry_msgs
    )
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
)

###########
## Build ##
###########
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_GXX_FLAGS "-Wall -Wextra -Wconversion  -pedantic -std=gnu11" -fopenmp)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV REQUIRED)
find_package( CGAL QUIET COMPONENTS  )
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(prototype src/simple_marker.cpp)
# target_link_libraries(prototype
#    ${catkin_LIBRARIES}
# )

add_executable(gui_controller src/gui_controller.cpp)
target_link_libraries(gui_controller 
   ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
)

