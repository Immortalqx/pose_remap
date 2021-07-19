cmake_minimum_required(VERSION 2.8.4)
project(pose_remap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        mavros
        mavros_msgs
        roscpp
        std_msgs
        tf2
        tf2_ros
        ddynamic_reconfigure
        )

find_package(gflags)

catkin_package(
        CATKIN_DEPENDS geometry_msgs mavros mavros_msgs roscpp std_msgs tf2 tf2_ros ddynamic_reconfigure
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(remap_pose_node src/remap_pose_node.cpp src/Remap.cpp src/Remap.h)

target_link_libraries(remap_pose_node
        ${catkin_LIBRARIES}
        gflags
        )

