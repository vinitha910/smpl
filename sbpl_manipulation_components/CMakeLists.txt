cmake_minimum_required(VERSION 2.8.3)
project(sbpl_manipulation_components)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(catkin REQUIRED
             COMPONENTS
                 angles
                 orocos_kdl
                 kdl_parser
                 leatherman
                 moveit_core
                 moveit_msgs
                 moveit_ros_planning
                 roscpp
                 sbpl_geometry_utils
                 urdf
                 visualization_msgs)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(include)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS
                   sbpl_geometry_utils
                   moveit_core
                   moveit_msgs
                   moveit_ros_planning
                   visualization_msgs
                   roscpp
                   angles
               DEPENDS Eigen)

add_library(${PROJECT_NAME}
            src/robot_model.cpp
            src/kdl_robot_model.cpp
            src/occupancy_grid.cpp
            src/collision_checker.cpp
            src/post_processing.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_kdl src/test_kdl_robot_model.cpp)
target_link_libraries(test_kdl ${PROJECT_NAME})
