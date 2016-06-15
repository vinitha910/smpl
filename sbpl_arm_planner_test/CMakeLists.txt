cmake_minimum_required(VERSION 2.8)
project(sbpl_arm_planner_test)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(Boost REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        angles
        cmake_modules
        geometry_msgs
        leatherman
        moveit_msgs
        moveit_ros_planning
        orocos_kdl
        roscpp
        sbpl_arm_planner
        sbpl_collision_checking
        sbpl_geometry_utils
        sbpl_kdl_robot_model
        sbpl_pr2_robot_model
        visualization_msgs)

find_package(Eigen REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(callPlanner src/call_planner.cpp)
target_link_libraries(callPlanner ${catkin_LIBRARIES})

install(
    TARGETS callPlanner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

