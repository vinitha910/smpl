cmake_minimum_required(VERSION 2.8.3)
project(sbpl_arm_planner_test)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED
             COMPONENTS
                 angles
                 geometry_msgs
                 orocos_kdl
                 leatherman
                 moveit_msgs
                 moveit_ros_planning
                 pviz
                 roscpp
                 sbpl_arm_planner
                 sbpl_geometry_utils
                 sbpl_collision_checking)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package()

add_executable(callPlanner src/call_planner.cpp)
set_target_properties(callPlanner PROPERTIES COMPILE_FLAGS "-std=c++0x")
target_link_libraries(callPlanner ${catkin_LIBRARIES})

install(TARGETS callPlanner
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

