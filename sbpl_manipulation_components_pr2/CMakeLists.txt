cmake_minimum_required(VERSION 2.8)
project(sbpl_manipulation_components_pr2)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin
    REQUIRED
    COMPONENTS
        angles
        pr2_arm_kinematics
        roscpp
        sbpl_geometry_utils
        urdf)

find_package(orocos_kdl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_manipulation_components_pr2
    CATKIN_DEPENDS
        angles
        pr2_arm_kinematics
        roscpp
        sbpl_geometry_utils
        urdf
    DEPENDS
        orocos_kdl)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})
include_directories(include)

add_library(
    sbpl_manipulation_components_pr2
    src/sbpl_math.cpp
    src/orientation_solver.cpp)
    # src/pr2_kdl_robot_model.cpp
    # src/ubr1_kdl_robot_model.cpp)

target_link_libraries(sbpl_manipulation_components_pr2 ${catkin_LIBRARIES})

install(
    TARGETS sbpl_manipulation_components_pr2
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
