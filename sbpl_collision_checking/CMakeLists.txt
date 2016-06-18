cmake_minimum_required(VERSION 2.8)
project(sbpl_collision_checking)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(Boost REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        cmake_modules
        eigen_conversions
        geometric_shapes
        geometry_msgs
        kdl_parser
        leatherman
        message_filters
        moveit_core
        moveit_msgs
        pr2_controllers_msgs
        roscpp
        sbpl_arm_planner
        sbpl_geometry_utils
        sensor_msgs
        shape_msgs
        tf_conversions
        urdf
        visualization_msgs)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_collision_checking
    CATKIN_DEPENDS
        eigen_conversions
        geometric_shapes
        geometry_msgs
        kdl_parser
        leatherman
        message_filters
        moveit_core
        moveit_msgs
        pr2_controllers_msgs
        roscpp
        sbpl_arm_planner
        sbpl_geometry_utils
        sensor_msgs
        shape_msgs
        tf_conversions
        urdf
        visualization_msgs
    DEPENDS
        Boost
        Eigen
        orocos_kdl)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})
include_directories(include)

#add_library(
#    sbpl_collision_checking
#    src/group.cpp
#    src/sbpl_collision_model.cpp
#    src/collision_model_impl.cpp
#    src/collision_model_config.cpp
#    src/collision_world.cpp
#    src/sbpl_collision_space.cpp)
#target_link_libraries(
#    sbpl_collision_checking
#    ${catkin_LIBRARIES}
#    ${orocos_kdl_LIBRARIES})
#set_target_properties(
#    sbpl_collision_checking
#    PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations")

#add_executable(test_space src/test_collision_space.cpp)
#target_link_libraries(test_space ${catkin_LIBRARIES} sbpl_collision_checking)

add_executable(
    test_collision_model
    src/collision_model_impl.cpp
    src/collision_model_config.cpp
    src/sbpl_collision_model.cpp
    src/test_collision_model.cpp)
target_link_libraries(test_collision_model ${catkin_LIBRARIES})

#install(
#    TARGETS sbpl_collision_checking test_space
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

