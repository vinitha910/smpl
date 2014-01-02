cmake_minimum_required(VERSION 2.8.3)
project(bfs3d)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(catkin REQUIRED roscpp)
include_directories(${catkin_INCLUDE_DIRS})

include_directories(include)
add_definitions(-DROS)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME} src/BFS_3D.cpp src/Search.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
