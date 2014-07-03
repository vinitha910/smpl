add_library(bfs3d src/BFS_3D.cpp src/Search.cpp)
target_link_libraries(bfs3d ${catkin_LIBRARIES})

install(
    TARGETS bfs3d
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
