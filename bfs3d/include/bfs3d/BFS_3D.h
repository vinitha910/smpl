#ifndef _BFS_3D_
#define _BFS_3D_

#include <boost/thread.hpp>

#define WALL         0x7FFFFFFF
#define UNDISCOVERED 0xFFFFFFFF

namespace sbpl_arm_planner {

class BFS_3D
{
public:
    
    BFS_3D(int length, int width, int height);
    ~BFS_3D();

    void getDimensions(int* length, int* width, int* height);

    void setWall(int x, int y, int z);
    bool isWall(int x, int y, int z);

    void run(int x, int y, int z);

    int getDistance(int x, int y, int z);

private:

    boost::thread m_search_thread;

    int dim_x, dim_y, dim_z;
    int dim_xy, dim_xyz;

    int origin;
    int volatile* distance_grid;

    int* queue;
    int queue_head, queue_tail;

    volatile bool running;

    void search(int, int, int volatile*, int*, int&, int&);
    inline int getNode(int, int, int);
};

} // namespace sbpl_arm_planner

#endif
