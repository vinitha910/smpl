#ifndef _BFS_3D_
#define _BFS_3D_

#include <stdio.h>
#include <queue>
#include <tuple>
#include <iostream>
#include <ros/ros.h>

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
    void run_components(int gx, int gy, int gz);

    int getDistance(int x, int y, int z);

    inline bool inBounds(int x, int y, int z) const
    {
        return !(x < 0 || y < 0 || z < 0 || x >= m_dim_x - 2 || y >= m_dim_y - 2 || z >= m_dim_z - 2);
    }

    inline int getNode(int x, int y, int z) const
    {
        if (!inBounds(x, y, z)) {
            return -1;
        }

        return (z + 1) * m_dim_xy + (y + 1) * m_dim_x + (x + 1);
    }

    inline bool getCoord(int node, int& x, int& y, int& z) const
    {
        if (node < 0 || node >= m_dim_xyz) {
            return false;
        }

        int zz = node / m_dim_xy;
        int yy = (node - zz * m_dim_xy) / m_dim_x;
        int xx = node - zz * m_dim_xy - yy * m_dim_x;
        z = zz - 1;
        y = yy - 1;
        x = xx - 1;
        return true;
    }

    int getNearestFreeNodeDist(int x, int y, int z);

    inline void setWall(int node) const
    {
        m_distance_grid[node] = WALL;
    }

    inline void unsetWall(int node) const
    {
        m_distance_grid[node] = UNDISCOVERED;
    }

    inline bool isWall(int node) const
    {
        return m_distance_grid[node] == WALL;
    }

    inline int isUndiscovered(int node) const
    {
        return m_distance_grid[node] < 0;
    }

    inline int neighbor(int node, int neighbor) const
    {
        return node + m_neighbor_offsets[neighbor];
    }

    bool isRunning() const { return m_running; }

    int countWalls() const;
    int countUndiscovered() const;
    int countDiscovered() const;

private:

    boost::thread m_search_thread;

    int m_dim_x, m_dim_y, m_dim_z;
    int m_dim_xy, m_dim_xyz;

    int volatile* m_distance_grid;

    int* m_queue;
    int m_queue_head, m_queue_tail;

    volatile bool m_running;

    int m_neighbor_offsets[26];
    std::vector<bool> m_closed;
    std::vector<int> m_distances;

    void search(
        int width,
        int planeSize,
        int volatile* distance_grid,
        int* queue,
        int& queue_head,
        int& queue_tail);

    void search(
        int width,
        int planeSize,
        int volatile* distance_grid,
        int* queue,
        int& queue_head,
        int& queue_tail,
        int volatile* frontier_grid,
        int* frontier_queue,
        int& frontier_queue_head,
        int& frontier_queue_tail);

    template <typename Visitor>
    void visit_free_cells(int node, const Visitor& visitor);
};

} // namespace sbpl_arm_planner

#endif
