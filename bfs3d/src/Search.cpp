#include <bfs3d/BFS_3D.h>

namespace sbpl_arm_planner {

#define EXPAND_NEIGHBOR(offset)                            \
    if (distance_grid[currentNode + offset] < 0) {         \
        queue[queue_tail++] = currentNode + offset;        \
        distance_grid[currentNode + offset] = currentCost; \
    }

void BFS_3D::search(
    int width,
    int planeSize,
    int volatile* distance_grid,
    int* queue,
    int& queue_head,
    int& queue_tail)
{
    while (queue_head < queue_tail) {
        int currentNode = queue[queue_head++];
        int currentCost = distance_grid[currentNode] + 1;

        EXPAND_NEIGHBOR(-width);
        EXPAND_NEIGHBOR(1);
        EXPAND_NEIGHBOR(width);
        EXPAND_NEIGHBOR(-1);
        EXPAND_NEIGHBOR(-width-1);
        EXPAND_NEIGHBOR(-width+1);
        EXPAND_NEIGHBOR(width+1);
        EXPAND_NEIGHBOR(width-1);
        EXPAND_NEIGHBOR(planeSize);
        EXPAND_NEIGHBOR(-width+planeSize);
        EXPAND_NEIGHBOR(1+planeSize);
        EXPAND_NEIGHBOR(width+planeSize);
        EXPAND_NEIGHBOR(-1+planeSize);
        EXPAND_NEIGHBOR(-width-1+planeSize);
        EXPAND_NEIGHBOR(-width+1+planeSize);
        EXPAND_NEIGHBOR(width+1+planeSize);
        EXPAND_NEIGHBOR(width-1+planeSize);
        EXPAND_NEIGHBOR(-planeSize);
        EXPAND_NEIGHBOR(-width-planeSize);
        EXPAND_NEIGHBOR(1-planeSize);
        EXPAND_NEIGHBOR(width-planeSize);
        EXPAND_NEIGHBOR(-1-planeSize);
        EXPAND_NEIGHBOR(-width-1-planeSize);
        EXPAND_NEIGHBOR(-width+1-planeSize);
        EXPAND_NEIGHBOR(width+1-planeSize);
        EXPAND_NEIGHBOR(width-1-planeSize);
    }
    m_running = false;
}

#undef EXPAND_NEIGHBOR

#define EXPAND_NEIGHBOR_FRONTIER(offset) \
{\
    if (distance_grid[currentNode + offset] < 0) {\
        queue[queue_tail++] = currentNode + offset;\
        distance_grid[currentNode + offset] = currentCost;\
    }\
    else if (distance_grid[currentNode + offset] == WALL) {\
        if (frontier_grid[currentNode + offset] < 0) {\
            frontier_queue[frontier_queue_tail++] = currentNode + offset;\
            frontier_grid[currentNode + offset] = currentCost;\
        }\
    }\
}

void BFS_3D::search(
    int width,
    int planeSize,
    int volatile* distance_grid,
    int* queue,
    int& queue_head,
    int& queue_tail,
    int volatile* frontier_grid,
    int* frontier_queue,
    int& frontier_queue_head,
    int& frontier_queue_tail)
{
    while (queue_head < queue_tail) {
        int currentNode = queue[queue_head++];
        int currentCost = distance_grid[currentNode] + 1;

        EXPAND_NEIGHBOR_FRONTIER(-width);
        EXPAND_NEIGHBOR_FRONTIER(1);
        EXPAND_NEIGHBOR_FRONTIER(width);
        EXPAND_NEIGHBOR_FRONTIER(-1);
        EXPAND_NEIGHBOR_FRONTIER(-width-1);
        EXPAND_NEIGHBOR_FRONTIER(-width+1);
        EXPAND_NEIGHBOR_FRONTIER(width+1);
        EXPAND_NEIGHBOR_FRONTIER(width-1);
        EXPAND_NEIGHBOR_FRONTIER(planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width-1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width+1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width+1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width-1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width-1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width+1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width+1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width-1-planeSize);
    }
    m_running = false;
}

#undef EXPAND_NEIGHBOR_FRONTIER

} // namespace sbpl_arm_planner
