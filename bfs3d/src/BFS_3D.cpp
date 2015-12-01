#include <bfs3d/BFS_3D.h>

namespace sbpl_arm_planner {

BFS_3D::BFS_3D(int width, int height, int length) :
    m_search_thread(),
    m_dim_x(),
    m_dim_y(),
    m_dim_z(),
    m_distance_grid(nullptr),
    m_queue(nullptr),
    m_queue_head(),
    m_queue_tail(),
    m_running(false),
    m_neighbor_offsets(),
    m_closed(),
    m_distances()
{
    if (width <= 0 || height <= 0 || length <= 0) {
        //error "Invalid dimensions"
        return;
    }

    m_dim_x = width + 2;
    m_dim_y = height + 2;
    m_dim_z = length + 2;

    m_dim_xy = m_dim_x * m_dim_y;
    m_dim_xyz = m_dim_xy * m_dim_z;

    m_neighbor_offsets[0] = -m_dim_x;
    m_neighbor_offsets[1] = 1;
    m_neighbor_offsets[2] = m_dim_x;
    m_neighbor_offsets[3] = -1;
    m_neighbor_offsets[4] = -m_dim_x-1;
    m_neighbor_offsets[5] = -m_dim_x+1;
    m_neighbor_offsets[6] = m_dim_x+1;
    m_neighbor_offsets[7] = m_dim_x-1;
    m_neighbor_offsets[8] = m_dim_xy;
    m_neighbor_offsets[9] = -m_dim_x+m_dim_xy;
    m_neighbor_offsets[10] = 1+m_dim_xy;
    m_neighbor_offsets[11] = m_dim_x+m_dim_xy;
    m_neighbor_offsets[12] = -1+m_dim_xy;
    m_neighbor_offsets[13] = -m_dim_x-1+m_dim_xy;
    m_neighbor_offsets[14] = -m_dim_x+1+m_dim_xy;
    m_neighbor_offsets[15] = m_dim_x+1+m_dim_xy;
    m_neighbor_offsets[16] = m_dim_x-1+m_dim_xy;
    m_neighbor_offsets[17] = -m_dim_xy;
    m_neighbor_offsets[18] = -m_dim_x-m_dim_xy;
    m_neighbor_offsets[19] = 1-m_dim_xy;
    m_neighbor_offsets[20] = m_dim_x-m_dim_xy;
    m_neighbor_offsets[21] = -1-m_dim_xy;
    m_neighbor_offsets[22] = -m_dim_x-1-m_dim_xy;
    m_neighbor_offsets[23] = -m_dim_x+1-m_dim_xy;
    m_neighbor_offsets[24] = m_dim_x+1-m_dim_xy;
    m_neighbor_offsets[25] = m_dim_x-1-m_dim_xy;

    m_distance_grid = new int[m_dim_xyz];
    m_queue = new int[width * height * length];

    for (int node = 0; node < m_dim_xyz; node++) {
        int x = node % m_dim_x;
        int y = node / m_dim_x % m_dim_y;
        int z = node / m_dim_xy;
        if (x == 0 || x == m_dim_x - 1 ||
            y == 0 || y == m_dim_y - 1 ||
            z == 0 || z == m_dim_z - 1)
        {
            m_distance_grid[node] = WALL;
        }
        else {
            m_distance_grid[node] = UNDISCOVERED;
        }
    }

    m_running = false;
}

BFS_3D::~BFS_3D()
{
    m_search_thread.join();

    delete[] m_distance_grid;
    delete[] m_queue;
}

void BFS_3D::getDimensions(int* width, int* height, int* length)
{
	*width = m_dim_x - 2;
	*height = m_dim_y - 2;
	*length = m_dim_z - 2;
}

void BFS_3D::setWall(int x, int y, int z)
{
    if (m_running) {
        //error "Cannot modify grid while search is running"
        return;
    }

    int node = getNode(x, y, z);
    m_distance_grid[node] = WALL;
}

bool BFS_3D::isWall(int x, int y, int z)
{
    int node = getNode(x, y, z);
    return m_distance_grid[node] == WALL;
}

void BFS_3D::run(int x, int y, int z)
{
    if (m_running) {
        //error "Search already running"
        return;
    }

    for (int i = 0; i < m_dim_xyz; i++) {
        if (m_distance_grid[i] != WALL) {
            m_distance_grid[i] = UNDISCOVERED;
        }
    }

    // get index of start coordinate
    int origin = getNode(x, y, z);

    // initialize the queue
    m_queue_head = 0;
    m_queue_tail = 1;
    m_queue[0] = origin;

    // initialize starting distance
    m_distance_grid[origin] = 0;

    // fire off background thread to compute bfs
    m_search_thread =
            boost::thread(&BFS_3D::search, this, m_dim_x, m_dim_xy, m_distance_grid, m_queue, m_queue_head, m_queue_tail);
    m_running = true;
}

void BFS_3D::run_components(int gx, int gy, int gz)
{
    for (int i = 0; i < m_dim_xyz; i++) {
        if (m_distance_grid[i] != WALL) {
            m_distance_grid[i] = UNDISCOVERED;
        }
    }

    // compute connected components
    std::vector<int> components(m_dim_xyz, -1);
    int comp_count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (isWall(i)) {
            continue;
        }
        if (components[i] == -1) {
            auto floodfiller = [&](int node) {
                return components[node] = comp_count;
            };
            visit_free_cells(i, floodfiller);
            comp_count++;
        }
    }

    // 1. for each component
    // 2.     find the nearest cell to the goal and its distance
    // 3.     floodfill distances from nearest cell, adding distance of the
    //                nearest cell to the goal
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (isWall(i)) {
            continue;
        }

        if (m_distance_grid[i] < 0) { // undiscovered
            int nearest_dist = -1;
            int nearest;
            auto find_nearest_to_goal = [&](int node) {
                int nx, ny, nz;
                getCoord(node, nx, ny, nz);

                const int dist_sq =
                        (nx - gx) * (nx - gx) +
                        (ny - gy) * (ny - gy) +
                        (nz - gz) * (nz - gz);

                if (nearest_dist == -1) {
                    nearest = node;
                    nearest_dist = dist_sq;
                }
                else if (dist_sq < nearest_dist) {
                    nearest = node;
                    nearest_dist = dist_sq;
                }
            };
            visit_free_cells(i, find_nearest_to_goal);

            int origin = nearest;
            m_queue_head = 0;
            m_queue_tail = 1;
            m_queue[0] = origin;
            m_distance_grid[origin] = (int)sqrt((double)nearest_dist);
            search(m_dim_x, m_dim_xy, m_distance_grid, m_queue, m_queue_head, m_queue_tail);
        }
    }
}

template <typename Visitor>
void BFS_3D::visit_free_cells(int node, const Visitor& visitor)
{
    if (isWall(node)) {
        return;
    }

    int nx, ny, nz;
    getCoord(node, nx, ny, nz);

    std::vector<bool> visited(m_dim_xyz, false);
    std::queue<int> nodes;
    nodes.push(node);

    while (!nodes.empty()) {
        int n = nodes.front();
        nodes.pop();

        visitor(n);

        for (int i = 0; i < 26; ++i) {
            int nn = neighbor(n, i);
            if (!visited[nn] && !isWall(nn)) {
                nodes.push(nn);
                // mark visited here to avoid adding nodes to the queue multiple
                // times
                visited[nn] = true;
                if (nodes.size() >= m_dim_xyz) {
                    ROS_ERROR("Wow queue is too damn big");
                    return;
                }
            }
        }
    }
}

int BFS_3D::getDistance(int x, int y, int z)
{
    int node = getNode(x, y, z);
    while (m_running && m_distance_grid[node] < 0);
    return m_distance_grid[node];
}

int BFS_3D::getNearestFreeNodeDist(int x, int y, int z)
{
//    ROS_INFO("Get Nearest Free Node Dist (%d, %d, %d)", x, y, z);

    // initialize closed set and distances
    m_closed.assign(m_dim_xyz, false);
    m_distances.assign(m_dim_xyz, -1);

    std::queue<std::tuple<int, int, int>> q;
    q.push(std::make_tuple(x, y, z));

    int n = getNode(x, y, z);
    m_distances[n] = 0;

    while (!q.empty()) {
        std::tuple<int, int, int> ncoords = q.front();
        q.pop();

        // extract the coordinates of this cell
        int nx = std::get<0>(ncoords);
        int ny = std::get<1>(ncoords);
        int nz = std::get<2>(ncoords);

        // extract the index of this cell
        n = getNode(nx, ny, nz);

//        ROS_INFO("Expanding %d (%zu in OPEN)", n, q.size());

        // mark as visited
        m_closed[n] = true;

        int dist = m_distances[n];

        // goal == found a free cell
        if (!isWall(n)) {
            int cell_dist = getDistance(nx, ny, nz);
            if (cell_dist < 0) {
                // TODO: mark as a wall, and move on
                setWall(nx, ny, nz);
                ROS_INFO("Encountered isolated cell, m_running: %s", m_running ? "true" : "false");
            }
            else {
//                ROS_INFO("Returning distance of %d", dist + cell_dist);
                return dist + cell_dist;
            }
        }


#define ADD_NEIGHBOR(xn, yn, zn) \
{\
if (inBounds(xn, yn, zn)) {\
    int nn = getNode(xn, yn, zn);\
    if (!m_closed[nn] && (m_distances[nn] == -1 || dist + 1 < m_distances[nn])) {\
        m_distances[nn] = dist + 1;\
        q.push(std::make_tuple(xn, yn, zn));\
    }\
}\
}

        ADD_NEIGHBOR(nx - 1, ny - 1, nz - 1);
        ADD_NEIGHBOR(nx - 1, ny - 1, nz    );
        ADD_NEIGHBOR(nx - 1, ny - 1, nz + 1);
        ADD_NEIGHBOR(nx - 1, ny,     nz - 1);
        ADD_NEIGHBOR(nx - 1, ny,     nz    );
        ADD_NEIGHBOR(nx - 1, ny,     nz + 1);
        ADD_NEIGHBOR(nx - 1, ny + 1, nz - 1);
        ADD_NEIGHBOR(nx - 1, ny + 1, nz    );
        ADD_NEIGHBOR(nx - 1, ny + 1, nz + 1);
        ADD_NEIGHBOR(nx    , ny - 1, nz - 1);
        ADD_NEIGHBOR(nx    , ny - 1, nz    );
        ADD_NEIGHBOR(nx    , ny - 1, nz + 1);
        ADD_NEIGHBOR(nx    , ny,     nz - 1);
//            ADD_NEIGHBOR(nx    , ny,     nz    );
        ADD_NEIGHBOR(nx    , ny,     nz + 1);
        ADD_NEIGHBOR(nx    , ny + 1, nz - 1);
        ADD_NEIGHBOR(nx    , ny + 1, nz    );
        ADD_NEIGHBOR(nx    , ny + 1, nz + 1);
        ADD_NEIGHBOR(nx + 1, ny - 1, nz - 1);
        ADD_NEIGHBOR(nx + 1, ny - 1, nz    );
        ADD_NEIGHBOR(nx + 1, ny - 1, nz + 1);
        ADD_NEIGHBOR(nx + 1, ny,     nz - 1);
        ADD_NEIGHBOR(nx + 1, ny,     nz    );
        ADD_NEIGHBOR(nx + 1, ny,     nz + 1);
        ADD_NEIGHBOR(nx + 1, ny + 1, nz - 1);
        ADD_NEIGHBOR(nx + 1, ny + 1, nz    );
        ADD_NEIGHBOR(nx + 1, ny + 1, nz + 1);
#undef ADD_NEIGHBOR
    }

    fprintf(stderr, "Found no free neighbor\n");
    return -1;
}

int BFS_3D::countWalls() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] == WALL) {
            ++count;
        }
    }
    return count;
}

int BFS_3D::countUndiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] == UNDISCOVERED) {
            ++count;
        }
    }
    return count;
}

int BFS_3D::countDiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] != WALL && m_distance_grid[i] >= 0) {
            ++count;
        }
    }
    return count;
}


} // namespace sbpl_arm_planner
