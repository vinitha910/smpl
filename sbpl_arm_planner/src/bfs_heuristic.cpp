#include <sbpl_arm_planner/bfs_heuristic.h>

#include <iostream>
#include <queue>

namespace sbpl_arm_planner {

BfsHeuristic::BfsHeuristic(
    EnvironmentROBARM3D* env,
    distance_field::PropagationDistanceField* df,
    double radius)
:
    Heuristic(env),
    m_grid(df),
    m_radius(radius),
    m_robarm_env(env)
{
    int xc, yc, zc;
    m_grid.getGridSize(xc, yc, zc);
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int x = 0; x < xc; ++x) {
        for (int y = 0; y < yc; ++y) {
            for (int z = 0; z < zc; ++z) {
                if (m_grid.getDistance(x, y, z) <= m_radius) {
                    m_bfs->setWall(x, y, z);
                    ++wall_count;
                }
            }
        }
    }

    ROS_INFO("%d/%d (%0.3f %%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

bool BfsHeuristic::setGoal(int x, int y, int z)
{
    ROS_INFO("Setting the BFS Heuristic goal (%d, %d, %d)", x, y, z);

    if (!m_bfs->inBounds(x, y, z)) {
        ROS_ERROR("BFS goal is out of bounds");
        return false;
    }

    // clear cells until the goal connects to a free cell
    int escape_count = 0;
    std::queue<int> q;
    q.push(m_bfs->getNode(x, y, z));
    bool escaped = false;

    int length, width, height;
    m_bfs->getDimensions(&length, &width, &height);
    std::vector<bool> visited((length + 2) * (width + 2) * (height + 2), false);

    while (!q.empty()) {
        int n = q.front();
        q.pop();

        visited[n] = true;

        // goal condition
        if (!m_bfs->isWall(n)) {
            break;
        }

        m_bfs->unsetWall(n);

        for (int i = 0; i < 26; ++i) {
            int neighbor = m_bfs->neighbor(n, i);
            if (!visited[neighbor]) {
                q.push(neighbor);
            }
        }

        ++escape_count;
    }

    ROS_INFO("Escaped goal cell in %d expansions", escape_count);

    m_bfs->run_components(x, y, z);
    ROS_INFO(" -> Running");
    while (m_bfs->isRunning());
    ROS_INFO(" -> Finished: %d walls, %d undiscovered, %d discovered", m_bfs->countWalls(), m_bfs->countUndiscovered(), m_bfs->countDiscovered());

    return true;
}

bool BfsHeuristic::setGoal(double x, double y, double z)
{
    int gx, gy, gz;
    m_grid.worldToGrid(x, y, z, gx, gy, gz);
    return setGoal(gx, gy, gz);
}

int BfsHeuristic::GetGoalHeuristic(int state_id)
{
    const EnvROBARM3DHashEntry_t* state = m_robarm_env->getHashEntry(state_id);
    if (state) {
        // TODO: some copypasta from environment here :(
        const int x = state->xyz[0];
        const int y = state->xyz[1];
        const int z = state->xyz[2];
        if (m_bfs->isWall(x, y, z)) {
            const int h = m_bfs->getNearestFreeNodeDist(x, y, z) * 100;
//            ROS_WARN_THROTTLE(1, "Queried heuristic for wall cell (%d)", h);
            return INT_MAX;
        }

        int cell_dist = m_bfs->getDistance(x, y, z);
        if (cell_dist < 0) {
            ROS_WARN_THROTTLE(1, "queried distance for isolated cell");
            m_bfs->setWall(x, y, z);
            while ((cell_dist = m_bfs->getNearestFreeNodeDist(x, y, z)) < 0);
            return cell_dist * 100;
//            return INT_MAX;
        }
        else {
            const int h = int(m_bfs->getDistance(x, y, z)) * 100 /*prm_->cost_per_cell_*/;
//            ROS_INFO_THROTTLE(1, "h(%d, %d, %d) = %d", x, y, z, h);
            return h;
        }
    }
    else {
        return 0;
    }
}

int BfsHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int BfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

visualization_msgs::MarkerArray BfsHeuristic::getWallsVisualization() const
{

}

visualization_msgs::MarkerArray BfsHeuristic::getValuesVisualization() const
{

}

} // namespace sbpl_arm_planner
