#include <sbpl_arm_planner/soft_bfs_heuristic.h>

namespace sbpl {
namespace manip {

SoftBfsHeuristic::SoftBfsHeuristic(
    EnvironmentROBARM3D* env,
    const OccupancyGridConstPtr& grid,
    const PlanningParams* params)
:
    ManipHeuristic(env, grid, params)
{
    syncGridAndBfs();
}

bool SoftBfsHeuristic::setGoal(int x, int y, int z)
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);

    if (!m_bfs->escapeCell(x, y, z)) {
        ROS_ERROR("BFS goal is out of bounds or couldn't be freed");
        return false;
    }

    m_bfs->run_components(x, y, z);
    ROS_INFO(" -> Running");
    while (m_bfs->isRunning());
    ROS_INFO(" -> Finished: %d walls, %d undiscovered, %d discovered", m_bfs->countWalls(), m_bfs->countUndiscovered(), m_bfs->countDiscovered());

    return true;
}

bool SoftBfsHeuristic::setGoal(double x, double y, double z)
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);
    return setGoal(gx, gy, gz);
}

int SoftBfsHeuristic::GetGoalHeuristic(int state_id)
{
    const EnvROBARM3DHashEntry_t* state = m_robarm_env->getHashEntry(state_id);
    if (state) {
        const int x = state->xyz[0];
        const int y = state->xyz[1];
        const int z = state->xyz[2];
        if (m_bfs_>isWall(x, y, z)) {
            const int h = 100 * m_bfs->getNearestFreeNodeDist(x, y, z);
            return INT_MAX;
        }
        else {
            int cell_dist = m_bfs->getDistance(x, y, z);
            if (cell_dist < 0) {
                ROS_WARN_THROTTLE(1, "queried distance for isolated cell");
                m_bfs->setWall(x, y, z);
                while ((cell_dist = m_bfs->getNearestFreeNodeDist(x, y, z)) < 0);
                return 100 * cell_dist;
            }
            else {
                return 100 * m_bfs->getDistance(x, y, z);
            }
        }
    }
    else {
        return 0;
    }
}

int SoftBfsHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int SoftBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

visualization_msgs::MarkerArray SoftBfsHeuristic::getWallsVisualization() const
{

}

visualization_msgs::MarkerArray SoftBfsHeuristic::getValuesVisualization() const
{

}

void SoftBfsHeuristic::syncGridAndBfs()
{
    int xc, yc, zc;
    m_grid->getGridSize(xc, yc, zc);
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int z = 0; z < zc; ++z) {
        for (int y = 0; y < yc; ++y) {
            for (int x = 0; x < xc; ++x) {
                if (m_grid->getDistance(x, y, z) <=
                        m_params->planning_link_sphere_radius_)
                {
                    m_bfs->setWall(x, y, z);
                    ++wall_count;
                }
            }
        }
    }

    ROS_INFO("%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

} // namespace manip
} // namespace sbpl
