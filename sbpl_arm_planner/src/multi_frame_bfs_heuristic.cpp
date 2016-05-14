#include <sbpl_arm_planner/multi_frame_bfs_heuristic.h>

namespace sbpl {
namespace manip {

MultiFrameBfsHeuristic::MultiFrameBfsHeuristic(
    EnvironmentROBARM3D* env,
    const OccupancyGridConstPtr& grid,
    const PlanningParams* params)
:
    ManipHeuristic(env, grid, params)
{
    syncGridAndBfs();
}

bool MultiFrameBfsHeuristic::setGoal(int x, int y, int z)
{
    ROS_INFO("Setting the BFS heuristic goal (%d, %d, %d)", x, y, z);
    if (!m_bfs->inBounds(x, y, z) || !m_ee_bfs->inBounds(x, y, z)) {
        ROS_ERROR("Heuristic goal is out of BFS bounds");
        return false;
    }

    m_bfs->run(x, y, z);
    m_ee_bfs->run(x, y, z);
}

bool MultiFrameBfsHeuristic::setGoal(double x, double y, double z)
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);
    return setGoal(gx, gy, gz);
}

double MultiFrameBfsHeuristic::getMetricDistance(double x, double y, double z)
{
    return 0.0;
}

int MultiFrameBfsHeuristic::GetGoalHeuristic(int state_id)
{
    const EnvROBARM3DHashEntry_t* state = m_robarm_env->getHashEntry(state_id);
    if (state) {
        std::vector<double> pose;
        if (!m_robot->computePlanningLinkFK(state, pose)) {
            ROS_ERROR("Failed to compute FK for planning link");
            return INT_MAX;
        }

        int eexyz[3];
        m_grid->worldToGrid(pose[0], pose[1], pose[2], eexyz[0], eexyz[1], eexyz[2]);
        const int ee_heur = getBfsCostToGoal(*m_ee_bfs, eexyz[0], eexyz[1], eexyz[2]);

        state->heur = getBfsCostToGoal(*m_bfs, state->xyz[0], state->xyz[1], state->xyz[2]);
        state->heur += ee_heur;
    }
    else {
        return 0;
    }
}

int MultiFrameBfsHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int MultiFrameBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

visualization_msgs::MarkerArray MultiFrameBfsHeuristic::getWallsVisualization() const
{
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray MultiFrameBfsHeuristic::getValuesVisualization() const
{
    return visualization_msgs::MarkerArray();
}

void MultiFrameBfsHeuristic::syncGridAndBfs()
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
                        m_params.planning_link_sphere_radius_)
                {
                    m_bfs->setWall(x, y, z);
                    m_ee_bfs->setWall(x, y, z);
                    ++wall_count;
                }
            }
        }
    }

    ROS_INFO("%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
    m_env->visualizationPublisher().publish(getWallsVisualization());
}

int MultiFrameBfsHeuristic::getBfsCostToGoal(
    const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return INT_MAX;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return INT_MAX;
    }
    else {
        return m_params->cost_per_cell_ * bfs.getDistance(x, y, z);
    }
}

} // namespace manip
} // namespace sbpl
