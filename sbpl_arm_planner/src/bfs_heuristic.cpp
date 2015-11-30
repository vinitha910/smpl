#include <sbpl_arm_planner/bfs_heuristic.h>

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
    ROS_INFO("Set the Bfs Heuristic goal");
    int xc, yc, zc;
    m_bfs->getDimensions(&xc, &yc, &zc);

    if (x < 0 || y < 0 || z < 0 || x >= xc || y >= yc || z >= zc) {
        return false;
    }

    m_bfs->run(x, y, z);
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
        int x = state->xyz[0];
        int y = state->xyz[1];
        int z = state->xyz[2];
        if (m_bfs->getDistance(x, y, z) > 1000000) {
            return INT_MAX;
        }
        else {
            const int h = int(m_bfs->getDistance(x, y, z)) * 100 /*prm_->cost_per_cell_*/;
            ROS_INFO_THROTTLE(1, "h(%d, %d, %d) = %d", x, y, z, h);
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

} // namespace sbpl_arm_planner
