#include <sbpl_arm_planner/bfs_heuristic.h>

namespace sbpl_arm_planner {

BfsHeuristic::BfsHeuristic(
    EnvironmentROBARM3D* env,
    distance_field::PropagationDistanceField* df,
    double radius)
:
    Heuristic(env),
    m_grid(df),
    m_radius(radius)
{
    int xc, yc, zc;
    m_grid.getGridSize(xc, yc, zc);
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    for (int x = 0; x < xc; ++x) {
        for (int y = 0; y < yc; ++y) {
            for (int z = 0; z < zc; ++z) {
                if (m_grid.getDistance(x, y, z) <= m_radius) {
                    m_bfs->setWall(x, y, z);
                }
            }
        }
    }
}

bool BfsHeuristic::setGoal(int x, int y, int z)
{
    int xc, yc, zc;
    m_bfs->getDimensions(&xc, &yc, &zc);

    if (x < 0 || y < 0 || z < 0 || x >= xc || y >= yc || z >= zc) {
        return false;
    }

    m_bfs->run(x, y, z);
    return true;
}

int BfsHeuristic::GetGoalHeuristic(int state_id)
{
    return 0;
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
