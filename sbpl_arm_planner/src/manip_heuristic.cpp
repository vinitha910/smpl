#include <sbpl_arm_planner/manip_heuristic.h>

namespace sbpl {
namespace manip {

ManipHeuristic::ManipHeuristic(
    ManipLattice* env,
    const OccupancyGridConstPtr& grid,
    const PlanningParams* params)
:
    Heuristic(env),
    m_manip_env(env),
    m_grid(grid),
    m_params(params)
{
}

bool ManipHeuristic::setGoal(int x, int y, int z)
{
    return true;
}

bool ManipHeuristic::setGoal(double x, double y, double z)
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);
    return setGoal(gx, gy, gz);
}

} // namespace manip
} // namespace sbpl
