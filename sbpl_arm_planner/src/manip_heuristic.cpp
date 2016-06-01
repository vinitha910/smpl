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

bool ManipHeuristic::setGoal(const GoalConstraint& goal)
{
    return true;
}

} // namespace manip
} // namespace sbpl
