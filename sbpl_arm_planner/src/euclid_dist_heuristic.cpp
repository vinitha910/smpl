#include <sbpl_arm_planner/euclid_dist_heuristic.h>

// standard includes
#include <math.h>

namespace sbpl {
namespace manip {

static inline
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2) const
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

EuclidDistHeuristic::EuclidDistHeuristic(
    EnvironmentROBARM3D* manip_env,
    const OccupancyGridConstPtr& grid,
    const PlanningParams* params)
:
    ManipHeuristic(manip_env, grid, params)
{
}

double EuclidDistHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    const std::vector<double>& goal_pose = m_manip_env->getGoal();
    return EuclideanDistance(x, y, z, goal_pose[0], goal_pose[1], goal_pose[2]);
}

int EuclidDistHeuristic::GetGoalHeuristic(int state_id)
{
    if (m_manip_env->isGoal(state_id)) {
        return 0;
    }

    const std::vector<double>& goal_pose = m_manip_env->getGoal();
    EnvROBARM3DDHashEntry_t* state = m_manip_env->getHashEntry(from_id);
    double x, y, z;
    m_grid->gridToWorld(state->xyz[0], state->xyz[1], state->xyz[2], x, y, z);
    state->heur = 500 * m_params->cost_per_meter_ * EuclideanDistance(
            x, y, z, goal_pose[0], goal_pose[1], goal_pose[2]);
    return state->heur;
}

int EuclidDistHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int EuclidDistHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    EnvROBARM3DHashEntry_t* from_entry = m_manip_env->getHashEntry(from_id);
    EnvROBARM3DHashEntry_t* to_entry = m_manip_env->getHashEntry(to_id);

    double fx, fy, fz, tx, ty, tz;
    m_grid->gridToWorld(
            from_entry->xyz[0], from_entry->xyz[1], from_entry->xyz[2],
            fx, fy, fz);
    m_grid->gridToWorld(
            to_entry->xyz[0], to_entry->xyz[1], to_entry->xyz[2],
            tx, ty, tz);
    from_entry->heur =
            500 * m_params->cost_per_meter_ * EuclideanDistance(fx, fy, fz, tx, ty, tz) / m_grid->getResolution();
    return from_entry->heur;
}

} // namespace manip
} // namespace sbpl
