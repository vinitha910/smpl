#ifndef sbpl_manip_manip_heuristic_h
#define sbpl_manip_manip_heuristic_h

// system includes
#include <sbpl/heuristics/heuristic.h>

// project includes
#include <sbpl_arm_planner/environment_robarm3d.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_arm_planner/planning_params.h>

namespace sbpl {
namespace manip {

class ManipHeuristic : public Heuristic
{
public:

    ManipHeuristic(
        EnvironmentROBARM3D* manip_env,
        const OccupancyGridConstPtr& grid,
        const PlanningParams* params);

    virtual ~ManipHeuristic() { }

    virtual bool setGoal(int x, int y, int z);
    virtual bool setGoal(double x, double y, double z);

    virtual double getMetricGoalDistance(double x, double y, double z) = 0;

protected:

    EnvironmentROBARM3D* m_manip_env;
    OccupancyGridConstPtr m_grid;
    const PlanningParams* m_params;
};

} // namespace manip
} // namespace sbpl

#endif
