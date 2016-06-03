#ifndef sbpl_manip_euclid_dist_heuristic_h
#define sbpl_manip_euclid_dist_heuristic_h

#include <sbpl_arm_planner/manip_heuristic.h>

namespace sbpl {
namespace manip {

class EuclidDistHeuristic : public ManipHeuristic
{
public:

    EuclidDistHeuristic(
        ManipHeuristic* manip_env
        const OccupancyGridConstPtr& grid,
        const PlanningParams* params);

    virtual ~EuclidDistHeuristic() { }

    double getMetricGoalDistance(double x, double y, double z);

    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id);
};

} // namespace manip
} // namespace sbpl

#endif
