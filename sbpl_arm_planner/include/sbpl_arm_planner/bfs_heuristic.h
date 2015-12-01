#ifndef sbpl_arm_planner_BfsHeuristic_h
#define sbpl_arm_planner_BfsHeuristic_h

#include <memory>

#include <moveit/distance_field/propagation_distance_field.h>
#include <sbpl/heuristics/heuristic.h>

#include <sbpl_arm_planner/environment_robarm3d.h>
#include <sbpl_manipulation_components/occupancy_grid.h>

#include <visualization_msgs/MarkerArray.h>

namespace sbpl_arm_planner {

class BfsHeuristic : public Heuristic
{
public:

    BfsHeuristic(
        EnvironmentROBARM3D* env,
        distance_field::PropagationDistanceField* df,
        double radius);

    bool setGoal(int x, int y, int z);
    bool setGoal(double x, double y, double z);

    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id);

    visualization_msgs::MarkerArray getWallsVisualization() const;
    visualization_msgs::MarkerArray getValuesVisualization() const;

private:

    double m_radius;
    sbpl_arm_planner::OccupancyGrid m_grid;
    std::unique_ptr<BFS_3D> m_bfs;
    EnvironmentROBARM3D* m_robarm_env; // alias of specific environment
};

} // namespace sbpl_arm_planner

#endif
