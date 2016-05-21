#ifndef sbpl_manip_soft_bfs_heuristic_h
#define sbpl_manip_soft_bfs_heuristic_h

#include <sbpl/heuristics/heuristic.h>

namespace sbpl {
namespace manip {

class SoftBfsHeuristic : public ManipHeuristic
{
public:

    SoftBfsHeuristic(
        ManipLattice* env,
        const OccupancyGridConstPtr& grid,
        const PlanningParams* params);

    bool setGoal(int x, int y, int z);
    bool setGoal(double x, double y, double z);

    visualization_msgs::MarkerArray getWallsVisualization() const;
    visualization_msgs::MarkerArray getValuesVisualization() const;

    double getMetricGoalDistance(double x, double y, double z);

    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id);

private:

    std::unique_ptr<BFS_3D> m_bfs;

    void syncGridAndBfs();
};

} // namespace manip
} // namespace sbpl

#endif
