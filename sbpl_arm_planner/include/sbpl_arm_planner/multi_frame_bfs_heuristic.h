#ifndef sbpl_manip_multi_frame_bfs_heuristic_h
#define sbpl_manip_multi_frame_bfs_heuristic_h

// standard includes
#include <memory>

// system includes
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_arm_planner/manip_heuristic.h>

namespace sbpl {
namespace manip {

class MultiFrameBfsHeuristic : public ManipHeuristic
{
public:

    MultiFrameBfsHeuristic(
        EnvironmentROBARM3D* env,
        const OccupancyGridConstPtr& grid,
        const PlanningParams* params);

    virtual ~MultiFrameBfsHeuristic();

    bool setGoal(int x, int y, int z);
    bool setGoal(double x, double y, double z);

    visualization_msgs::MarkerArray getWallsVisualization() const;
    visualization_msgs::MarkerArray getValuesVisualization() const;

    double getMetricDistance(double x, double y, double z);

    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id);

private:

    std::unique_ptr<BFS_3D> m_bfs;
    std::unique_ptr<BFS_3D> m_ee_bfs;

    void syncGridAndBfs();
    int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;
};

} // namespace manip
} // namespace sbpl

#endif
