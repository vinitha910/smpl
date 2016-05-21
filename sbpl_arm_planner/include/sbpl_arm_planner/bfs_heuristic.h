#ifndef sbpl_manip_bfs_heuristic_h
#define sbpl_manip_bfs_heuristic_h

// standard includes
#include <memory>

// system includes
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_arm_planner/manip_heuristic.h>

namespace sbpl {
namespace manip {

class BFS_3D;

class BfsHeuristic : public ManipHeuristic
{
public:

    BfsHeuristic(
        ManipLattice* env,
        const OccupancyGridConstPtr& grid,
        const PlanningParams* params);

    virtual ~BfsHeuristic();

    bool setGoal(int x, int y, int z);

    visualization_msgs::MarkerArray getWallsVisualization() const;
    visualization_msgs::MarkerArray getValuesVisualization() const;

    /// \name Inherited from ManipHeuristic
    ///@{
    double getMetricGoalDistance(double x, double y, double z);
    ///@}

    /// \name Inherited from Heuristic
    ///@{
    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id);
    ///@}

private:

    std::unique_ptr<BFS_3D> m_bfs;

    void syncGridAndBfs();
    int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;
};

} // namespace manip
} // namespace sbpl

#endif
