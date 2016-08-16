#include <sbpl_arm_planner/robot_state_lattice.h>

namespace sbpl {
namespace manip {

RobotStateLattice::RobotStateLattice()
{
}

RobotStateLattice::~RobotStateLattice()
{
}

void RobotStateLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    GetSuccs(state_id, succs, costs);
    true_costs->assign(succs->size(), true);
}

int RobotStateLattice::GetTrueCost(int parentID, int childID)
{
    return 1;
}

} // namespace manip
} // namespace sbpl
