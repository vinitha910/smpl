////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <smpl/graph/adaptive_workspace_lattice.h>

auto std::hash<sbpl::motion::AdaptiveGridState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<decltype(s.gx)>()(s.gx));
    boost::hash_combine(seed, std::hash<decltype(s.gy)>()(s.gy));
    boost::hash_combine(seed, std::hash<decltype(s.gz)>()(s.gz));
    return seed;
}

auto std::hash<sbpl::motion::AdaptiveWorkspaceState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace sbpl {
namespace motion {

std::ostream& operator<<(std::ostream& o, const AdaptiveGridState& s)
{
    o << "{ x: " << s.gx << ", y: " << s.gy << ", z: " << s.gy << " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const AdaptiveWorkspaceState& s)
{
    o << "{ coord: " << s.coord << ", state: " << s.state << " }";
    return o;
}

AdaptiveWorkspaceLattice::AdaptiveWorkspaceLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params,
    OccupancyGrid* grid)
:
    Extension(),
    RobotPlanningSpace(robot, checker, params),
    m_grid(grid),
    m_states(),
    m_dim_grid(),
    m_hi_to_id(),
    m_lo_to_id(),
    m_start_state_id(-1),
    m_goal_state_id(-1)
{
    m_dim_grid.assign(
            m_grid->numCellsX(),
            m_grid->numCellsY(),
            m_grid->numCellsZ(),
            false);

    m_goal_state_id = reserveHashEntry(true);
    ROS_DEBUG_NAMED(params()->graph_log, " goal state has state ID %d", m_goal_state_id);
}

AdaptiveWorkspaceLattice::~AdaptiveWorkspaceLattice()
{
    for (AdaptiveState* state : m_states) {
        if (state->hid) {
            AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
            delete hi_state;
        } else {
            AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
            delete lo_state;
        }
    }
}

bool AdaptiveWorkspaceLattice::init(const Params& _params)
{
    if (!WorkspaceLatticeBase::init(_params)) {
        return false;
    }

    return true;
}

bool AdaptiveWorkspaceLattice::initialized() const
{
    return false;
}

bool AdaptiveWorkspaceLattice::projectToPoint(
    int state_id,
    Eigen::Vector3d& pos)
{
    if (state_id == m_goal_state_id) {
        assert(goal().tgt_off_pose.size() >= 3);
        pos.x() = goal().tgt_off_pose[0];
        pos.y() = goal().tgt_off_pose[1];
        pos.z() = goal().tgt_off_pose[2];
        return true;
    }

    AdaptiveState* state = m_states[state_id];
    if (state->hid) {
        AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
    } else {
        AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
        pos.x() = lo_state->x;
        pos.y() = lo_state->y;
        pos.z() = lo_state->z;
    }

    return true;
}

bool AdaptiveWorkspaceLattice::addFullDimRegion(int state_id)
{
    return false;
}

bool AdaptiveWorkspaceLattice::setTunnel(const std::vector<int>& states)
{
    return false;
}

int AdaptiveWorkspaceLattice::getStartStateID() const
{
    return m_start_state_id;
}

int AdaptiveWorkspaceLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

bool AdaptiveWorkspaceLattice::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    return false;
}

Extension* AdaptiveWorkspaceLattice::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<AdaptiveGraphExtension>())
    {
        return this;
    }
    return nullptr;
}

void AdaptiveWorkspaceLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    if (state_id == m_goal_state_id) {
        return;
    }

    AdaptiveState* state = m_states[state_id];
    if (state_id->hid) {
        AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
        return GetSuccs(*hi_state, succs, costs);
    } else {
        AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
        return GetSuccs(*lo_state, succs, costs);
    }
}

void AdaptiveWorkspaceLattice::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{

}

void AdaptiveWorkspaceLattice::PrintState(
    int state_id,
    bool verbose,
    FILE* f)
{

}

void AdaptiveWorkspaceLattice::GetSuccs(
    const AdaptiveGridState& state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{

}

void AdaptiveWorkspaceLattice::GetSuccs(
    const AdaptiveWorkspaceState& state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{

}

int AdaptiveWorkspaceLattice::reserveHashEntry(bool hid)
{
    AdaptiveState* entry;
    if (hid) {
        entry = new AdaptiveWorkspaceState;
    } else {
        entry = new AdaptiveGridState;
    }
    entry->hid = hid;

    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    // map planner state -> graph state
    int* pinds = new int[NUMOFINDICES_STATEID2IND];
    std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(pinds);

    return state_id;
}

} // namespace motion
} // namespace sbpl
