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

#ifndef SMPL_ADAPTIVE_WORKSPACE_LATTICE_H
#define SMPL_ADAPTIVE_WORKSPACE_LATTICE_H

// standard includes
#include <functional>
#include <ostream>
#include <tuple>
#include <vector>

// project includes
#include <smpl/grid.h>
#include <smpl/types.h>
#include <smpl/graph/robot_planning_space.h>

namespace sbpl {
namespace motion {

struct AdaptiveState
{
    bool hid;
};

struct AdaptiveGridState : public AdaptiveState
{
    double x;
    double y;
    double z;
    int gx;
    int gy;
    int gz;
};

std::ostream& operator<<(std::ostream& o, const AdaptiveGridState& s);
inline
bool operator==(const AdaptiveGridState& a, const AdaptiveGridState& b)
{
    return std::tie(a.gx, a.gy, a.gz) == std::tie(b.gx, b.gy, b.gz);
}

struct AdaptiveWorkspaceState : public AdaptiveState
{
    RobotState state;
    WorkspaceCoord coord;
};

std::ostream& operator<<(std::ostream& o, const AdaptiveWorkspaceState& s);
inline
bool operator==(
    const AdaptiveWorkspaceState& a,
    const AdaptiveWorkspaceState& b)
{
    return a.coord == b.coord;
}

} // namespace motion
} // namespace sbpl

namespace std {

template <>
struct hash<sbpl::motion::AdaptiveGridState>
{
    typedef sbpl::motion::AdaptiveGridState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

template <>
struct hash<sbpl::motion::AdaptiveWorkspaceState>
{
    typedef sbpl::motion::AdaptiveWorkspaceState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace sbpl {
namespace motion {

class AdaptiveWorkspaceLattice :
    public RobotPlanningSpace,
    public AdaptiveGraphExtension,
    public PointProjectionExtension
{
public:

    /// WorkspaceLattice-specific parameters
    struct Params
    {
        // NOTE: (x, y, z) resolutions defined by the input occupancy grid

        int R_count;
        int P_count;
        int Y_count;

        std::vector<double> free_angle_res;
    };

    AdaptiveWorkspaceLattice(
        RobotModel* robot,
        CollisionChecker* checker,
        PlanningParams* params,
        OccupancyGrid* grid);

    ~AdaptiveWorkspaceLattice();

    /// \name Reimplemented Public Functions from WorkspaceLatticeBase
    ///@{
    bool init(const Params& _params) override;
    bool initialized() const override;
    ///@}

    /// \name Required Public Functions from PointProjectionExtension
    ///@{
    bool projectToPoint(int state_id, Eigen::Vector3d& pos) override;
    ///@}

    /// \name Required Public Functions from AdaptiveGraphExtension
    ///@{
    bool addFullDimRegion(int state_id) override;
    bool setTunnel(const std::vector<int>& states) override;
    ///@}

    /// \name Required Public Functions from RobotPlanningSpcae
    ///@{
    int getStartStateID() const override;
    int getGoalStateID() const override;

    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    virtual Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;

    void PrintState(int state_id, bool verbose, FILE* f = nullptr) override;
    ///@}

private:

    std::vector<AdaptiveState*> m_states;

    Grid3<bool> m_dim_grid;

    typedef AdaptiveWorkspaceState HiStateKey;
    typedef PointerValueHash<HiStateKey> HiStateHash;
    typedef PointerValueEqual<HiStateKey> HiStateEqual;
    typedef AdaptiveGridState LoStateKey;
    typedef PointerValueHash<LoStateKey> LoStateHash;
    typedef PointerValueEqual<LoStateKey> LoStateEqual;

    hash_map<HiStateKey*, int, HiStateHash, HiStateEqual> m_hi_to_id;
    hash_map<LoStateKey*, int, LoStateHash, LoStateEqual> m_lo_to_id;

    int m_start_state_id;
    int m_goal_state_id;

    void GetSuccs(
        const AdaptiveGridState& state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetSuccs(
        const AdaptiveWorkspaceState& state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    int reserveHashEntry(bool hid);
};

} // namespace motion
} // namespace sbpl

#endif
