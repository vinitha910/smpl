////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#ifndef sbpl_manip_manip_lattice_h
#define sbpl_manip_manip_lattice_h

// standard includes
#include <time.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <sbpl/planners/planner.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/mdpconfig.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/graph/robot_planning_space.h>

namespace sbpl {
namespace motion {

class RobotHeuristic;

typedef std::vector<int> RobotCoord;

struct ManipLatticeState
{
    int stateID;        // hash entry ID number
    RobotCoord coord;   // discrete coordinate
    RobotState state;   // corresponding continuous coordinate
};

inline
bool operator==(const ManipLatticeState& a, const ManipLatticeState& b)
{
    return a.coord == b.coord;
}

} // namespace motion
} // namespace sbpl

namespace std {

template <>
struct hash<sbpl::motion::ManipLatticeState>
{
    typedef sbpl::motion::ManipLatticeState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace sbpl {
namespace motion {

SBPL_CLASS_FORWARD(ManipLattice);

/// \class Discrete space constructed by expliciting discretizing each joint
class ManipLattice :
    public RobotPlanningSpace,
    public PointProjectionExtension,
    public ExtractRobotStateExtension
{
public:

    ManipLattice(
        RobotModel* robot,
        CollisionChecker* checker,
        PlanningParams* params,
        OccupancyGrid* grid);

    ~ManipLattice();

    RobotState getStartConfiguration() const;

    double getStartDistance(double x, double y, double z);
    double getStartDistance(const std::vector<double>& pose);

    double getGoalDistance(double x, double y, double z);
    double getGoalDistance(const std::vector<double>& pose);

    void getExpandedStates(std::vector<RobotState>& states) const;

    /// \name Reimplemented Public Functions from RobotPlanningSpace
    ///@{
    void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;
    int GetTrueCost(int parent_id, int child_id) override;
    ///@}

    /// \name Required Public Functions from ExtractRobotStateExtension
    ///@{
    const RobotState& extractState(int state_id);
    ///@}

    /// \name Required Public Functions from PointProjectionExtension
    ///@{
    bool projectToPoint(int state_id, Eigen::Vector3d& pos);
    ///@}

    /// \name Required Public Functions from RobotPlanningSpace
    ///@{
    bool setStart(const RobotState& state) override;
    bool setGoal(const GoalConstraint& goal) override;
    int getStartStateID() const override;
    int getGoalStateID() const override;
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    virtual Extension* getExtension(size_t class_code);
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    void PrintState(int state_id, bool verbose, FILE* fout = nullptr) override;
    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;
    ///@}

protected:

    /// \name discretization methods
    ///@{
    void coordToState(const RobotCoord& coord, RobotState& state) const;
    void stateToCoord(const RobotState& state, RobotCoord& coord) const;
    ///@}

    ManipLatticeState* getHashEntry(int state_id) const;
    ManipLatticeState* getHashEntry(const RobotCoord& coord);
    ManipLatticeState* createHashEntry(
        const RobotCoord& coord,
        const RobotState& state);
    ManipLatticeState* getOrCreateState(
        const RobotCoord& coord,
        const RobotState& state);

    visualization_msgs::MarkerArray getStateVisualization(
        const RobotState& vars,
        const std::string& ns);

private:

    struct StateHash
    {
        typedef const ManipLatticeState* argument_type;
        typedef std::size_t result_type;

        result_type operator()(argument_type s) const
        {
            return std::hash<ManipLatticeState>()(*s);
        }
    };

    struct StateEqual
    {
        typedef const ManipLatticeState* argument_type;
        bool operator()(argument_type a, argument_type b) const
        {
            return *a == *b;
        }
    };

    OccupancyGrid* m_grid;

    ForwardKinematicsInterface* m_fk_iface;

    // cached from robot model
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool> m_continuous;

    ManipLatticeState* m_goal_entry;
    ManipLatticeState* m_start_entry;

    // maps from coords to stateID
    hash_map<ManipLatticeState*, int, StateHash, StateEqual> m_state_to_id;

    // maps from stateID to coords
    std::vector<ManipLatticeState*> m_states;

    // stateIDs of expanded states
    std::vector<int> m_expanded_states;
    bool m_near_goal;
    clock_t m_t_start;

    bool setGoalPose(const GoalConstraint& goal);
    bool setGoalConfiguration(const GoalConstraint& goal);

    void startNewSearch();

    bool computePlanningFrameFK(
        const RobotState& state,
        std::vector<double>& pose) const;

    std::vector<double> getTargetOffsetPose(
        const std::vector<double>& tip_pose) const;

    /// \name planning
    ///@{
    bool isGoal(const RobotState& state, const std::vector<double>& pose);
    ///@}

    /// \name costs
    ///@{
    int cost(
        ManipLatticeState* HashEntry1,
        ManipLatticeState* HashEntry2,
        bool bState2IsGoal);
    void computeCostPerCell();
    int getActionCost(
        const RobotState& first,
        const RobotState& last,
        int dist);

    bool checkAction(
        const RobotState& state,
        const Action& action,
        double& dist);
    ///@}
};

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin
// 0, ...
inline
void ManipLattice::coordToState(
    const RobotCoord& coord,
    RobotState& state) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (size_t i = 0; i < coord.size(); ++i) {
        if (m_continuous[i]) {
            state[i] = coord[i] * params()->coord_delta[i];
        } else {
            state[i] = m_min_limits[i] + coord[i] * params()->coord_delta[i];
        }
    }
}

inline
void ManipLattice::stateToCoord(
    const RobotState& state,
    RobotCoord& coord) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (size_t i = 0; i < state.size(); ++i) {
        if (m_continuous[i]) {
            double pos_angle = angles::normalize_angle_positive(state[i]);

            coord[i] = (int)((pos_angle + params()->coord_delta[i] * 0.5) / params()->coord_delta[i]);

            if (coord[i] == params()->coord_vals[i]) {
                coord[i] = 0;
            }
        }
        else {
            coord[i] = (int)(((state[i] - m_min_limits[i]) / params()->coord_delta[i]) + 0.5);
        }
    }
}

} // namespace motion
} // namespace sbpl

#endif
