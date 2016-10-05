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
#include <sbpl_arm_planner/bfs3d/bfs3d.h>
#include <sbpl_arm_planner/collision_checker.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/planning_params.h>
#include <sbpl_arm_planner/robot_model.h>
#include <sbpl_arm_planner/robot_planning_space.h>
#include <sbpl_arm_planner/types.h>

namespace sbpl {
namespace manip {

class RobotHeuristic;

struct ManipLatticeState
{
    int stateID;                // hash entry ID number
    int xyz[3];                 // planning frame cell (x, y, z)
    double dist;
    std::vector<int> coord;     // discrete coordinate
    RobotState state;           // corresponding continuous coordinate
};

inline
bool operator==(const ManipLatticeState& a, const ManipLatticeState& b)
{
    return a.coord == b.coord;
}

} // namespace manip
} // namespace sbpl

namespace std {

template <>
struct hash<sbpl::manip::ManipLatticeState>
{
    typedef sbpl::manip::ManipLatticeState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace sbpl {
namespace manip {

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

    const ManipLatticeState* getHashEntry(int state_id) const;

    bool computePlanningFrameFK(
        const std::vector<double>& state,
        std::vector<double>& pose) const;

    const std::vector<double>& getGoal() const;

    std::vector<double> getTargetOffsetPose(
        const std::vector<double>& tip_pose) const;

    std::vector<double> getStartConfiguration() const;
    std::vector<double> getGoalConfiguration() const;

    double getStartDistance(double x, double y, double z);
    double getStartDistance(const std::vector<double>& pose);

    double getGoalDistance(double x, double y, double z);
    double getGoalDistance(const std::vector<double>& pose);

    virtual void getExpandedStates(
        std::vector<std::vector<double>>& ara_states) const;

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

    bool m_near_goal;
    clock_t m_t_start;

    GoalConstraint m_goal;

    ManipLatticeState* m_goal_entry;
    ManipLatticeState* m_start_entry;

    // maps from coords to stateID
    hash_map<ManipLatticeState*, int, StateHash, StateEqual> m_state_to_id;

    // maps from stateID to coords
    std::vector<ManipLatticeState*> m_states;

    // stateIDs of expanded states
    std::vector<int> m_expanded_states;

    bool setGoalPosition(
        const std::vector<std::vector<double>>& goals,
        const std::vector<std::vector<double>>& offsets,
        const std::vector<std::vector<double>>& tolerances);

    bool setGoalConfiguration(
        const std::vector<double>& angles,
        const std::vector<double>& angle_tolerances);

    virtual bool StateID2Angles(int stateID, RobotState& angles) const;

    ManipLatticeState* getHashEntry(const std::vector<int>& coord);
    ManipLatticeState* createHashEntry(
        const std::vector<int>& coord,
        const RobotState& state,
        double dist,
        int endeff[3]);
    ManipLatticeState* getOrCreateState(
        const std::vector<int>& coord,
        const RobotState& state,
        double dist,
        int endeff[3]);

    /// \name coordinate frame/angle functions
    ///@{
    virtual void coordToAngles(
        const std::vector<int>& coord,
        std::vector<double>& angles) const;
    virtual void anglesToCoord(
        const std::vector<double>& angle,
        std::vector<int>& coord) const;
    ///@}

    /// \name planning
    ///@{
    virtual bool isGoal(
        const RobotState& state,
        const std::vector<double>& pose);
    ///@}

    /// \name costs
    ///@{
    int cost(
        ManipLatticeState* HashEntry1,
        ManipLatticeState* HashEntry2,
        bool bState2IsGoal);
    virtual void computeCostPerCell();
    int getActionCost(
        const std::vector<double>& from_config,
        const std::vector<double>& to_config,
        int dist);

    bool checkAction(
        const RobotState& state,
        const Action& action,
        double& dist);
    ///@}

    /// \name output
    ///@{
    void printJointArray(
        FILE* fOut,
        ManipLatticeState* HashEntry,
        bool bVerbose);
    ///@}

    visualization_msgs::MarkerArray getStateVisualization(
        const std::vector<double>& vars,
        const std::string& ns);
};

} // namespace manip
} // namespace sbpl

#endif
