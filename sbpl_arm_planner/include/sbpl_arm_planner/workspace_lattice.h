////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
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

#ifndef sbpl_manip_workspace_lattice_h
#define sbpl_manip_workspace_lattice_h

// standard includes
#include <vector>

// system includes
#include <ros/ros.h>
#include <sbpl/headers.h>

// project includes
#include <sbpl_arm_planner/collision_checker.h>
#include <sbpl_arm_planner/motion_primitive.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/planning_params.h>
#include <sbpl_arm_planner/robot_model.h>
#include <sbpl_arm_planner/robot_planning_space.h>
#include <sbpl_arm_planner/types.h>

namespace sbpl {
namespace manip {

#define BROKEN 1

/// continuous state ( x, y, z, R, P, Y, j1, ..., jn )
typedef std::vector<double> WorkspaceState;

/// discrete coordinate ( x, y, z, R, P, Y, j1, ..., jn )
typedef std::vector<int> WorkspaceCoord;

/// 6-dof pose ( x, y, z, R, P, Y )
typedef std::vector<double> SixPose;

/// 6-dof pose ( x, y, z, qw, qx, qy, qz )
typedef std::vector<double> SevenPose;

/// 3-dof position ( x, y, z )
typedef std::vector<double> Position;

struct WorkspaceLatticeState
{
    WorkspaceCoord coord;
    RobotState state;
    int h;
};

inline
bool operator==(const WorkspaceLatticeState& a, const WorkspaceLatticeState& b)
{
    return a.coord == b.coord;
}

} // namespace manip
} // namespace sbpl

namespace std {

template <>
struct hash<sbpl::manip::WorkspaceLatticeState>
{
    typedef sbpl::manip::WorkspaceLatticeState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace sbpl {
namespace manip {

/// \class Discrete state lattice representation representing a robot as the
///     pose of one of its links and all redundant joint variables
class WorkspaceLattice : public RobotPlanningSpace
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

    struct PoseGoal
    {
        SixPose pose;
        Position offset;
        SixPose tolerance;
    };

    WorkspaceLattice(
        RobotModel* robot,
        CollisionChecker* checker,
        PlanningParams* params,
        OccupancyGrid* grid);

    ~WorkspaceLattice();

    bool init(RobotModel* robot, const Params& params);
    bool initialized() const;

    // TODO: add path extraction function that returns path in workspace rep

    ///@{
    bool setStart(const RobotState& state);
    bool setGoal(const GoalConstraint& goal);

    int getStartStateID() const;
    int getGoalStateID() const;
    ///@}

    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path);

    /// \name Reimplemented Public Functions from RobotPlanningSpace
    ///@{
    virtual int GetGoalHeuristic(int state_id) override;
    ///@}

    /// \name Required Public Functions from RobotPlanningSpace
    ///@{
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    virtual void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    virtual void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;

    virtual void PrintState(
        int state_id,
        bool verbose,
        FILE* fout = nullptr) override;
    ///@}

private:

    struct StateHash
    {
        typedef const WorkspaceLatticeState* argument_type;
        typedef std::size_t result_type;

        result_type operator()(argument_type s) const
        {
            return std::hash<WorkspaceLatticeState>()(*s);
        }
    };

    struct StateEqual
    {
        typedef const WorkspaceLatticeState* argument_type;
        bool operator()(argument_type a, argument_type b) const
        {
            return *a == *b;
        }
    };

    // Context Interfaces
    OccupancyGrid* m_grid;
    RobotModel* m_robot;
    CollisionChecker* m_cc;

    ForwardKinematicsInterface* m_fk_iface;
    InverseKinematicsInterface* m_ik_iface;
    RedundantManipulatorInterface* m_rm_iface;

    PlanningParams* m_params;

    GoalConstraint m_goal;

    WorkspaceLatticeState* m_goal_entry;
    int m_goal_state_id;

    WorkspaceLatticeState* m_start_entry;
    int m_start_state_id;

    // maps state -> id
    hash_map<const WorkspaceLatticeState*, int, StateHash, StateEqual> m_state_to_id;

    // maps id -> state
    std::vector<WorkspaceLatticeState*> m_states;

    clock_t m_t_start;
    bool m_near_goal;

    /// \name State Space Configuration
    ///@{

    // ( res_x, res_y, res_z, res_R, res_P, res_Y, res_j1, ..., res_j2 )
    std::vector<double> m_res;
    std::vector<int> m_val_count;
    int m_dof_count;
    std::vector<size_t> m_fangle_indices;

    ///@}

    /// \name Action Space Configuration
    ///@{

    std::vector<MotionPrimitive> m_prims;

    ///@}

    bool setGoalPose(const PoseGoal& goal);
    bool setGoalPoses(const std::vector<PoseGoal>& goals);

    size_t freeAngleCount() const { return m_fangle_indices.size(); }

    int createState(const WorkspaceCoord& coord);
    WorkspaceLatticeState* getState(int state_id);

    // conversions between robot states, workspace states, and workspacce coords
    void stateRobotToWorkspace(const RobotState& state, WorkspaceState& ostate);
    void stateRobotToCoord(const RobotState& state, WorkspaceCoord& coord);
    bool stateWorkspaceToRobot(const WorkspaceState& state, RobotState& ostate);
    void stateWorkspaceToCoord(const WorkspaceState& state, WorkspaceCoord& coord);
    bool stateCoordToRobot(const WorkspaceCoord& coord, RobotState& state);
    void stateCoordToWorkspace(const WorkspaceCoord& coord, WorkspaceState& state);

    // TODO: variants of workspace -> robot that don't restrict redundant angles
    // TODO: variants of workspace -> robot that take in a full seed state

    // conversions from discrete coordinates to continuous states
    void posWorkspaceToCoord(const double* gp, int* wp);
    void posCoordToWorkspace(const int* wp, double* gp);
    void rotWorkspaceToCoord(const double* gr, int* wr);
    void rotCoordToWorkspace(const int* wr, double* gr);
    void poseWorkspaceToCoord(const double* gp, int* wp);
    void poseCoordToWorkspace(const int* wp, double* gp);
    void favWorkspaceToCoord(const double* wa, int* ga);
    void favCoordToWorkspace(const int* ga, double* wa);

    void getActions(const WorkspaceLatticeState& state, std::vector<Action>& actions);

    bool checkAction(
        const RobotState& state,
        const Action& action,
        double& dist);

    bool isGoal(const WorkspaceState& state);

    visualization_msgs::MarkerArray getStateVisualization(
        const RobotState& state,
        const std::string& ns);

#if !BROKEN
    std::vector<double> mp_gradient_;
    std::vector<double> mp_dist_;

    int computeMotionCost(const RobotState& a, const RobotState& b);

    int getJointAnglesForMotionPrimWaypoint(
        const std::vector<double>& mp_point,
        const WorkspaceState& wcoord,
        const RobotState& pangles,
        WorkspaceState& final_wcoord,
        std::vector<RobotState>& angles);

    bool getMotionPrimitive(WorkspaceLatticeState* parent, MotionPrimitive& mp);
    void getAdaptiveMotionPrim(
        int type,
        WorkspaceLatticeState* parent,
        MotionPrimitive& mp);
    void getVector(
        int x1, int y1, int z1,
        int x2, int y2, int z2,
        int& xout, int& yout, int& zout,
        int multiplier,
        bool snap = true);
    bool getDistanceGradient(int& x, int& y, int& z);
#endif
};

} // namespace manip
} // namespace sbpl

#endif

