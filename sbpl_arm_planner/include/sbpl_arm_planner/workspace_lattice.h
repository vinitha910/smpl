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
#include <unordered_map>
#include <vector>

// system includes
#include <sbpl/headers.h>

// project includes
#include <sbpl_arm_planner/collision_checker.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/robot_model.h>
#include <sbpl_arm_planner/types.h>

namespace sbpl {
namespace manip {

#define BROKEN 1

typedef std::vector<double> WorkspaceState;
typedef std::vector<int> WorkspaceCoord;
typedef std::vector<double> SixPose;

struct WorkspaceLatticeState
{
    // discrete coordinate ( x, y, z, R, P, Y, j1, ..., jn )
    WorkspaceCoord coord;

    // real-valued state
    RobotState state;
};

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

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

/// \class Discrete state lattice representation representing a robot as the
///     pose of one of its links and all redundant joint variables
class WorkspaceLattice : public DiscreteSpaceInformation
{
public:

    struct Params
    {
        int R_count;
        int P_count;
        int Y_count;
        int free_angle_index;
        std::vector<double> res_joints;
    };

    WorkspaceLattice(
        OccupancyGrid *grid,
        RobotModel* robot,
        CollisionChecker* cc,
        PlanningParams* params);

    ~WorkspaceLattice();

    bool init(const Params& params);
    bool initialized() const;

    RobotModel* getRobotModel() { return m_robot; }
    CollisionChecker* getCollisionChecker();

    /// \name Start and Goal States
    ///@{
    bool setStartState(const RobotState& state);

    struct PoseGoal
    {
        std::vector<double> pose;
        std::vector<double> offset;
        std::vector<double> tolerance;
    };

    bool setGoalPose(const PoseGoal& goal);
    bool setGoalPoses(const std::vector<PoseGoal>& goals);

    int getStartStateID() const;
    int getGoalStateID() const;
    ///@}

    /// \name Path Extraction
    ///@{
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path);

    // TODO: add variant that returns the path in the workspace representation
    ///@}

    /// \name Reimplemented Public Functions
    ///@{
    virtual int GetFromToHeuristic(int from_state_id, int to_state_id) override;
    virtual int GetGoalHeuristic(int state_id) override;
    virtual int GetStartHeuristic(int state_id) override;
    virtual void GetSuccs(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV) override;
    virtual void PrintState(
        int state_id,
        bool verbose,
        FILE* fout = nullptr) override;
    ///@}

    /// \name Reimplemented Public Functions (Unused)
    ///@{
    virtual bool InitializeMDPCfg(MDPConfig* MDPCfg) override { return false; }
    virtual bool InitializeEnv(const char* sEnvFile) override { return false; }
    virtual void GetPreds(
        int TargetStateID,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV) override;
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) override;
    virtual void SetAllPreds(CMDPSTATE* state) override;
    virtual void PrintEnv_Config(FILE* fOut) override;
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

    ros::Publisher m_vpub;

    // Context Interfaces
    OccupancyGrid* m_grid;
    RobotModel* m_robot;
    CollisionChecker* m_cc;

    PlanningParams* m_params;

    ManipHeuristic* m_heur;

    GoalConstraint m_goal;

    WorkspaceLatticeState* m_goal_entry;
    int m_goal_state_id;
    WorkspaceLatticeState* m_start_entry;

    // maps state -> id
    hash_map<const WorkspaceLatticeState*, int, StateHash, StateEqual> m_state_to_id;

    // maps id -> state
    std::vector<WorkspaceLatticeState*> m_states;

    std::vector<double> mp_gradient_;
    std::vector<double> mp_dist_;

    /// \name State Space Configuration
    ///@{

    // ( res_x, res_y, res_z, res_R, res_P, res_Y, res_j1, ..., res_j2 )
    std::vector<double> m_res;
    std::vector<double> m_val_count;
    int m_dof_count;
    int m_free_angle_count;
    int m_free_angle_idx;

    ///@}

    /// \name Action Space Configuration
    ///@{

    std::vector<MotionPrimitive> m_prims;

    ///@}

    int createState(const WorkspaceCoord& coord);
    WorkspaceLatticeState* getState(int state_id);

    // conversions between robot states and workspace states discrete and continuous
    void stateRobotToWorkspace(const RobotState& state, WorkspaceState& ostate);
    void stateRobotToCoord(const RobotState& state, WorkspaceCoord& coord);
    bool stateWorkspaceToRobot(const WorkspaceState& state, RobotState& ostate);
    void stateWorkspaceToCoord(const WorkspaceState& state, WorkspaceCoord& coord);
    bool stateCoordToRobot(const WorkspaceCoord& coord, RobotState& state);
    void stateCoordToWorkspace(const WorkspaceCoord& coord, WorkspaceState& state);

    // conversions from discrete coordinates to continuous states
    void posWorkspaceToCoord(double* cp, int* dp);
    void posCoordToWorkspace(int* dp, double* cp);
    void rotWorkspaceToCoord(double* cr, int* dr);
    void rotCoordToWorkspace(int* dr, double* cr);
    void poseWorkspaceToCoord(double* cp, int* dp);
    void poseCoordToWorkspace(int* dp, double* cp);
    void favWorkspaceToCoord(double* ca, int* da);
    void favCoordToWorkspace(int* da, double* ca);

    void getActions(const RobotState& state, std::vector<Action>& actions);

    bool checkAction(
        const RobotState& state,
        const Action& action,
        double& dist);

    // TODO: remove these in favor of above conventions
    void robotStateToCoord(const RobotState& state, WorkspaceCoord& coord);
    void stateIDToPose(int stateID, int* xyz, int* rpy, int* fangle);
    void worldPoseToCoord(double* wxyz, double* wrpy, double wfangle, WorkspaceCoord& coord);
    void discToWorldXYZ(const int* xyz, double* wxyz);
    void discToWorldRPY(int* rpy, double* wrpy);
    void discToWorldFAngle(int fangle, double* wfangle);
    void worldToDiscXYZ(const double* wxyz, int* xyz);
    void worldToDiscRPY(double* wrpy, int* rpy);
    void worldToDiscFAngle(double wfangle, int* fangle);
    bool convertWorldPoseToAngles(const std::vector<double>& wpose, std::vector<double>& angles);
    bool convertWorldPoseToAngles(const std::vector<double>& wpose, std::vector<double> seed, std::vector<double>& angles);

    bool isGoal(const WorkspaceState& state);

    int getJointAnglesForMotionPrimWaypoint(
        const std::vector<double>& mp_point,
        const WorkspaceState& wcoord,
        const RobotState& pangles,
        WorkspaceState& final_wcoord,
        std::vector<RobotState>& angles);

    int computeMotionCost(const RobotState& a, const RobotState& b);

    void visualizeState(const RobotState& state, const std::string& ns);

#if !BROKEN
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

