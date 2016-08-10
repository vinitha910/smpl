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

struct WorkspaceLatticeState
{
    // discrete coordinate ( x, y, z, R, P, Y, j1, ..., jn )
    std::vector<int> coord;

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

    void computeGradient(
        const MotionPrimitive &mp,
        unsigned char &d,
        bool collision);

    int getXYZRPYHeuristic(int FromStateID, int ToStateID);

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

protected:

    virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* ActionV=NULL );

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

    PlanningParams* m_params;

    ManipHeuristic* m_heur;

    GoalConstraint m_goal;

    WorkspaceLatticeState* m_goal_entry;
    WorkspaceLatticeState* m_start_entry;

    // maps state -> id
    hash_map<const WorkspaceLatticeState*, int, StateHash, StateEqual> m_state_to_id;

    // maps id -> state
    std::vector<WorkspaceLatticeState*> m_states;

    // ( res_x, res_y, res_z, res_R, res_P, res_Y, res_j1, ..., res_j2 )

    std::vector<double> mp_gradient_;
    std::vector<double> mp_dist_;

    // state space information
    std::vector<double> m_res;
    std::vector<double> m_val_count;
    int m_dof_count;
    int m_free_angle_count;
    int m_free_angle_idx;

    // action space
    std::vector<MotionPrimitive> m_mprims;

    int createState(const std::vector<int>& coord);
    WorkspaceLatticeState* getState(int state_id);

    void StateID2Angles(int stateID, std::vector<double>& angles);

    /* discretization */
    void coordToPose(
        const std::vector<int>& coord,
        int* xyz,
        int* rpy,
        int* fangle);
    void coordToWorldPose(
        const std::vector<int>& coord,
        double* xyz,
        double* rpy,
        double* fangle);
    void coordToWorldPose(
        const std::vector<int>& coord,
        std::vector<double>& wcoord);
    void robotStateToCoord(
        const RobotState& state,
        std::vector<int>& coord);
    void stateIDToPose(
        int stateID,
        int* xyz,
        int* rpy,
        int* fangle);
    void stateIDToWorldPose(
        int stateID,
        double* wxyz,
        double* wrpy,
        double* wfangle);
    void worldPoseToCoord(
        double* wxyz,
        double* wrpy,
        double wfangle,
        std::vector<int>& coord);
    void coordToAngles(
        const std::vector<int>& coord,
        std::vector<double>& angles);
    void discToWorldXYZ(const int* xyz, double* wxyz);
    void discToWorldRPY(int* rpy, double* wrpy);
    void discToWorldFAngle(int fangle, double* wfangle);
    void worldToDiscXYZ(const double* wxyz, int* xyz);
    void worldToDiscRPY(double* wrpy, int* rpy);
    void worldToDiscFAngle(double wfangle, int* fangle);
    bool convertCoordToAngles(
        const std::vector<int>* coord,
        std::vector<double>* angles);
    bool convertWorldPoseToAngles(
        const std::vector<double>& wpose,
        std::vector<double>& angles);
    bool convertWorldPoseToAngles(
        const std::vector<double>& wpose,
        std::vector<double> seed,
        std::vector<double>& angles);

    bool isGoalPosition(double* xyz, double* rpy, double fangle);
    void getAdaptiveMotionPrim(
        int type,
        WorkspaceLatticeState* parent,
        MotionPrimitive& mp);
    int getJointAnglesForMotionPrimWaypoint(
        const std::vector<double>& mp_point,
        const std::vector<double>& wcoord,
        const std::vector<double>& pangles,
        std::vector<double>& final_wcoord,
        std::vector<std::vector<double>>& angles);
    bool getMotionPrimitive(WorkspaceLatticeState* parent, MotionPrimitive& mp);
    int isMotionValid(
        const std::vector<double>& start,
        const std::vector<double>& end,
        int& path_length,
        int& nchecks,
        unsigned char& dist);
    void getVector(
        int x1, int y1, int z1,
        int x2, int y2, int z2,
        int& xout, int& yout, int& zout,
        int multiplier,
        bool snap = true);
    bool getDistanceGradient(int& x, int& y, int& z);
    void computeCostPerCell();
    int computeMotionCost(
        const std::vector<double>& a,
        const std::vector<double>& b);
};

inline
void WorkspaceLattice::discToWorldXYZ(const int* gp, double* wp)
{
    m_grid->gridToWorld(gp[0], gp[1], gp[2], wp[0], wp[1], wp[2]);
}

inline
void WorkspaceLattice::worldToDiscXYZ(const double* wp, int* gp)
{
    m_grid->worldToGrid(wp[0], wp[1], wp[2], gp[0], gp[0], gp[0]);
}

inline
void WorkspaceLattice::discToWorldRPY(int* rpy, double* wrpy)
{
    wrpy[0] = angles::normalize_angle((double)rpy[0] * EnvROBARMCfg.coord_delta[3]);
    wrpy[1] = angles::normalize_angle((double)rpy[1] * EnvROBARMCfg.coord_delta[4]);
    wrpy[2] = angles::normalize_angle((double)rpy[2] * EnvROBARMCfg.coord_delta[5]);

    ROS_DEBUG("[discToWorldRPY] rpy: %d %d %d --> %2.3f %2.3f %2.3f", rpy[0], rpy[1], rpy[2], wrpy[0], wrpy[1], wrpy[2]);
}

inline
void WorkspaceLattice::worldToDiscRPY(double* wrpy, int* rpy)
{
    rpy[0] = (int)((angles::normalize_angle_positive(wrpy[0]) + EnvROBARMCfg.coord_delta[3] * 0.5) / EnvROBARMCfg.coord_delta[3]) % EnvROBARMCfg.coord_vals[3];
    rpy[1] = (int)((angles::normalize_angle_positive(wrpy[1]) + EnvROBARMCfg.coord_delta[4] * 0.5) / EnvROBARMCfg.coord_delta[4]) % EnvROBARMCfg.coord_vals[4];
    rpy[2] = (int)((angles::normalize_angle_positive(wrpy[2]) + EnvROBARMCfg.coord_delta[5] * 0.5) / EnvROBARMCfg.coord_delta[5]) % EnvROBARMCfg.coord_vals[5];

    ROS_DEBUG("[worldToDiscRPY] rpy: %2.3f %2.3f %2.3f --> %d %d %d", wrpy[0], wrpy[1], wrpy[2], rpy[0], rpy[1], rpy[2]);

    if (rpy[0] >= EnvROBARMCfg.coord_vals[3] || rpy[1] >= EnvROBARMCfg.coord_vals[4] || rpy[2] >= EnvROBARMCfg.coord_vals[5]) {
        ROS_ERROR("[worldToDiscRPY] wrpy: %0.3f %0.3f %0.3f discretized to %d %d %d", wrpy[0], wrpy[1], wrpy[2], rpy[0], rpy[1], rpy[2]);
    }
}

inline
void WorkspaceLattice::discToWorldFAngle(int fangle, double* wfangle)
{
    *wfangle = angles::normalize_angle((double)fangle * EnvROBARMCfg.coord_delta[6]);

    ROS_DEBUG("[discToWorldFAngle] fangle: %d --> %2.3f", fangle, *wfangle);
}

inline
void WorkspaceLattice::worldToDiscFAngle(double wfangle, int* fangle)
{
    *fangle = (int)((angles::normalize_angle_positive(wfangle) + EnvROBARMCfg.coord_delta[6] * 0.5) / EnvROBARMCfg.coord_delta[6]) % EnvROBARMCfg.coord_vals[6];

    ROS_DEBUG("[worldToDiscFAngle] fangle: %2.3f --> %d", wfangle, *fangle);

    if (*fangle >= EnvROBARMCfg.coord_vals[6]) {
        ROS_ERROR("[worldToDiscFAngle] fangle: %0.3f discretized to: %d", wfangle, *fangle);
    }
}

inline
void WorkspaceLattice::coordToPose(
    const std::vector<int>& coord,
    int* xyz,
    int* rpy,
    int* fangle)
{
    xyz[0] = coord[0];
    xyz[1] = coord[1];
    xyz[2] = coord[2];
    rpy[0] = coord[3];
    rpy[1] = coord[4];
    rpy[2] = coord[5];
    *fangle = coord[6];

    ROS_DEBUG("[coordToPose] xyz: %u %u %u  rpy: %d %d %d fangle: %u", xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2], *fangle);
}

inline
void WorkspaceLattice::coordToWorldPose(
    const std::vector<int>& coord,
    double* wxyz,
    double* wrpy,
    double* wfangle)
{
    int xyz[3] = {0}, rpy[3] = {0}, fangle = 0;

    coordToPose(coord,xyz,rpy,&fangle);

    discToWorldXYZ(xyz,wxyz);
    discToWorldRPY(rpy,wrpy);
    discToWorldFAngle(fangle,wfangle);
}

inline
void WorkspaceLattice::coordToWorldPose(
    const std::vector<int>& coord,
    std::vector<double>& wcoord)
{
    double xyz[3] = { 0 };
    double rpy[3] = { 0 };
    double fangle = 0;

    coordToWorldPose(coord, xyz, rpy, &fangle);

    wcoord[0] = xyz[0];
    wcoord[1] = xyz[1];
    wcoord[2] = xyz[2];
    wcoord[3] = rpy[0];
    wcoord[4] = rpy[1];
    wcoord[5] = rpy[2];
    wcoord[6] = fangle;
}

} // namespace manip
} // namespace sbpl

#endif

