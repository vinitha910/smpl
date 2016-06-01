////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Maxim Likhachev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the University of Pennsylvania nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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

/// \author: Benjamin Cohen

#ifndef sbpl_manip_manip_lattice_h
#define sbpl_manip_manip_lattice_h

// standard includes
#include <time.h>
#include <memory>
#include <string>
#include <vector>

// system includes
#include <angles/angles.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/planners/planner.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/mdpconfig.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <bfs3d/BFS_3D.h>
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/planning_params.h>
#include <sbpl_manipulation_components/collision_checker.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_manipulation_components/robot_model.h>

namespace sbpl {
namespace manip {

class ManipHeuristic;

enum GoalType
{
    INVALID_GOAL_TYPE = -1,
    XYZ_GOAL,
    XYZ_RPY_GOAL,
    NUMBER_OF_GOAL_TYPES
};

struct GoalConstraint
{
    std::vector<double> pose;           // goal pose of the planning link
    std::vector<double> tgt_off_pose;   // goal pose offset from planning link
    double xyz_offset[3];               // offset from the planning link
    double xyz_tolerance[3];            // (x, y, z) tolerance
    double rpy_tolerance[3];            // (R, P, Y) tolerance
    GoalType type;                      // type of goal constraint
};

struct GoalConstraint7DOF
{
    std::vector<double> angles;
    std::vector<double> angle_tolerances;
};

struct EnvROBARM3DHashEntry_t
{
    int stateID;                // hash entry ID number
    int heur;                   // cached heuristic value (why, I don't know; it's only used in getExpandedStates and another print)
    int xyz[3];                 // planning link pos (xyz)
    double dist;
    std::vector<int> coord;     // discrete coordinate
    RobotState state;           // corresponding continuous coordinate
};

/// \class Environment to be used when planning for a Robotic Arm using the SBPL
class ManipLattice : public DiscreteSpaceInformation
{
public:

    ManipLattice(
        OccupancyGrid* grid,
        RobotModel* rmodel,
        CollisionChecker* cc,
        ActionSet* as,
        PlanningParams* pm);

    ~ManipLattice();

    bool initEnvironment(ManipHeuristic* heur);

    virtual bool setStartConfiguration(const std::vector<double>& angles);

    /// \brief Set a 6-dof goal pose for the tip link.
    ///
    /// \param goals A list of goal poses/positions for offsets from the tip
    ///      link. The format of each element is { x_i, y_i, z_i, R_i, P_i, Y_i,
    ///      6dof? } where the first 6 elements specify the goal pose of the end
    ///      effector and the 7th element is a flag indicating whether
    ///      orientation constraints are required.
    ///
    /// \param offsets A list of offsets from the tip link corresponding to
    ///     \p goals. The goal condition and the heuristic values will be
    ///     computed relative to these offsets.
    ///
    /// \param tolerances A list of goal pose/position tolerances corresponding
    ///     to the \p goals. The format of each element is { dx_i, dy_i, dz_i,
    ///     dR_i, dP_i, dY_i } in meters/radians.
    virtual bool setGoalPosition(
        const std::vector<std::vector<double>>& goals,
        const std::vector<std::vector<double>>& offsets,
        const std::vector<std::vector<double>>& tolerances);

    /// \brief Set a full joint configuration goal.
    virtual bool setGoalConfiguration(
        const std::vector<double>& angles,
        const std::vector<double>& angle_tolerances);

    virtual void getExpandedStates(
        std::vector<std::vector<double>>& ara_states) const;

    virtual void convertStateIDPathToJointAnglesPath(
        const std::vector<int>& idpath,
        std::vector<std::vector<double>>& path) const;

    virtual bool convertStateIDPathToJointTrajectory(
        const std::vector<int>& idpath,
        trajectory_msgs::JointTrajectory& traj) const;

    virtual void convertStateIDPathToShortenedJointAnglesPath(
        const std::vector<int>& idpath,
        std::vector<std::vector<double>>& path,
        std::vector<int>& idpath_short);

    virtual void StateID2Angles(int stateID, std::vector<double>& angles) const;

    RobotModel* getRobotModel() { return rmodel_; }
    CollisionChecker* getCollisionChecker() { return cc_; }
    ros::Publisher& visualizationPublisher() { return m_vpub; }
    bool use7DOFGoal() const { return m_use_7dof_goal; }

    /// \brief Return the ID of the goal state or -1 if no goal has been set.
    int getGoalStateID() const;

    /// \brief Return the ID of the start state or -1 if no start has been set.
    int getStartStateID() const;

    /// \brief Return the 6-dof goal pose for the tip link.
    ///
    /// Return the 6-dof goal pose for the tip link, as last set by
    /// setGoalPosition(). If no goal has been set, the returned vector is
    /// empty.
    const std::vector<double>& getGoal() const;

    /// \brief Return the 6-dof goal pose for the offset from the tip link.
    std::vector<double> getTargetOffsetPose(
        const std::vector<double>& tip_pose) const;

    const GoalConstraint& getCartesianGoal() const;

    /// \brief Return the full joint configuration goal.
    ///
    /// Return the full joint configuration goal, as last set by
    /// setGoalConfiguration().
    std::vector<double> getGoalConfiguration() const;

    const GoalConstraint7DOF& getJointGoal() const;

    std::vector<double> getStart() const;

    /// \brief Get the (heuristic) distance from the planning frame position to the goal
    double getDistanceToGoal(double x, double y, double z);

    // \brief Get the (heuristic) distance from the planning link pose to the goal
    double getDistanceToGoal(const std::vector<double>& pose);

    const EnvROBARM3DHashEntry_t* getHashEntry(int state_id) const;
    bool isGoal(int state_id) const;

    /// \name Reimplemented Public Functions
    ///@{
    bool InitializeMDPCfg(MDPConfig* MDPCfg);
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
    virtual int GetGoalHeuristic(int stateID);
    virtual int GetStartHeuristic(int stateID);
    virtual void GetSuccs(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV);
    virtual void GetLazySuccs(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV,
        std::vector<bool>* isTrueCost);
    virtual int GetTrueCost(int parentID, int childID);
    int SizeofCreatedEnv();
    void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);
    ///@}

    /// \name Reimplemented Public Functions (Unused)
    ///@{
    virtual bool InitializeEnv(const char* sEnvFile) { return false; }
    void GetPreds(
        int TargetStateID,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV);
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    void SetAllPreds(CMDPSTATE* state);
    void PrintEnv_Config(FILE* fOut);
    ///@}

protected:

    // Context Interfaces
    OccupancyGrid* grid_;
    RobotModel* rmodel_;
    CollisionChecker* cc_;
    ActionSet* as_;

    PlanningParams* prm_;

    ManipHeuristic* m_heur;

    // cached from robot model
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool> m_continuous;

    bool m_near_goal;
    clock_t m_t_start;
    double m_time_to_goal_region;

    bool m_use_7dof_goal;

    GoalConstraint m_goal;
    GoalConstraint7DOF m_goal_7dof;

    EnvROBARM3DHashEntry_t* m_goal_entry;
    EnvROBARM3DHashEntry_t* m_start_entry;

    // maps from coords to stateID
    int m_HashTableSize;
    std::vector<EnvROBARM3DHashEntry_t*>* m_Coord2StateIDHashTable;

    // maps from stateID to coords
    std::vector<EnvROBARM3DHashEntry_t*> m_states;

    // stateIDs of expanded states
    std::vector<int> m_expanded_states;

    ros::NodeHandle nh_;
    ros::Publisher m_vpub;

    bool m_initialized;

    // note: const althought RobotModel::computePlanningLinkFK used underneath
    // may not be
    bool computePlanningFrameFK(
        const std::vector<double>& state,
        std::vector<double>& pose) const;

    /** hash table */
    unsigned int intHash(unsigned int key);
    unsigned int getHashBin(const std::vector<int>& coord);
    virtual EnvROBARM3DHashEntry_t* getHashEntry(
        const std::vector<int>& coord,
        bool bIsGoal);
    virtual EnvROBARM3DHashEntry_t* createHashEntry(
        const std::vector<int>& coord,
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
    virtual bool isGoalState(
        const std::vector<double>& pose,
        GoalConstraint& goal);
    virtual bool isGoalState(
        const std::vector<double>& angles,
        GoalConstraint7DOF& goal);
    virtual bool isGoal(
        const std::vector<double>& angles,
        const std::vector<double>& pose);
    ///@}

    /// \name costs
    ///@{
    int cost(
        EnvROBARM3DHashEntry_t* HashEntry1,
        EnvROBARM3DHashEntry_t* HashEntry2,
        bool bState2IsGoal);
    virtual void computeCostPerCell();
    int getActionCost(
        const std::vector<double>& from_config,
        const std::vector<double>& to_config,
        int dist);
    ///@}

    /// \name output
    ///@{
    void printHashTableHist();
    void printJointArray(
        FILE* fOut,
        EnvROBARM3DHashEntry_t* HashEntry,
        bool bGoal,
        bool bVerbose);
    ///@}

    void visualizeState(
        const std::vector<double>& jvals,
        const std::string& ns) const;
};

} // namespace manip
} // namespace sbpl

#endif
