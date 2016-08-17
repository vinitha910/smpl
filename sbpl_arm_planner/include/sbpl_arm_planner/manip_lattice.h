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
#include <vector>

// system includes
#include <sbpl/planners/planner.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/mdpconfig.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/bfs3d/bfs3d.h>
#include <sbpl_arm_planner/collision_checker.h>
#include <sbpl_arm_planner/manip_lattice_observers.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/planning_params.h>
#include <sbpl_arm_planner/robot_model.h>
#include <sbpl_arm_planner/robot_state_lattice.h>
#include <sbpl_arm_planner/types.h>

namespace sbpl {
namespace manip {

class ManipHeuristic;

struct ManipLatticeState
{
    int stateID;                // hash entry ID number
    int heur;                   // cached heuristic value (why, I don't know; it's only used in getExpandedStates and another print)
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

/// \class Discrete state lattice representation representing a robot as the
///     set of all its joint variables
class ManipLattice : public RobotStateLattice
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

    RobotModel* getRobotModel() { return m_robot; }
    CollisionChecker* getCollisionChecker() { return m_cc; }
    ros::Publisher& visualizationPublisher() { return m_vpub; }

    const ManipLatticeState* getHashEntry(int state_id) const;

    bool computePlanningFrameFK(
        const std::vector<double>& state,
        std::vector<double>& pose) const;

    /// \name Start and Goal States
    ///@{
    virtual bool setStartState(const RobotState& angles);

    virtual bool setGoalPosition(
        const std::vector<std::vector<double>>& goals,
        const std::vector<std::vector<double>>& offsets,
        const std::vector<std::vector<double>>& tolerances);

    virtual bool setGoalConfiguration(
        const std::vector<double>& angles,
        const std::vector<double>& angle_tolerances);

    const std::vector<double>& getGoal() const;

    std::vector<double> getTargetOffsetPose(
        const std::vector<double>& tip_pose) const;

    const GoalConstraint& getGoalConstraints() const;

    std::vector<double> getStartConfiguration() const;
    std::vector<double> getGoalConfiguration() const;

    double getStartDistance(double x, double y, double z);
    double getStartDistance(const std::vector<double>& pose);

    double getGoalDistance(double x, double y, double z);
    double getGoalDistance(const std::vector<double>& pose);

    int getStartStateID() const;
    int getGoalStateID() const;

    void insertStartObserver(ManipLatticeStartObserver* observer);
    void removeStartObserver(ManipLatticeStartObserver* observer);
    void insertGoalObserver(ManipLatticeGoalObserver* observer);
    void removeGoalObserver(ManipLatticeGoalObserver* observer);
    ///@}

    virtual void getExpandedStates(
        std::vector<std::vector<double>>& ara_states) const;

    /// \name Path Extraction
    ///@{
    virtual bool extractPath(
        const std::vector<int>& idpath,
        std::vector<RobotState>& path);
    ///@}

    /// \name Reimplemented Public Functions
    ///@{
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID) override;
    virtual int GetGoalHeuristic(int stateID) override;
    virtual int GetStartHeuristic(int stateID) override;
    virtual void GetSuccs(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV) override;
    virtual void GetLazySuccs(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV,
        std::vector<bool>* isTrueCost) override;
    virtual int GetTrueCost(int parentID, int childID) override;
    virtual void PrintState(
        int stateID, bool bVerbose, FILE* fOut = NULL) override;
    virtual void GetPreds(
        int TargetStateID,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV) override;
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

    std::vector<ManipLatticeStartObserver*> m_start_observers;
    std::vector<ManipLatticeGoalObserver*> m_goal_observers;

    // Context Interfaces
    OccupancyGrid* m_grid;
    RobotModel* m_robot;
    CollisionChecker* m_cc;
    ActionSet* m_as;

    ForwardKinematicsInterface* m_fk_iface;

    PlanningParams* m_params;

    ManipHeuristic* m_heur;

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
    int m_HashTableSize;
    std::vector<ManipLatticeState*>* m_Coord2StateIDHashTable;

    // maps from stateID to coords
    std::vector<ManipLatticeState*> m_states;

    // stateIDs of expanded states
    std::vector<int> m_expanded_states;

    ros::NodeHandle nh_;
    ros::Publisher m_vpub;

    bool m_initialized;

    virtual bool StateID2Angles(int stateID, RobotState& angles) const;

    /** hash table */
    unsigned int intHash(unsigned int key);
    unsigned int getHashBin(const std::vector<int>& coord);
    virtual ManipLatticeState* getHashEntry(const std::vector<int>& coord);
    virtual ManipLatticeState* createHashEntry(
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
    virtual bool isGoal(
        const std::vector<double>& angles,
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

    // NOTE: const although CollisionChecker used underneath may not be
    bool checkAction(
        const RobotState& state,
        const Action& action,
        double& dist) const;
    ///@}

    /// \name output
    ///@{
    void printHashTableHist();
    void printJointArray(
        FILE* fOut,
        ManipLatticeState* HashEntry,
        bool bVerbose);
    ///@}

    void visualizeState(
        const std::vector<double>& jvals,
        const std::string& ns) const;
};

} // namespace manip
} // namespace sbpl

#endif
