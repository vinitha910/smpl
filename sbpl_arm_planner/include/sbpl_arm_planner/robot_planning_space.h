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

#ifndef sbpl_manip_robot_planning_space_h
#define sbpl_manip_robot_planning_space_h

// standard includes
#include <vector>

// system includes
#include <Eigen/Dense>
#include <sbpl/discrete_space_information/environment.h>

// project includes
#include <sbpl_arm_planner/action_space.h>
#include <sbpl_arm_planner/collision_checker.h>
#include <sbpl_arm_planner/forward.h>
#include <sbpl_arm_planner/robot_planning_space_observer.h>
#include <sbpl_arm_planner/planning_params.h>
#include <sbpl_arm_planner/robot_model.h>
#include <sbpl_arm_planner/types.h>

namespace sbpl {
namespace manip {

SBPL_CLASS_FORWARD(RobotHeuristic);
SBPL_CLASS_FORWARD(RobotPlanningSpace);

class RobotPlanningSpace :
    public DiscreteSpaceInformation,
    public virtual Extension
{
public:

    RobotPlanningSpace(
        RobotModel* robot,
        CollisionChecker* checker,
        const PlanningParams* params);

    virtual ~RobotPlanningSpace();

    virtual bool setStart(const RobotState& state);
    virtual bool setGoal(const GoalConstraint& goal);

    virtual int getStartStateID() const = 0;
    virtual int getGoalStateID() const = 0;

    virtual bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) = 0;

    virtual bool setActionSpace(const ActionSpacePtr& actions);

    virtual bool insertHeuristic(const RobotHeuristicPtr& h);
    virtual bool eraseHeuristic(const RobotHeuristicPtr& h);
    virtual bool hasHeuristic(const RobotHeuristicPtr& h);

    RobotModel* robot() { return m_robot; }
    const RobotModel* robot() const { return m_robot; }

    CollisionChecker* collisionChecker() { return m_checker; }
    const CollisionChecker* collisionChecker() const { return m_checker; }

    const PlanningParams* params() const { return m_params; }

    ActionSpacePtr actionSpace() { return m_actions; }
    ActionSpaceConstPtr actionSpace() const { return m_actions; }

    const RobotState& startState() const { return m_start; }
    const GoalConstraint& goal() const { return m_goal; }

    size_t numHeuristics() const;
    RobotHeuristicPtr heuristic(size_t i);
    RobotHeuristicConstPtr heuristic(size_t i) const;

    void insertObserver(RobotPlanningSpaceObserver* obs);
    void eraseObserver(RobotPlanningSpaceObserver* obs);
    bool hasObserver(RobotPlanningSpaceObserver* obs) const;

    void notifyStartChanged(const RobotState& state);
    void notifyGoalChanged(const GoalConstraint& goal);

    /// \name Reimplemented Public Functions from DiscreteSpaceInformation
    ///@{
    virtual int GetGoalHeuristic(int state_id) override;
    virtual int GetStartHeuristic(int state_id) override;
    virtual int GetFromToHeuristic(int from_id, int to_id) override;

    virtual void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;

    virtual int GetTrueCost(int parentID, int childID) override;
    ///@}

    /// \name Restate Required Public Functions from DiscreteSpaceInformation
    ///@{
    virtual void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override = 0;

    virtual void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override = 0;

    virtual void PrintState(
        int state_id,
        bool verbose,
        FILE* f = nullptr) override = 0;
    ///@}

private:

    RobotModel* m_robot;
    CollisionChecker* m_checker;
    const PlanningParams* m_params;

    ActionSpacePtr m_actions;

    RobotState m_start;
    GoalConstraint m_goal;

    std::vector<RobotHeuristicPtr> m_heuristics;

    std::vector<RobotPlanningSpaceObserver*> m_obs;

    // Make all attempts to hide the set of useless functions from
    // DiscreteSpaceInformation
    virtual bool InitializeEnv(const char*) final { return false; }
    virtual bool InitializeMDPCfg(MDPConfig*) final { return false; }
    virtual int SizeofCreatedEnv() final { return 0; }
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE*) final { }
    virtual void SetAllPreds(CMDPSTATE*) final { }
    virtual void PrintEnv_Config(FILE*) final { }
};

class PointProjectionExtension : public virtual Extension
{
public:

    virtual ~PointProjectionExtension() { }

    virtual bool projectToPoint(int state_id, Eigen::Vector3d& pos) = 0;
};

inline
size_t RobotPlanningSpace::numHeuristics() const
{
    return m_heuristics.size();
}

inline
RobotHeuristicPtr RobotPlanningSpace::heuristic(size_t i)
{
    if (i >= m_heuristics.size()) {
        return RobotHeuristicPtr();
    }
    return m_heuristics[i];
}

inline
RobotHeuristicConstPtr RobotPlanningSpace::heuristic(size_t i) const
{
    if (i >= m_heuristics.size()) {
        return RobotHeuristicConstPtr();
    }
    return m_heuristics[i];
}

} // namespace manip
} // namespace sbpl

#endif
