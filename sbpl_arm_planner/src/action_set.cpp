////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Maxim Likhachev
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

 /// \author Benjamin Cohen

#include <limits>

#include <leatherman/print.h>
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/environment_robarm3d.h>

#define VERIFY_KINEMATICS 0

namespace sbpl_arm_planner {

bool ActionSet::Load(const std::string& action_file, ActionSet& action_set)
{
    ActionSet as;

    FILE* fCfg = NULL;
    if ((fCfg = fopen(action_file.c_str(), "r")) == NULL) {
        ROS_ERROR("Failed to open action set file. (file: '%s')", action_file.c_str());
        return false;
    }

    char sTemp[1024];
    int nrows = 0, ncols = 0, short_mprims = 0;

    if (fCfg == NULL) {
        ROS_ERROR("unable to open the params file. Exiting.");
        return false;
    }

    if (fscanf(fCfg, "%s", sTemp) < 1) {
        ROS_WARN("Parsed string has length < 1.");
    }

    if (strcmp(sTemp, "Motion_Primitives(degrees):") != 0) {
        ROS_ERROR("First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
        return false;
    }

    // number of actions
    if (fscanf(fCfg, "%s", sTemp) < 1) {
        ROS_WARN("Parsed string has length < 1.");
        return false;
    }
    else {
        nrows = atoi(sTemp);
    }

    // length of joint array
    if (fscanf(fCfg, "%s", sTemp) < 1) {
        ROS_WARN("Parsed string has length < 1.");
        return false;
    }
    else {
        ncols = atoi(sTemp);
    }

    // number of short distance motion primitives
    if (fscanf(fCfg, "%s", sTemp) < 1) {
        ROS_WARN("Parsed string has length < 1.");
        return false;
    }
    else {
        short_mprims = atoi(sTemp);
    }

    if (short_mprims == nrows) {
        ROS_ERROR("# of motion prims == # of short distance motion prims. No long distance motion prims set.");
    }

    std::vector<double> mprim(ncols, 0);

    bool have_short_dist_mprims = false;
    for (int i = 0; i < nrows; ++i) {
        for (int j = 0; j < ncols; ++j) {
            if (fscanf(fCfg, "%s", sTemp) < 1)  {
                ROS_WARN("Parsed string has length < 1.");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                mprim[j] = 2.0 * angles::from_degrees(atof(sTemp));
                ROS_DEBUG("Got %s deg -> %.3f rad", sTemp, mprim[j]);
            }
            else {
                ROS_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }
        }
        if (i < (nrows - short_mprims)) {
            as.addMotionPrim(mprim, false);
        }
        else {
            as.addMotionPrim(mprim, true);
            have_short_dist_mprims = true;
        }
    }

    if (have_short_dist_mprims) {
        as.useAmp(MotionPrimitive::SHORT_DISTANCE);
    }

    action_set = as;
    return true;
}

const double ActionSet::DefaultAmpThreshold = 0.2;

ActionSet::ActionSet() :
    mp_(),
    m_mprim_enabled(),
    m_mprim_thresh(),
    use_multiple_ik_solutions_(false)
{
    clear();
}

bool ActionSet::init(EnvironmentROBARM3D* env, bool use_multiple_ik_solutions)
{
    if (!env) {
        return false;
    }

    env_ = env;
    use_multiple_ik_solutions_ = use_multiple_ik_solutions;
    return true;
}

void ActionSet::addMotionPrim(
    const std::vector<double>& mprim,
    bool short_dist_mprim,
    bool add_converse)
{
    MotionPrimitive m;

    if (short_dist_mprim) {
        m.type = sbpl_arm_planner::MotionPrimitive::SHORT_DISTANCE;
    }
    else {
        m.type = sbpl_arm_planner::MotionPrimitive::LONG_DISTANCE;
    }

    m.id =  mp_.size();
    m.action.push_back(mprim);
    mp_.push_back(m);

    if (add_converse) {
        Action a;
        a.resize(1);
        a[0] = mprim;
        for (int i = 0; i < int(mprim.size()); ++i) {
            if (mprim[i] != 0) {
                a[0][i] *= -1;
            }
        }
        m.action = a;
        m.id =  mp_.size();
        mp_.push_back(m);
    }
}

void ActionSet::clear()
{
    mp_.clear();

    // add all amps to the motion primitive set
    MotionPrimitive mprim;

    mprim.type = MotionPrimitive::SNAP_TO_RPY;
    mprim.id = 0;
    mprim.action.clear();
    mp_.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ;
    mprim.id = 1;
    mprim.action.clear();
    mp_.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ_RPY;
    mprim.id = 2;
    mprim.action.clear();
    mp_.push_back(mprim);

    // disable all motion primitives except long distance
    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
        m_mprim_enabled[i] = false;
        m_mprim_thresh[i] = DefaultAmpThreshold;
    }
}

int ActionSet::longDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::LONG_DISTANCE ? 1 : 0);
            });
}

int ActionSet::shortDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::SHORT_DISTANCE ? 1 : 0);
            });
}

bool ActionSet::useAmp(MotionPrimitive::Type type) const
{
    if (type == MotionPrimitive::LONG_DISTANCE) {
        return true;
    }
    else if (type >= 0 && type < MotionPrimitive::NUMBER_OF_MPRIM_TYPES) {
        return m_mprim_enabled[type];
    }
    else {
        return false;
    }
}

bool ActionSet::useMultipleIkSolutions() const
{
    return use_multiple_ik_solutions_;
}

double ActionSet::ampThresh(MotionPrimitive::Type type) const
{
    if (type == MotionPrimitive::LONG_DISTANCE) {
        return std::numeric_limits<double>::infinity();
    }
    else if (type >= 0 && type < MotionPrimitive::NUMBER_OF_MPRIM_TYPES) {
        return m_mprim_thresh[type];
    }
    else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

void ActionSet::useAmp(MotionPrimitive::Type type, bool enable)
{
    if (type >= 0 && type < MotionPrimitive::NUMBER_OF_MPRIM_TYPES) {
        m_mprim_enabled[type] = enable;
    }
}

void ActionSet::useMultipleIkSolutions(bool enable)
{
    use_multiple_ik_solutions_ = enable;
}

void ActionSet::ampThresh(MotionPrimitive::Type type, double thresh)
{
    if (type >= 0 && type < MotionPrimitive::NUMBER_OF_MPRIM_TYPES) {
        m_mprim_thresh[type] = thresh;
    }
}

void ActionSet::print() const
{
    for (size_t i = 0; i < mp_.size(); ++i) {
        mp_[i].print();
    }
}

bool ActionSet::getActionSet(
    const RobotState& parent,
    std::vector<Action>& actions)
{
    std::vector<double> pose;
    if (!env_->getRobotModel()->computePlanningLinkFK(parent, pose)) {
        ROS_ERROR("Failed to compute forward kinematics for planning link");
        return false;
    }

    // get distance to the goal pose
    const double d = env_->getDistanceToGoal(pose[0], pose[1], pose[2]);

    std::vector<Action> act;
    for (size_t i = 0; i < mp_.size(); ++i) {
        const MotionPrimitive& prim = mp_[i];
        if (getAction(parent, d, prim, act)) {
            actions.insert(actions.end(), act.begin(), act.end());
        }
    }

    if (actions.empty()) {
        ROS_WARN_ONCE("No motion primitives specified");
    }

    return true;
}

bool ActionSet::getAction(
    const RobotState& parent,
    double dist_to_goal,
    const MotionPrimitive& mp,
    std::vector<Action>& actions)
{
    std::vector<double> goal = env_->getGoal();

    if (!mprimActive(dist_to_goal, mp.type)) {
        return false;
    }

    switch (mp.type) {
    case MotionPrimitive::LONG_DISTANCE:
    {
        actions.resize(1);
        return applyMotionPrimitive(parent, mp, actions[0]);
    }
    case MotionPrimitive::SHORT_DISTANCE:
    {
        actions.resize(1);
        return applyMotionPrimitive(parent, mp, actions[0]);
    }
    case MotionPrimitive::SNAP_TO_RPY:
    {
        return computeIkAction(
                parent,
                goal,
                dist_to_goal,
                ik_option::RESTRICT_XYZ_JOINTS,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ:
    {
        // TODO: implement?
        return false;
    }
    case MotionPrimitive::SNAP_TO_XYZ_RPY:
    {
        if (!env_->use7DOFGoal()) {
            return computeIkAction(
                    parent,
                    goal,
                    dist_to_goal,
                    ik_option::UNRESTRICTED,
                    actions);
        }
        else {
            // goal is 7dof; instead of computing  IK, use the goal itself as
            // the IK solution
            actions.resize(1);
            actions[0].resize(1);
            actions[0][0] = env_->getGoalConfiguration();
        }

        return true;
    }
    default:
        ROS_ERROR("Motion Primitives of type '%d' are not supported.", mp.type);
        return false;
    }
}

bool ActionSet::applyMotionPrimitive(
    const RobotState& state,
    const MotionPrimitive& mp,
    Action& action)
{
    action = mp.action;
    for (size_t i = 0; i < action.size(); ++i) {
        if (action[i].size() != state.size()) {
            return false;
        }

        for (size_t j = 0; j < action[i].size(); ++j) {
            action[i][j] = angles::normalize_angle(action[i][j] + state[j]);
        }
    }
    return true;
}

bool ActionSet::computeIkAction(
    const RobotState& state,
    const std::vector<double>& goal,
    double dist_to_goal,
    sbpl_arm_planner::ik_option::IkOption option,
    std::vector<Action>& actions)
{
    if (use_multiple_ik_solutions_) {
        //get actions for multiple ik solutions
        std::vector<std::vector<double>> solutions;
        if (!env_->getRobotModel()->computeIK(goal, state, solutions, option)) {
            ROS_WARN("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)",
                    to_string(option).c_str(), dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
            return false;
        }
        actions.resize(solutions.size());
        for (size_t a = 0; a < actions.size(); a++){
            actions[a].resize(1);
            actions[a][0] = solutions[a];
        }
    }
    else {
        //get single action for single ik solution
        std::vector<double> ik_sol;
        if (!env_->getRobotModel()->computeIK(goal, state, ik_sol)) {
            ROS_WARN("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)", to_string(option).c_str(), dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
            return false;
        }
        actions.resize(1);
        actions[0].resize(1);
        actions[0][0] = ik_sol;
    }

    return true;
}

bool ActionSet::mprimActive(
    double dist_to_goal,
    MotionPrimitive::Type type) const
{
    if (type == MotionPrimitive::LONG_DISTANCE) {
        return !(m_mprim_enabled[MotionPrimitive::SHORT_DISTANCE] &&
            dist_to_goal <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE]);
    }
    else {
        return m_mprim_enabled[type] && dist_to_goal < m_mprim_thresh[type];
    }
}

} // namespace sbpl_arm_planner
