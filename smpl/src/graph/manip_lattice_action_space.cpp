////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/graph/manip_lattice_action_space.h>

// standard includes
#include <limits>

// system includes
#include <leatherman/print.h>

// project includes
#include <smpl/angles.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/robot_heuristic.h>

#define VERIFY_KINEMATICS 0

namespace sbpl {
namespace motion {

// Action Set File Format

// Motion_Primitives(degrees): <i actions> <j planning joint variables> <k short distance motion primitives>
// dv11         dv12        ... dv1m
// ...
// dv(i-k)1     dv(i-k)2    ... dv(i-k)m
// dv(i-k+1)1   dv(i-k+1)2  ... dv(i-k+1)m
// ...
// dvi1         dvi2        ... dvim

bool ManipLatticeActionSpace::load(const std::string& action_filename)
{
    FILE* fCfg = NULL;
    if ((fCfg = fopen(action_filename.c_str(), "r")) == NULL) {
        ROS_ERROR("Failed to open action set file. (file: '%s')", action_filename.c_str());
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
                mprim[j] = angles::to_radians(atof(sTemp));
                ROS_DEBUG("Got %s deg -> %.3f rad", sTemp, mprim[j]);
            }
            else {
                ROS_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }
        }
        if (i < (nrows - short_mprims)) {
            addMotionPrim(mprim, false);
        }
        else {
            addMotionPrim(mprim, true);
            have_short_dist_mprims = true;
        }
    }

    if (have_short_dist_mprims) {
        useAmp(MotionPrimitive::SHORT_DISTANCE, true);
    }

    return true;
}

const double ManipLatticeActionSpace::DefaultAmpThreshold = 0.2;

ManipLatticeActionSpace::ManipLatticeActionSpace(const RobotPlanningSpacePtr& pspace) :
    ActionSpace(pspace),
    mp_(),
    m_mprim_enabled(),
    m_mprim_thresh(),
    use_multiple_ik_solutions_(false)
{
    clear();

    RobotModel* robot = planningSpace()->robot();

    m_fk_iface = robot->getExtension<ForwardKinematicsInterface>();
    m_ik_iface = robot->getExtension<InverseKinematicsInterface>();

    if (!m_fk_iface || !m_ik_iface) {
        ROS_WARN("Action Set requires Forward and Inverse Kinematics Interfaces");
    }

    readParameters(*pspace->params());
}

ManipLatticeActionSpace::~ManipLatticeActionSpace()
{
}

void ManipLatticeActionSpace::addMotionPrim(
    const std::vector<double>& mprim,
    bool short_dist_mprim,
    bool add_converse)
{
    MotionPrimitive m;

    if (short_dist_mprim) {
        m.type = MotionPrimitive::SHORT_DISTANCE;
    }
    else {
        m.type = MotionPrimitive::LONG_DISTANCE;
    }

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
        mp_.push_back(m);
    }
}

void ManipLatticeActionSpace::clear()
{
    mp_.clear();

    // add all amps to the motion primitive set
    MotionPrimitive mprim;

    mprim.type = MotionPrimitive::SNAP_TO_RPY;
    mprim.action.clear();
    mp_.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ;
    mprim.action.clear();
    mp_.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ_RPY;
    mprim.action.clear();
    mp_.push_back(mprim);

    // disable all motion primitives except long distance
    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
        m_mprim_enabled[i] = false;
        m_mprim_thresh[i] = DefaultAmpThreshold;
    }
}

int ManipLatticeActionSpace::longDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::LONG_DISTANCE ? 1 : 0);
            });
}

int ManipLatticeActionSpace::shortDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::SHORT_DISTANCE ? 1 : 0);
            });
}

bool ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type) const
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

bool ManipLatticeActionSpace::useMultipleIkSolutions() const
{
    return use_multiple_ik_solutions_;
}

double ManipLatticeActionSpace::ampThresh(MotionPrimitive::Type type) const
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

void ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type, bool enable)
{
    if (type >= 0 && type < MotionPrimitive::NUMBER_OF_MPRIM_TYPES) {
        m_mprim_enabled[type] = enable;
    }
}

void ManipLatticeActionSpace::useMultipleIkSolutions(bool enable)
{
    use_multiple_ik_solutions_ = enable;
}

void ManipLatticeActionSpace::ampThresh(MotionPrimitive::Type type, double thresh)
{
    if (type >= 0 && type < MotionPrimitive::NUMBER_OF_MPRIM_TYPES) {
        m_mprim_thresh[type] = thresh;
    }
}

void ManipLatticeActionSpace::print() const
{
    for (size_t i = 0; i < mp_.size(); ++i) {
        mp_[i].print();
    }
}

void ManipLatticeActionSpace::updateStart(const RobotState& start)
{
    RobotPlanningSpaceObserver::updateStart(start);
}

void ManipLatticeActionSpace::updateGoal(const GoalConstraint& goal)
{
    RobotPlanningSpaceObserver::updateGoal(goal);
}

bool ManipLatticeActionSpace::apply(
    const RobotState& parent,
    std::vector<Action>& actions)
{
    std::vector<double> pose;
    if (!m_fk_iface->computePlanningLinkFK(parent, pose)) {
        ROS_ERROR("Failed to compute forward kinematics for planning link");
        return false;
    }

    // get distance to the goal pose
    double goal_dist = 0.0;
    double start_dist = 0.0;
    if (planningSpace()->numHeuristics() > 0) {
        RobotHeuristicPtr h = planningSpace()->heuristic(0);
        goal_dist = h->getMetricGoalDistance(pose[0], pose[1], pose[2]);
        start_dist = h->getMetricStartDistance(pose[0], pose[1], pose[2]);
    }

    std::vector<Action> act;
    for (size_t i = 0; i < mp_.size(); ++i) {
        const MotionPrimitive& prim = mp_[i];
        if (getAction(parent, goal_dist, start_dist, prim, act)) {
            actions.insert(actions.end(), act.begin(), act.end());
        }
    }

    if (actions.empty()) {
        ROS_WARN_ONCE("No motion primitives specified");
    }

    return true;
}

bool ManipLatticeActionSpace::getAction(
    const RobotState& parent,
    double goal_dist,
    double start_dist,
    const MotionPrimitive& mp,
    std::vector<Action>& actions)
{
    if (!mprimActive(start_dist, goal_dist, mp.type)) {
        return false;
    }

    GoalType goal_type = planningSpace()->goal().type;
    const std::vector<double>& goal_pose = planningSpace()->goal().pose;

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
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_XYZ,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ:
    {
        return computeIkAction(
                parent,
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_RPY,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ_RPY:
    {
        if (planningSpace()->goal().type != GoalType::JOINT_STATE_GOAL) {
            return computeIkAction(
                    parent,
                    goal_pose,
                    goal_dist,
                    ik_option::UNRESTRICTED,
                    actions);
        }
        else {
            // goal is 7dof; instead of computing  IK, use the goal itself as
            // the IK solution
            actions.resize(1);
            actions[0].resize(1);
            actions[0][0] = planningSpace()->goal().angles;
        }

        return true;
    }
    default:
        ROS_ERROR("Motion Primitives of type '%d' are not supported.", mp.type);
        return false;
    }
}

bool ManipLatticeActionSpace::readParameters(const PlanningParams& p)
{
    useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ, p.use_xyz_snap_mprim);
    useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_RPY, p.use_rpy_snap_mprim);
    useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY, p.use_xyzrpy_snap_mprim);
    useAmp(sbpl::motion::MotionPrimitive::SHORT_DISTANCE, p.use_short_dist_mprims);
    ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ, p.xyz_snap_thresh);
    ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_RPY, p.rpy_snap_thresh);
    ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY, p.xyzrpy_snap_thresh);
    ampThresh(sbpl::motion::MotionPrimitive::SHORT_DISTANCE, p.short_dist_mprims_thresh);
    return true;
}

bool ManipLatticeActionSpace::applyMotionPrimitive(
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
            action[i][j] = action[i][j] + state[j];
        }
    }
    return true;
}

bool ManipLatticeActionSpace::computeIkAction(
    const RobotState& state,
    const std::vector<double>& goal,
    double dist_to_goal,
    ik_option::IkOption option,
    std::vector<Action>& actions)
{
    if (use_multiple_ik_solutions_) {
        //get actions for multiple ik solutions
        std::vector<std::vector<double>> solutions;
        if (!m_ik_iface->computeIK(goal, state, solutions, option)) {
            ROS_DEBUG("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)",
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
        if (!m_ik_iface->computeIK(goal, state, ik_sol)) {
            ROS_DEBUG("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)", to_string(option).c_str(), dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
            return false;
        }
        actions.resize(1);
        actions[0].resize(1);
        actions[0][0] = ik_sol;
    }

    return true;
}

bool ManipLatticeActionSpace::mprimActive(
    double start_dist,
    double goal_dist,
    MotionPrimitive::Type type) const
{
    if (type == MotionPrimitive::LONG_DISTANCE) {
        const bool near_endpoint =
                (goal_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE] ||
                start_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE]);
        return !(m_mprim_enabled[MotionPrimitive::SHORT_DISTANCE] && near_endpoint);
    }
    else if (type == MotionPrimitive::SHORT_DISTANCE) {
        const bool near_endpoint =
                (goal_dist <= m_mprim_thresh[type] ||
                start_dist <= m_mprim_thresh[type]);
        return m_mprim_enabled[type] && near_endpoint;
    }
    else {
        return m_mprim_enabled[type] && goal_dist <= m_mprim_thresh[type];
    }
}

} // namespace motion
} // namespace sbpl
