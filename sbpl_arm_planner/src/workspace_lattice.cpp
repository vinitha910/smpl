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

#include <sbpl_arm_planner/workspace_lattice.h>

// system includes
#include <boost/functional/hash.hpp>
#include <leatherman/print.h>

// project includes
#include <sbpl_arm_planner/angles.h>

auto std::hash<sbpl::manip::WorkspaceLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace sbpl {
namespace manip {

template <
    class InputIt,
    class Equal = std::equal_to<typename std::iterator_traits<InputIt>::value_type>>
bool all_equal(InputIt first, InputIt last, typename std::iterator_traits<InputIt>::reference val)
{
    typedef typename std::iterator_traits<InputIt>::reference reference;
    return std::all_of(
            first, last,
            [&val](reference a) { return Equal()(a, val); });
}

WorkspaceLattice::WorkspaceLattice(
    OccupancyGrid* grid,
    CollisionChecker* cc,
    PlanningParams* params)
:
    m_vpub(),
    m_grid(grid),
    m_robot(nullptr),
    m_cc(cc),
    m_params(params),
    m_goal(),
    m_goal_entry(nullptr),
    m_start_entry(nullptr),
    m_state_to_id(),
    m_states(),
    m_near_goal(false),
    m_fangle_indices(),
    m_dof_count(-1)
{
    m_vpub = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("visualization_markers", 100);
}

WorkspaceLattice::~WorkspaceLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();

    // NOTE: StateID2IndexMapping cleared by DiscreteSpaceInformation
}

bool WorkspaceLattice::init(RobotModel* robot, const Params& params)
{
    assert(robot);
    m_robot = robot;

    m_fk_iface = robot->getExtension<ForwardKinematicsInterface>();
    if (!m_fk_iface) {
        ROS_WARN("Workspace Lattice requires Forward Kinematics Interface extension");
        return false;
    }

    m_ik_iface = robot->getExtension<InverseKinematicsInterface>();
    if (!m_ik_iface) {
        ROS_WARN("Workspace Lattice requires Inverse Kinematics Interface extension");
        return false;
    }

    m_rm_iface = robot->getExtension<RedundantManipulatorInterface>();
    if (!m_rm_iface) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return false;
    }

    m_fangle_indices.resize(m_rm_iface->redundantVariableCount());
    for (size_t i = 0; i < m_fangle_indices.size(); ++i) {
        m_fangle_indices[i] = m_rm_iface->redundantVariableIndex(i);
    }
    m_dof_count = 6 + m_fangle_indices.size();

    m_res.resize(m_dof_count);

    m_res[0] = m_grid->getResolution();
    m_res[1] = m_grid->getResolution();
    m_res[2] = m_grid->getResolution();
    // TODO: limit these ranges and handle discretization appropriately
    m_res[3] = (2.0 * M_PI) / params.R_count;
    m_res[4] = (2.0 * M_PI) / params.P_count;
    m_res[5] = (2.0 * M_PI) / params.Y_count;

    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        m_res[6 + i] = (2.0 * M_PI) / params.free_angle_res[i];
    }

    m_val_count[0] = std::numeric_limits<int>::max();
    m_val_count[1] = std::numeric_limits<int>::max();
    m_val_count[2] = std::numeric_limits<int>::max();
    m_val_count[3] = params.R_count;
    m_val_count[4] = params.P_count;
    m_val_count[5] = params.Y_count;
    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        m_val_count[6 + i] = (2.0 * M_PI) / params.free_angle_res[i];
    }

    ROS_INFO("discretization of workspace lattice:");
    ROS_INFO("  x: { res: %0.3f, count: %d }", m_res[0], m_val_count[0]);
    ROS_INFO("  y: { res: %0.3f, count: %d }", m_res[1], m_val_count[1]);
    ROS_INFO("  z: { res: %0.3f, count: %d }", m_res[2], m_val_count[2]);
    ROS_INFO("  R: { res: %0.3f, count: %d }", m_res[3], m_val_count[3]);
    ROS_INFO("  P: { res: %0.3f, count: %d }", m_res[4], m_val_count[4]);
    ROS_INFO("  Y: { res: %0.3f, count: %d }", m_res[5], m_val_count[5]);
    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        ROS_INFO("  J%d: { res: %0.3f, count: %d }", i, m_res[6 + i], m_val_count[6 + i]);
    }

    ROS_DEBUG_NAMED(m_params->graph_log_, "initialize environment");
    m_start_entry = nullptr;

    // this should serve as a reasonable dummy state since no valid state should
    // have an empty coordinate vector
    WorkspaceCoord fake_coord;
    m_goal_state_id = createState(fake_coord);
    m_goal_entry = getState(m_goal_state_id);
    ROS_DEBUG_NAMED(m_params->graph_log_, "  goal state has id %d", m_goal_state_id);

    m_prims.clear();

    MotionPrimitive prim;

    // create 26-connected position motions
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }

                std::vector<double> d(m_dof_count, 0.0);
                d[0] = m_res[0] * dx;
                d[1] = m_res[1] * dy;
                d[2] = m_res[2] * dz;
                prim.type = MotionPrimitive::Type::LONG_DISTANCE;
                prim.action.clear();
                prim.action.push_back(std::move(d));

                m_prims.push_back(prim);
            }
        }
    }

    // create 2-connected motions for rotation and free angle motions
    for (int a = 3; a < m_dof_count; ++a) {
        std::vector<double> d(m_dof_count, 0.0);

        d[a] = m_res[a] * -1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        m_prims.push_back(prim);

        d[a] = m_res[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        m_prims.push_back(prim);
    }
}

bool WorkspaceLattice::initialized() const
{
    return m_goal_entry; // sufficient for the moment
}

bool WorkspaceLattice::setStartState(const RobotState& state)
{
    if (!initialized()) {
        ROS_ERROR_NAMED(m_params->graph_log_, "cannot set start state on uninitialized lattice");
        return false;
    }

    ROS_DEBUG_NAMED(m_params->graph_log_, "set the start state");
    if (state.size() < m_params->num_joints_) {
        ROS_ERROR_NAMED(m_params->graph_log_, "start state contains insufficient coordinate positions");
        return false;
    }

    ROS_DEBUG_NAMED(m_params->graph_log_, "  angles: %s", to_string(state).c_str());

    if (!m_robot->checkJointLimits(state)) {
        ROS_ERROR_NAMED(m_params->graph_log_, "start state violates joint limits");
        return false;
    }

    double dist;
    if (!m_cc->isStateValid(state, true, false, dist)) {
        ROS_WARN("start state is in collision");
        return false;
    }

    visualizeState(state, "start_config");
    WorkspaceCoord start_coord;
    stateRobotToCoord(state, start_coord);

    m_start_state_id = createState(start_coord);
    m_start_entry = getState(m_start_state_id);
    m_start_entry->state = state;

    return true;
}

bool WorkspaceLattice::setGoalPose(const PoseGoal& goal)
{
    if (!initialized()) {
        ROS_ERROR_NAMED(m_params->graph_log_, "cannot set goal pose on uninitialized lattice");
        return false;
    }

    // TODO/NOTE: 7 for compatibility with ManipLattice goal
    if (goal.pose.size() != 7) {
        ROS_ERROR("goal element has incorrect format");
        return false;
    }

    if (goal.offset.size() != 3) {
        ROS_ERROR("offset element has incorrect format");
        return false;
    }

    if (goal.tolerance.size() != 6) {
        ROS_ERROR("tolerance element has incorrect format");
        return false;
    }

    ROS_DEBUG_NAMED(m_params->graph_log_, "set the goal state");

    // check if an IK solution exists for the goal pose before we do
    // the search we plan even if there is no solution
    RobotState seed(m_params->num_joints_, 0);
    RobotState ik_solution;
    if (!m_ik_iface->computeIK(goal.pose, seed, ik_solution)) {
        ROS_WARN("No valid IK solution for the goal pose.");
    }

    m_goal.pose = goal.pose;
    std::copy(goal.offset.begin(), goal.offset.end(), m_goal.xyz_offset);
    std::copy(goal.tolerance.begin(), goal.tolerance.begin() + 3, m_goal.xyz_tolerance);
    std::copy(goal.tolerance.begin() + 3, goal.tolerance.begin() + 6, m_goal.rpy_tolerance);

    m_goal.type = GoalType::XYZ_RPY_GOAL;

    ROS_DEBUG_NAMED(m_params->graph_log_, "  xyz (meters): (%0.2f, %0.2f, %0.2f)", m_goal.pose[0], m_goal.pose[1], m_goal.pose[2]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "  tol (meters): (%0.3f, %0.3f, %0.3f)", m_goal.xyz_tolerance[0], m_goal.xyz_tolerance[1], m_goal.xyz_tolerance[2]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "  rpy (radians): (%0.2f, %0.2f, %0.2f)", m_goal.pose[3], m_goal.pose[4], m_goal.pose[5]);
    ROS_DEBUG_NAMED(m_params->graph_log_, "  tol (radians): (%0.3f, %0.3f, %0.3f)", m_goal.rpy_tolerance[0], m_goal.rpy_tolerance[1], m_goal.rpy_tolerance[2]);

    m_near_goal = false;
    m_t_start = clock();

    return true;
}

bool WorkspaceLattice::setGoalPoses(const std::vector<PoseGoal>& goals)
{
    return false;
}

int WorkspaceLattice::getStartStateID() const
{
    return m_start_state_id;
}

int WorkspaceLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

bool WorkspaceLattice::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    if (ids.empty()) {
        path.clear();
        return true;
    }

    path.clear();

    WorkspaceLatticeState* start_entry = getState(ids[0]);
    path.push_back(start_entry->state);

    for (size_t i = 1; i < ids.size(); ++i) {
        const int prev_id = ids[i - 1];
        const int curr_id = ids[i];

        if (prev_id == getGoalStateID()) {
            ROS_ERROR_NAMED(m_params->graph_log_, "cannot determine goal state successors during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            // TODO: variant of get succs that returns unique state ids
            WorkspaceLatticeState* prev_entry = getState(prev_id);
            std::vector<Action> actions;
            getActions(*prev_entry, actions);

            WorkspaceLatticeState* best_goal_entry = nullptr;
            int best_cost = std::numeric_limits<int>::max();

            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                const WorkspaceState& final_state = action.back();
                if (!isGoal(final_state)) {
                    continue;
                }

                double dist;
                if (!checkAction(prev_entry->state, action, dist)) {
                    continue;
                }

                WorkspaceCoord goal_coord;
                stateWorkspaceToCoord(final_state, goal_coord);

                int goal_id = createState(goal_coord);
                WorkspaceLatticeState* goal_state = getState(goal_id);

                // shouldn't have created a new state, so no need to set the
                // continuous state counterpart
                assert(goal_state->state.size() == m_params->num_joints_);

                best_cost = 30; // Hardcoded primitive value in GetSuccs
                best_goal_entry = goal_state;
                break;
            }

            if (!best_goal_entry) {
                ROS_ERROR_NAMED(m_params->graph_log_, "failed to find valid goal successor during path extraction");
                return false;
            }

            path.push_back(best_goal_entry->state);
        }
        else {
            WorkspaceLatticeState* state_entry = getState(curr_id);
            path.push_back(state_entry->state);
        }
    }

    return true;
}

int WorkspaceLattice::GetFromToHeuristic(int from_state_id, int to_state_id)
{
    return 0;
}

int WorkspaceLattice::GetGoalHeuristic(int state_id)
{
    int xyz_heur = 0, rpy_heur = 0;
    WorkspaceLatticeState* state = getState(state_id);

    int r = (m_goal_entry->coord[3] - state->coord[3]) % m_val_count[3];
    int p = (m_goal_entry->coord[4] - state->coord[4]) % m_val_count[4];
    int y = (m_goal_entry->coord[5] - state->coord[5]) % m_val_count[5];

    rpy_heur = (abs(r) + abs(p) + abs(y)) * m_params->cost_per_cell_;
    ROS_DEBUG("stateid: %5d   xyz: %2d %2d %2d  rpy: %2d %2d %2d  (goal-rpy: %2d %2d %2d)", state_id, state->coord[0], state->coord[1], state->coord[2], state->coord[3], state->coord[4], state->coord[5], m_goal_entry->coord[3], m_goal_entry->coord[4], m_goal_entry->coord[5]);
    ROS_DEBUG("               xyz-heur: %4d  rpy-heur: %4d  (r: %2d  p:%2d  y: %2d)", xyz_heur, rpy_heur, r, p, y);

    state->h = xyz_heur + rpy_heur;
    return xyz_heur + rpy_heur;
}

int WorkspaceLattice::GetStartHeuristic(int state_id)
{
    return 0;
}

void WorkspaceLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    // clear the successor arrays
    succs->clear();
    costs->clear();

    ROS_DEBUG_NAMED(m_params->expands_log_, "expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    WorkspaceLatticeState* parent_entry = getState(state_id);

    assert(parent_entry);
    assert(parent_entry->coord.size() == m_dof_count);

    ROS_DEBUG_NAMED(m_params->expands_log_, "  coord: %s", to_string(parent_entry->coord).c_str());
    ROS_DEBUG_NAMED(m_params->expands_log_, "  state: %s", to_string(parent_entry->state).c_str());

    visualizeState(parent_entry->state, "expansion");

    std::vector<Action> actions;
    getActions(*parent_entry, actions);

    ROS_DEBUG_NAMED(m_params->expands_log_, "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(m_params->expands_log_, "    action %zu", i);
        ROS_DEBUG_NAMED(m_params->expands_log_, "      waypoints: %zu", action.size());

        double dist;
        if (!checkAction(parent_entry->state, action, dist)) {
            continue;
        }

        const WorkspaceState& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        int succ_id = createState(succ_coord);
        WorkspaceLatticeState* succ_state = getState(succ_id);
        succ_state->state = parent_entry->state;

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(final_state);

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        }
        else {
            succs->push_back(succ_id);
        }

        costs->push_back(30);

        ROS_DEBUG_NAMED(m_params->expands_log_, "      succ: %zu", i);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        id: %5i", succ_id);
        ROS_DEBUG_NAMED(m_params->expands_log_, "        coord: %s", to_string(succ_state->coord).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        state: %s", to_string(succ_state->state).c_str());
        ROS_DEBUG_NAMED(m_params->expands_log_, "        cost: %5d", 30);
    }
}

void WorkspaceLattice::PrintState(
    int state_id,
    bool verbose,
    FILE* fout)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    if (!fout) {
        fout = stdout;
    }

    WorkspaceLatticeState* state = getState(state_id);

    std::stringstream ss;
    ss << "{ coord: " << state->coord << ", state: " << state->state << " }";

    if (fout == stdout) {
        ROS_DEBUG_NAMED(m_params->graph_log_, "%s", ss.str().c_str());
    }
    else if (fout == stderr) {
        ROS_WARN_NAMED(m_params->graph_log_, "%s", ss.str().c_str());
    }
    else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

/// \brief Create a state entry for a given coordinate and return its id
///
/// If an entry already exists for the coordinate, the id corresponding to that
/// entry is returned; otherwise, a new entry is created and its id returned.
int WorkspaceLattice::createState(const WorkspaceCoord& coord)
{
    WorkspaceLatticeState state;
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit != m_state_to_id.end()) {
        return sit->second;
    }

    int new_id = (int)m_states.size();

    // create a new entry
    WorkspaceLatticeState* state_entry = new WorkspaceLatticeState(state);

    // map id <-> state
    m_states.push_back(state_entry);
    m_state_to_id[state_entry] = new_id;

    int* indices = new int[NUMOFINDICES_STATEID2IND];
    std::fill(indices, indices + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(indices);

    return new_id;
}

/// \brief Retrieve a state by its id.
///
/// The id is not checked for validity and the state is assumed to have already
/// been created, either by GetSuccs during a search or by designating a new
/// start or goal state.
WorkspaceLatticeState* WorkspaceLattice::getState(int state_id)
{
    assert(state_id >= 0 && state_id < m_states.size());
    return m_states[state_id];
}

void WorkspaceLattice::stateRobotToWorkspace(
    const RobotState& state,
    WorkspaceState& ostate)
{
    SixPose pose;
    bool res = m_fk_iface->computePlanningLinkFK(state, pose);
    assert(res); // forward kinematics shouldn't fail

    ostate.resize(m_dof_count);
    std::copy(pose.begin(), pose.end(), ostate.begin());
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        ostate[6 + fai] = state[m_fangle_indices[fai]];
    }
}

void WorkspaceLattice::stateRobotToCoord(
    const RobotState& state,
    WorkspaceCoord& coord)
{
    WorkspaceState ws_state;
    stateRobotToWorkspace(state, ws_state);
    stateWorkspaceToCoord(ws_state, coord);
}

bool WorkspaceLattice::stateWorkspaceToRobot(
    const WorkspaceState& state,
    RobotState& ostate)
{
    SixPose pose(state.begin(), state.begin() + 6);

    RobotState seed(m_params->num_joints_, 0);
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        seed[m_fangle_indices[fai]] = state[6 + fai];
    }

    // TODO: unrestricted variant?
    return m_rm_iface->computeFastIK(pose, seed, ostate);
}

void WorkspaceLattice::stateWorkspaceToCoord(
    const WorkspaceState& state,
    WorkspaceCoord& coord)
{
    coord.resize(m_dof_count);
    posWorkspaceToCoord(&state[0], &coord[0]);
    rotWorkspaceToCoord(&state[3], &coord[3]);
    favWorkspaceToCoord(&state[6], &coord[6]);
}

bool WorkspaceLattice::stateCoordToRobot(
    const WorkspaceCoord& coord,
    RobotState& state)
{

}

void WorkspaceLattice::stateCoordToWorkspace(
    const WorkspaceCoord& coord,
    WorkspaceState& state)
{

}

/// \brief Convert a discrete position to its continuous counterpart.
void WorkspaceLattice::posWorkspaceToCoord(const double* wp, int* gp)
{
    m_grid->worldToGrid(wp[0], wp[1], wp[2], gp[0], gp[1], gp[2]);
}

/// \brief Convert a continuous position to its discrete counterpart.
void WorkspaceLattice::posCoordToWorkspace(const int* gp, double* wp)
{
    m_grid->gridToWorld(gp[0], gp[1], gp[2], wp[0], wp[1], wp[2]);
}

/// \brief Convert a continuous rotation to its continuous counterpart.
void WorkspaceLattice::rotWorkspaceToCoord(const double* wr, int* gr)
{
    gr[0] = (int)((angles::normalize_angle_positive(wr[0]) + m_res[3] * 0.5) / m_res[3]) % m_val_count[3];
    gr[1] = (int)((angles::normalize_angle_positive(wr[1]) + m_res[4] * 0.5) / m_res[4]) % m_val_count[4];
    gr[2] = (int)((angles::normalize_angle_positive(wr[2]) + m_res[5] * 0.5) / m_res[5]) % m_val_count[5];
}

/// \brief Convert a discrete rotation to its continuous counterpart.
void WorkspaceLattice::rotCoordToWorkspace(const int* gr, double* wr)
{
    wr[0] = angles::normalize_angle((double)gr[0] * m_res[3]);
    wr[1] = angles::normalize_angle((double)gr[1] * m_res[4]);
    wr[2] = angles::normalize_angle((double)gr[2] * m_res[5]);
}

/// \brief Convert a continuous pose to its discrete counterpart.
void WorkspaceLattice::poseWorkspaceToCoord(const double* wp, int* gp)
{
    posWorkspaceToCoord(wp, gp);
    rotWorkspaceToCoord(wp + 3, gp + 3);
}

/// \brief Convert a discrete pose to its continuous counterpart.
void WorkspaceLattice::poseCoordToWorkspace(const int* gp, double* wp)
{
    posCoordToWorkspace(gp, wp);
    rotCoordToWorkspace(gp + 3, wp + 3);
}

/// \brief Convert a continuous free angle vector to its discrete counterpart.
void WorkspaceLattice::favWorkspaceToCoord(const double* wa, int* ga)
{
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        ga[fai] = (int)((angles::normalize_angle_positive(wa[fai]) + m_res[6 + fai] * 0.5) / m_res[6 + fai]) % m_val_count[6 + fai];
    }
}

/// \brief Convert a discrete free angle vector to its continuous counterpart.
void WorkspaceLattice::favCoordToWorkspace(const int* ga, double* wa)
{
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        wa[fai] = angles::normalize_angle((double)ga[fai] * m_res[6 + fai]);
    }
}

bool WorkspaceLattice::isGoal(const WorkspaceState& state)
{
    // check position
    if (fabs(state[0] - m_goal.pose[0]) <= m_goal.xyz_tolerance[0] &&
        fabs(state[1] - m_goal.pose[1]) <= m_goal.xyz_tolerance[1] &&
        fabs(state[2] - m_goal.pose[2]) <= m_goal.xyz_tolerance[2])
    {
        // log the amount of time required for the search to get close to the goal
        if (!m_near_goal) {
            double time_to_goal_region = (clock() - m_t_start) / (double)CLOCKS_PER_SEC;
            m_near_goal = true;
            ROS_INFO("search is at the goal position after %0.3f sec", time_to_goal_region);
        }
        // check orientation
        if (angles::shortest_angle_dist(state[3], m_goal.pose[3]) <= m_goal.rpy_tolerance[0] &&
            angles::shortest_angle_dist(state[4], m_goal.pose[4]) <= m_goal.rpy_tolerance[1] &&
            angles::shortest_angle_dist(state[5], m_goal.pose[5]) <= m_goal.rpy_tolerance[2])
        {
            return true;
        }
    }
    return false;
}

void WorkspaceLattice::visualizeState(
    const RobotState& state,
    const std::string& ns)
{
    auto ma = m_cc->getCollisionModelVisualization(state);
    for (auto& marker : ma.markers) {
        marker.ns = ns;
    }
    m_vpub.publish(ma);
}

void WorkspaceLattice::getActions(
    const WorkspaceLatticeState& entry,
    std::vector<Action>& actions)
{
    actions.clear();
    actions.reserve(m_prims.size());

    const double short_dist_mprims_thresh = 0.0;
    const bool use_multi_res_prims = false;

    RobotState last_state(m_dof_count);

    for (const auto& prim : m_prims) {
        if (prim.type == MotionPrimitive::Type::LONG_DISTANCE) {
            if (entry.h <= short_dist_mprims_thresh &&
                use_multi_res_prims)
            {
                continue;
            }
        }
        else if (prim.type == MotionPrimitive::Type::SHORT_DISTANCE) {
            if (entry.h > short_dist_mprims_thresh &&
                use_multi_res_prims)
            {
                continue;
            }
        }

        Action action;

        last_state = entry.state;
        for (const RobotState& delta_state : prim.action) {
            // increment the state
            for (size_t d = 0; d < m_dof_count; ++d) {
                last_state[d] += delta_state[d];
            }

            action.push_back(last_state);
        }

        actions.push_back(action);
    }
}

bool WorkspaceLattice::checkAction(
    const RobotState& state,
    const Action& action,
    double& dist)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    // check waypoints for ik solutions and joint limits
    for (size_t iidx = 0; iidx < action.size(); ++iidx) {
        const WorkspaceState& istate = action[iidx];

        ROS_DEBUG_NAMED(m_params->expands_log_, "        %zu: %s", iidx, to_string(istate).c_str());

        RobotState irstate;
        if (!stateWorkspaceToRobot(istate, irstate)) {
            ROS_DEBUG_NAMED(m_params->expands_log_, "         -> failed to find ik solution");
            violation_mask |= 0x00000001;
            break;
        }

        wptraj.push_back(irstate);

        if (!m_robot->checkJointLimits(irstate)) {
            ROS_DEBUG_NAMED(m_params->expands_log_, "        -> violates joint limits");
            violation_mask |= 0x00000002;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    int plen = 0;
    int nchecks = 0;
    if (!m_cc->isStateToStateValid(state, wptraj[0], plen, nchecks, dist)) {
        ROS_DEBUG_NAMED(m_params->expands_log_, "        -> path to first waypoint in collision");
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    for (size_t iidx = 1; iidx < wptraj.size(); ++iidx) {
        const RobotState& prev_istate = wptraj[iidx - 1];
        const RobotState& curr_istate = wptraj[iidx];
        if (!m_cc->isStateToStateValid(prev_istate, curr_istate, plen, nchecks, dist)) {
            ROS_DEBUG_NAMED(m_params->expands_log_, "        -> path between waypoints in collision");
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

#if !BROKEN

int WorkspaceLattice::computeMotionCost(
    const RobotState& from,
    const RobotState& to)
{
    double time = 0;
    double time_max = 0;

    for (size_t i = 0; i < from.size(); ++i) {
        time = fabs(angles::normalize_angle(from[i] - to[i])) / prms_.joint_vel_[i];

        ROS_DEBUG("%zu: from: %0.4f to: %0.4f dist: %0.4f vel:%0.4f time: %0.4f", i, from[i], to[i], fabs(angles::normalize_angle(from[i] - to[i])), prms_.joint_vel_[i], time);

        if (time > time_max) {
            time_max = time;
        }
    }

    ROS_DEBUG("motion cost: %d  max_time:%0.4f", (int)(m_params->cost_per_second_ * time_max), time_max);

    return m_params->cost_per_second_ * time_max;
}

int WorkspaceLattice::getJointAnglesForMotionPrimWaypoint(
    const std::vector<double>& mp_point,
    const WorkspaceState& init_wcoord,
    const RobotState& pangles,
    WorkspaceState& final_wcoord,
    std::vector<RobotState>& states)
{
    int status = 0;
    SixPose pose(6, 0);
    RobotState seed = pangles;

    for (size_t a = 0; a < 6; ++a) {
        pose[a] = init_wcoord[a] + mp_point[a];
    }

    seed[m_free_angle_idx] = init_wcoord[6] + mp_point[6];
    final_wcoord = pose;
    final_wcoord.push_back(seed[m_free_angle_idx]);

    // orientation solver - for rpy motion
    if (mp_point[0] == 0 && mp_point[1] == 0 && mp_point[2] == 0 && mp_point[6] == 0) {
        if (!rpysolver_->isOrientationFeasible(m_goal.rpy, seed, states)) {
            status = -solver_types::ORIENTATION_SOLVER;
        }
        else {
            return solver_types::ORIENTATION_SOLVER;
        }
    }

    // analytical IK
    states.resize(1, std::vector<double>(m_dof_count, 0));
    if (!m_robot->computeFastIK(pose, seed, states[0])) {
        status = -solver_types::IK;

        // IK search
        if (!m_ik_iface->computeIK(pose, seed, states[0]))
            status = -solver_types::IK_SEARCH;
        else {
            final_wcoord[6] = states[0][m_free_angle_idx];
            return solver_types::IK_SEARCH;
        }
    }
    else {
        return solver_types::IK;
    }

    return status;
}

bool WorkspaceLattice::getMotionPrimitive(
    WorkspaceLatticeState* parent,
    MotionPrimitive& mp)
{
    if (mp.type == SNAP_TO_RPY) {
        if (parent->h > prms_.cost_per_cell_ * 6) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY, parent, mp);
    }
    else if (mp.type == SNAP_TO_XYZRPY) {
        if (parent->h > prms_.cost_per_cell_ * 10) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_XYZRPY, parent, mp);
    }
    else if (mp.type == SNAP_TO_XYZ_THEN_TO_RPY) {
        if (parent->h > prms_.cost_per_cell_ * 15) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_XYZ_THEN_TO_RPY, parent, mp);
    }
    else if (mp.type == SNAP_TO_RPY_THEN_TO_XYZ) {
        if (parent->h > prms_.cost_per_cell_ * 15) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
    }
    else if (mp.type == SNAP_TO_RPY_AT_START) {
        if (parent->h < prms_.cost_per_cell_ * 3) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY_AT_START, parent, mp);
    }
    else if (mp.type == RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ) {
        if (parent->h > prms_.cost_per_cell_ * 20) {
            return false;
        }
        int x, y, z;
        if (!getDistanceGradient(x, y, z)) {
            return false;
        }
        getAdaptiveMotionPrim(RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
    }
    else if (mp.type == RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ) {
        if (m_goal_entry->coord[3] == parent->coord[3] &&
            m_goal_entry->coord[4] == parent->coord[4] &&
            m_goal_entry->coord[5] == parent->coord[5])
        {
            ROS_DEBUG_NAMED(m_params->expands_log_, "Already at goal rpy. Not doing the retract motion.");
            return false;
        }

        int x, y, z;
        if (!getDistanceGradient(x, y, z)) {
            ROS_ERROR("Zero GRadient");
            return false;
        }

        getAdaptiveMotionPrim(RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ, parent, mp);
    }
    else {
        ROS_WARN("Invalid motion primitive type");
    }

    if (all_equal(mp.coord.begin(), mp.coord.end(), 0)) {
        ROS_DEBUG("Not using adaptive mprim cause its all zeros. We should be at goal??? (type: %s)", to_string(mp.type).c_str());
        return false;
    }

    return true;
}

void WorkspaceLattice::getAdaptiveMotionPrim(
    int type,
    WorkspaceLatticeState* parent,
    MotionPrimitive& mp)
{
    mp.m.clear();
    mp.coord.clear();
    mp.coord.resize(m_dof_count, 0);

    // set rpy to motion required to satisfy goal orientation
    if (type == SNAP_TO_RPY) {
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    }
    // snap to goal pose in one motion
    else if (type == SNAP_TO_XYZRPY) {
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[0][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[0][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.m[0][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    }
    // first satisfy goal orientation in place, then move into the goal position
    else if (type == SNAP_TO_RPY_THEN_TO_XYZ) {
        mp.m.resize(2, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.m[1][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[1][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[1][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    }
    // first move to the goal position, then rotate into the goal orientation
    else if (type == SNAP_TO_XYZ_THEN_TO_RPY) {
        mp.m.resize(2, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[0][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[0][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.m[1][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[1][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[1][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    }
    else if (type == SNAP_TO_RPY_AT_START) {
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        getVector(
                m_goal_entry->coord[0], m_goal_entry->coord[1],
                m_goal_entry->coord[2], parent->coord[0], parent->coord[1], parent->coord[2], mp.coord[0],
                mp.coord[1], mp.coord[2], 5);
        getVector(
                m_goal_entry->coord[3], m_goal_entry->coord[4],
                m_goal_entry->coord[5], parent->coord[3], parent->coord[4], parent->coord[5], mp.coord[3],
                mp.coord[4], mp.coord[5], 10);

        mp.m[0][0] = double(mp.coord[0]) * m_res[0];
        mp.m[0][1] = double(mp.coord[1]) * m_res[1];
        mp.m[0][2] = double(mp.coord[2]) * m_res[2];
        mp.m[0][3] = angles::normalize_angle(double(mp.coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(mp.coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(mp.coord[5]) * m_res[5]);
        ROS_DEBUG_NAMED(m_params->expands_log_, "     [snap_to_rpy_at_start] xyz-coord: %d %d %d  rpy-coord: %d %d %d  fa-coord: %d", mp.coord[0], mp.coord[1], mp.coord[2], mp.coord[3], mp.coord[4], mp.coord[5], mp.coord[6]);
        ROS_DEBUG_NAMED(m_params->expands_log_, "     [snap_to_rpy_at_start] xyz: %0.3f %0.3f %0.3f    rpy: %0.3f %0.3f %0.3f   fa: %0.3f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
    }
    else if (type == RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ) {
        int x, y, z;
        std::vector<int> icoord(m_dof_count, 0);
        if (!getDistanceGradient(x, y, z)) {
            ROS_ERROR("I shouldn't be here...");
        }

        // get xyz for retracted pose
        getVector(
                parent->coord[0] + x, parent->coord[1] + y, parent->coord[2] + z,
                parent->coord[0], parent->coord[1], parent->coord[2],
                icoord[0], icoord[1], icoord[2],
                6, false);
        mp.m.resize(3, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(icoord[0]) * m_res[0];
        mp.m[0][1] = double(icoord[1]) * m_res[1];
        mp.m[0][2] = double(icoord[2]) * m_res[2];
        mp.m[1][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[1][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[1][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.m[2][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[2][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[2][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];

        // debugging
        if (mp.m.size() == 3) {
            std::vector<double> wcoord(m_dof_count, 0), gwcoord(m_dof_count, 0);
            coordToWorldPose(parent->coord, wcoord);
            coordToWorldPose(m_goal_entry->coord, gwcoord);
            ROS_DEBUG_NAMED(m_params->expands_log_, " [distance_gradiant] %d %d %d", x, y, z);
            ROS_DEBUG_NAMED(m_params->expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->h);
            ROS_DEBUG_NAMED(m_params->expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->h);
            ROS_DEBUG_NAMED(m_params->expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->h);
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[2][0], mp.m[2][1], mp.m[2][2], mp.m[2][3], mp.m[2][4], mp.m[2][5], mp.m[2][6], parent->h);
        }
        else {
            std::vector<double> wcoord(m_dof_count, 0), gwcoord(m_dof_count, 0);
            coordToWorldPose(parent->coord, wcoord);
            coordToWorldPose(m_goal_entry->coord, gwcoord);
            ROS_DEBUG_NAMED(m_params->expands_log_, " [distance_gradiant] %d %d %d", x, y, z);
            ROS_DEBUG_NAMED(m_params->expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->h);
            ROS_DEBUG_NAMED(m_params->expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->h);
            ROS_DEBUG_NAMED(m_params->expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-rpy]  -- ");
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->h);
        }
    }
    else if (type == RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ) {
        int x, y, z;
        std::vector<int> icoord(m_dof_count, 0);
        if (!getDistanceGradient(x, y, z)) {
            ROS_ERROR("I shouldn't be here...");
        }

        // get xyz for retracted pose
        getVector(
                parent->coord[0] + x, parent->coord[1] + y, parent->coord[2] + z,
                parent->coord[0], parent->coord[1],
                parent->coord[2], icoord[0], icoord[1],
                icoord[2], 4, false);

        // rotate towards rpy
        getVector(
                m_goal_entry->coord[3], m_goal_entry->coord[4], m_goal_entry->coord[5],
                parent->coord[3], parent->coord[4], parent->coord[5],
                mp.coord[3], mp.coord[4], mp.coord[5], 15);

        std::vector<double> im(m_dof_count, 0);
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(icoord[0]) * m_res[0];
        mp.m[0][1] = double(icoord[1]) * m_res[1];
        mp.m[0][2] = double(icoord[2]) * m_res[2];

        if (mp.coord[3] != 0 || mp.coord[4] != 0 || mp.coord[5] != 0) {
            im[3] = angles::normalize_angle(double(mp.coord[3]) * m_res[3]);
            im[4] = angles::normalize_angle(double(mp.coord[4]) * m_res[4]);
            im[5] = angles::normalize_angle(double(mp.coord[5]) * m_res[5]);
            mp.m.push_back(im);
        }
        std::vector<double> wcoord(m_dof_count, 0), gwcoord(m_dof_count, 0);
        coordToWorldPose(parent->coord, wcoord);
        coordToWorldPose(m_goal_entry->coord, gwcoord);
        ROS_DEBUG_NAMED(m_params->expands_log_, " [distance_gradiant] %d %d %d", x, y, z);
        ROS_DEBUG_NAMED(m_params->expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->h);
        ROS_DEBUG_NAMED(m_params->expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->h);
        ROS_DEBUG_NAMED(m_params->expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
        ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->h);
    }
    else {
        ROS_WARN("Invalid adaptive motion primitive type.");
    }
}

void WorkspaceLattice::getVector(
    int x1, int y1, int z1,
    int x2, int y2, int z2,
    int& xout, int& yout, int& zout,
    int multiplier,
    bool snap)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dz = z1 - z2;
    double length = sqrt(dx * dx + dy * dy + dz * dz);

    if (!snap) {
        if (dx != 0) {
            xout = (dx * multiplier) / length + 0.5;
        }
        else {
            xout = 0;
        }

        if (dy != 0) {
            yout = (dy * multiplier) / length + 0.5;
        }
        else {
            yout = 0;
        }

        if (dz != 0) {
            zout = (dz * multiplier) / length + 0.5;
        }
        else {
            zout = 0;
        }
    }
    else {
        if (dx > 0) {
            xout = min((dx * multiplier) / length + 0.5, dx);
        }
        else if (dx < 0) {
            xout = max((dx * multiplier) / length + 0.5, dx);
        }
        else {
            xout = 0;
        }

        if (dy > 0) {
            yout = min((dy * multiplier) / length + 0.5, dy);
        }
        else if (dy < 0) {
            yout = max((dy * multiplier) / length + 0.5, dy);
        }
        else {
            yout = 0;
        }

        if (dz > 0) {
            zout = min((dz * multiplier) / length + 0.5, dz);
        }
        else if (dz < 0) {
            zout = max((dz * multiplier) / length + 0.5, dz);
        }
        else {
            zout = 0;
        }
    }
    ROS_DEBUG("xyz1: %d %d %d    xyz2: %d %d %d", x1, y1, z1, x2, y2, z2);
    ROS_DEBUG("dx: %1.2f dy: %1.2f dz: %1.2f length: %1.2f multiplier: %d unit_double{%1.2f %1.2f %1.2f}  unit_int{%d %d %d}", dx, dy, dz, length, multiplier, dx / length, dy / length, dz / length, xout, yout, zout);
}

bool WorkspaceLattice::getDistanceGradient(int& x, int& y, int& z)
{
    mp_gradient_[0] = mp_dist_[0] - mp_dist_[1];
    mp_gradient_[1] = mp_dist_[2] - mp_dist_[3];
    mp_gradient_[2] = mp_dist_[4] - mp_dist_[5];

    x = mp_gradient_[0];
    y = mp_gradient_[1];
    z = mp_gradient_[2];

    if (mp_dist_[0] == -1 || mp_dist_[1] == -1) {
        mp_gradient_[0] = 0;
        return false;
    }
    if (mp_dist_[2] == -1 || mp_dist_[3] == -1) {
        mp_gradient_[1] = 0;
        return false;
    }
    if (mp_dist_[4] == -1 || mp_dist_[5] == -1) {
        mp_gradient_[2] = 0;
        return false;
    }
    if (mp_gradient_[0] == 0 && mp_gradient_[1] == 0 && mp_gradient_[2] == 0) {
        return false;
    }

    double norm = sqrt(mp_gradient_[0] * mp_gradient_[0] + mp_gradient_[1] * mp_gradient_[1] + mp_gradient_[2] * mp_gradient_[2]);
    ROS_INFO("gradient_x: %2.2f   gradient_y: %2.2f   gradient_z: %2.2f  norm: %2.2f", mp_gradient_[0], mp_gradient_[1], mp_gradient_[2], norm);
    return true;
}

#endif

} // namespace manip
} // namespace sbpl
