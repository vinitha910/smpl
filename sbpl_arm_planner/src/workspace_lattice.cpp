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

#include <boost/functional/hash.hpp>

auto std::hash<sbpl::manip::WorkspaceLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.vstates, s.vstates + s.num_vehicles));
    boost::hash_combine(seed, s.visited);
    return seed;
}

namespace sbpl {
namespace manip {

template <
    class InputIt,
    class Equal = std::equal_to<std::iterator_traits<InputIt>::value_type>>
bool all_equal(InputIt first, InputIt last, const T& val)
{
    typedef std::iterator_traits<InputIt>::reference reference;
    return std::all_of(
            first, last,
            [&val](const reference a)) { return a == val; });
}

WorkspaceLattice::WorkspaceLattice(
    OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* cc,
    PlanningParams* params)
:
    m_vpub(),
    m_grid(grid),
    m_robot(robot),
    m_cc(cc),
    m_params(params),
    m_heur(nullptr),
    m_goal(),
    m_goal_entry(nullptr),
    m_start_entry(nullptr),
    m_state_to_id(),
    m_states(),
    mp_dist_(6),
    mp_gradient_(3),
    m_free_angle_count(1), // TODO: retrieve from robot model extension
    m_free_angle_idx(-1),
    m_dof_count(-1)
{
    m_vpub = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("visualization_markers", 100);
}

WorkspaceLattice::~WorkspaceLattice()
{
}

bool WorkspaceLattice::init(const Params& params)
{
    m_free_angle_count = 1;
    m_dof_count = 6 + m_free_angle_count;
    m_free_angle_idx = params.free_angle_index;

    m_res.resize(m_dof_count);

    m_res[0] = m_grid->getResolution();
    m_res[1] = m_grid->getResolution();
    m_res[2] = m_grid->getResolution();
    // TODO: limit these ranges and handle discretization appropriately
    m_res[3] = (2.0 * M_PI) / params.R_count;
    m_res[4] = (2.0 * M_PI) / params.P_count;
    m_res[5] = (2.0 * M_PI) / params.Y_count;

    for (int i = 0; i < m_free_angle_count; ++i) {
        m_res[6 + i] = (2.0 * M_PI) / params.joint_res[i];
    }

    m_val_count[0] = std::numeric_limits<int>::max();
    m_val_count[1] = std::numeric_limits<int>::max();
    m_val_count[2] = std::numeric_limits<int>::max();
    m_val_count[3] = params.R_count;
    m_val_count[4] = params.P_count;
    m_val_count[5] = params.Y_count;
    for (int i = 0; i < m_free_angle_count; ++i) {
        m_val_count[6 + i] = (2.0 * M_PI) / params.joint_res[i];
    }

    ROS_INFO("discretization of workspace lattice:");
    ROS_INFO("  x: { res: %0.3f, count: %d }", m_res[0], m_val_count[0]);
    ROS_INFO("  y: { res: %0.3f, count: %d }", m_res[1], m_val_count[1]);
    ROS_INFO("  z: { res: %0.3f, count: %d }", m_res[2], m_val_count[2]);
    ROS_INFO("  R: { res: %0.3f, count: %d }", m_res[3], m_val_count[3]);
    ROS_INFO("  P: { res: %0.3f, count: %d }", m_res[4], m_val_count[4]);
    ROS_INFO("  Y: { res: %0.3f, count: %d }", m_res[5], m_val_count[5]);
    for (int i = 0; i < m_free_angle_count; ++i) {
        ROS_INFO("  J%d: { res: %0.3f, count: %d }", m_res[6 + i], m_val_count[6 + i]);
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
                prim.action = std::move(d);

                m_prims.push_back(prim);
            }
        }
    }

    // create 2-connected motions for rotation and free angle motions
    for (int a = 3; a < m_dof_count; ++a) {
        std::vector<double> d(m_dof_count, 0.0);
        d[a] = m_res[a] * -1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action = d;
        m_prims.push_back(prim);

        std::vector<double> d(m_dof_count, 0.0);
        d[a] = m_res[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action = d;
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

    ROS_DEBUG_NAMMED(m_params->graph_log_, "  angles: %s", to_string(state).c_str());

    SixPose pose;
    if (!computePlanningFrameFK(state, pose)) {
        ROS_ERROR_NAMED(m_params->graph_log_, "failed to compute forward kinematics for start state");
        return false;
    }
    ROS_DEBUG_NAMED(m_params->graph_log_, "planning link pose: { x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f }", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    if (!m_robot->checkJointLimits(state, state)) {
        ROS_ERROR_NAMED(m_param->graph_log_, "start state violates joint limits");
        return false;
    }

    if (!m_cc->isStateValid(state, true, false, dist)) {
        ROS_WARN("start state is in collision");
        return false;
    }

    visualizeState(state, "start_config");
    WorkspaceCoord start_coord;
    stateRobotToCoord(state, start_coord);

    int start_id = createState(start_coord);
    m_start_entry = getState(start_id);
    m_start_entry->state = state;

    return true;
}

bool WorkspaceLattice::setGoalPose(const PoseGoal& goal)
{
    if (goal.pose.size() != 7) {
        ROS_ERROR("Insufficient goal pose vector");
        return false;
    }

    if (goal.offset.size() != 3) {
        ROS_ERROR("Insufficient goal pose offset vector");
        return false;
    }

    if (goal.tolerance.size() != 6) {
        ROS_ERROR("Insufficient goal pose tolerance vector");
        return false;
    }

    // check if an IK solution exists for the goal pose before we do
    // the search we plan even if there is no solution
    RobotState seed(m_params->num_joints_, 0);
    RobotState ik_solution;
    if (!m_robot->computeIK(goal.pose, seed, ik_solution)) {
        ROS_WARN("No valid IK solution for the goal pose.");
    }

    m_goal.pose = goal.pose;

    m_goal.xyz_tolerance[0] = goal.tolerance[0];
    m_goal.xyz_tolerance[1] = goal.tolerance[1];
    m_goal.xyz_tolerance[2] = goal.tolerance[2];
    m_goal.rpy_tolerance[0] = goal.tolerance[3];
    m_goal.rpy_tolerance[1] = goal.tolerance[4];
    m_goal.rpy_tolerance[2] = goal.tolerance[5];

    m_goal.type = GoalType::XYZ_RPY_GOAL;

    m_goal.xyz_disc_tolerance = goal.tolerance[0] / m_grid->getResolution();
    m_goal.rpy_disc_tolerance = goal.tolerance[3] / m_res[3];

    worldPoseToCoord(m_goal.xyz, m_goal.rpy, m_goal.fangle, m_goal_entry->coord);

    ROS_INFO(" xyz: %.2f %.2f %.2f (meters) (tol: %.3fm)", m_goal.xyz[0], m_goal.xyz[1], m_goal.xyz[2], m_goal.xyz_tolerance[0]);
    ROS_INFO(" rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3frad)", m_goal.rpy[0], m_goal.rpy[1], m_goal.rpy[2], m_goal.rpy_tolerance[0]);
    ROS_INFO(" coord: %u %u %u (tol: %d)   %u %u %u (tol: %d)", m_goal_entry->coord[0], m_goal_entry->coord[1], m_goal_entry->coord[2], m_goal.xyz_disc_tolerance, m_goal_entry->coord[3], m_goal_entry->coord[4], m_goal_entry->coord[5], m_goal.rpy_disc_tolerance);

    ROS_INFO("COST TO GOAL OF START STATE: %d", getBFSCostToGoal(m_start_entry->coord[0], m_start_entry->coord[1], m_start_entry->coord[2]));

    return true;
}

bool WorkspaceLattice::setGoalPoses(const std::vector<PoseGoal>& goals)
{
    return false;
}

int WorkspaceLattice::getStartStateID() const
{
    return -1;
}

int WorkspaceLattice::getGoalStateID() const
{
    return -1;
}

bool WorkspaceLatticee::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    std::vector<int> cost;
    std::vector<int> succs;
    WorkspaceState interm_wcoord(m_dof_count, 0);
    WorkspaceState source_wcoord(m_dof_count, 0);
    std::vector<RobotState> interm_angles;
    path.clear();

    ROS_DEBUG_NAMED(m_params->solution_log_, "Path:");
    for (size_t p = 0; p < idpath.size() - 1; ++p) {
        int src_id;
        int dst_id;
        int best_cost;
        int best_succ;
        src_id = idpath[p];
        dst_id = idpath[p + 1];

        cost.clear();
        GetSuccs(src_id, &succs, &cost);

        best_cost = std::numeric_limimts<int>::max();
        best_succ = -1;

        // find the best successor
        for (size_t s = 0; s < succs.size(); ++s) {
            if (succs[s] == dst_id && cost[s] <= best_cost) {
                best_cost = cost[s];
                best_succ = s;
            }
        }

        if (best_succ == -1) {
            ROS_ERROR("[%zu/%zu] Transition not found during path extraction. (%d -> %d)", p, idpath.size(), src_id, dst_id);
            return false;
        }

        ROS_INFO("Found transition (%d, %d) via successor %d", src_id, dst_id, best_succ);

        WorkspaceLatticeState* source_entry = getState(src_id);
        const RobotState& source_angles = source_entry->state;
        stateCoordToWorkspace(source_entry->coord, source_wcoord);

        const MotionPrimitive& mprim = m_prims[best_succ];
        for (size_t i = 0; i < mprim.m.size(); ++i) {
            if (getJointAnglesForMotionPrimWaypoint(
                    mprim.m[i], source_wcoord, source_angles, interm_wcoord, interm_angles) > 0)
            {
                for (auto& point : interm_angles) {
                    // normalize joint angles
                    for (double& pos : point) {
                        d = angles::normalize_angle(d);
                    }
                    path.push_back(point);
                }

                source_wcoord = interm_wcoord;
            }
            else {
                ROS_WARN("Failed to convert coords to angles when attempting to construct the path.");
            }
        }
        ROS_DEBUG_NAMED(m_params->solution_log_, "[%2zu] stateid: %5d mp_index: %2d mp_type: %14s mp_group: %d num_waypoints: %zu heur: %d", p, idpath[p], best_succ, to_string(m_prims[best_succ].type).c_str(), m_prims[best_succ].group, interm_angles.size(), source_entry->heur);
    }
}

int WorkspaceLattice::GetFromToHeuristic(int from_state_id, int to_state_id)
{
    int xyz_heur = 0, rpy_heur = 0;
    WorkspaceLatticeState* state = EnvROBARM.StateID2CoordTable[FromStateID];

    xyz_heur = getBFSCostToGoal(state->coord[0], state->coord[1], state->coord[2]);

    int r = (m_goal_entry->coord[3] - state->coord[3]) % EnvROBARMCfg.coord_vals[3];
    int p = (m_goal_entry->coord[4] - state->coord[4]) % EnvROBARMCfg.coord_vals[4];
    int y = (m_goal_entry->coord[5] - state->coord[5]) % EnvROBARMCfg.coord_vals[5];

    rpy_heur = (abs(r) + abs(p) + abs(y)) * m_params->cost_per_cell_;
    ROS_DEBUG("stateid: %5d   xyz: %2d %2d %2d  rpy: %2d %2d %2d  (goal-rpy: %2d %2d %2d)", FromStateID, state->coord[0], state->coord[1], state->coord[2], state->coord[3], state->coord[4], state->coord[5], m_goal_entry->coord[3], m_goal_entry->coord[4], m_goal_entry->coord[5]);
    ROS_DEBUG("               xyz-heur: %4d  rpy-heur: %4d  (r: %2d  p:%2d  y: %2d)", xyz_heur, rpy_heur, r, p, y);

    state->heur = xyz_heur + rpy_heur;
    return xyz_heur + rpy_heur;
}

int WorkspaceLattice::GetGoalHeuristic(int state_id)
{
    return 0;
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

    const WorkspaceState zeros(m_dof_count, 0);
    std::vector<RobotState> wptraj;

    // clear the successor arrays
    succs->clear();
    costs->clear();
    succs->reserve(m_prims.size());
    costs->reserve(m_prims.size());
    if (actions) {
        actions->clear();
        actions->reserve(m_prims.size());
    }

    ROS_DEBUG_NAMED(m_params->expands_log_, "expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    WorkspaceLatticeState* parent = getState(state_id);

    assert(parent);
    assert(parent->coord.size() == m_dof_count);

    ROS_DEBUG_NAMED(m_params->expands_log_, "  coord: %s", to_string(parent->coord).c_str());
    ROS_DEBUG_NAMED(m_params->expands_log_, "  state: %s", to_string(parent->state).c_str());

    const RobotState& source_angles = parent->state;
    visualizeState(source_angles, "expansion");

    std::vector<Action> actions;
    getActions(source_angles, actions);

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(m_params->expands_log_, "    action %zu", i);
        ROS_DEBUG_NAMED(m_params->expands_log_, "      waypoints: %zu", action.size());

        double dist = 100;
        bool invalid_prim = false;
        double dist_temp = 0;
        int motion_cost = 0;
        int path_length = 0;
        int wp_length = 0;

        if (!checkAction(source_angles, action, dist)) {
            continue;
        }

        const RobotState& last_state = action.back();
        WorkspaceState last_workspace_state;
        stateRobotToWorkspace(last_state, last_workspace_state);

        std::vector<int> succ_coord;
        discretizeWorkspaceState(last_workspace_state, succ_coord);

        // compute the successor coords
        for (size_t a = 0; a < mp_coord.size(); ++a) {
            if ((succ_coord[a] + mp_coord[a]) < 0) {
                succ_coord[a] = ((EnvROBARMCfg.coord_vals[a] + succ_coord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
            }
            else {
                succ_coord[a] = ((int)(succ_coord[a] + int(mp_coord[a])) % EnvROBARMCfg.coord_vals[a]);
            }
        }

        // check if this state meets the goal criteria
        bool bSuccisGoal = false;
        if (isGoal(last_workspace_state)) {
            bSuccisGoal = true;

            for (int j = 0; j < m_dof_count; j++) {
                m_goal_entry->coord[j] = succ_coord[j];
            }

            m_goal_entry->state = sangles;
            ROS_DEBUG("Goal state has been found. Parent StateID: %d (obstacle distance: %0.3f)", state_id, dist);
            ROS_DEBUG("  coord: %s", to_string(succ_coord).c_str());
        }

        // check if hash entry already exists, if not then create one
        int succ_id = createState(succ_coord);
        WorkspaceLatticeState* succ_state = getState(succ_id);
        succ_state->state = sangles;
        succ_state->action = i;
        ROS_DEBUG("  parentid: %d stateid: %d mprim: %zu cost: %4d heur: %2d coord: %s", state_id, succ_id, i, motion_cost, GetFromToHeuristic(succ_id, m_goal_state_id), to_string(succ_coord).c_str());

        if (m_prims[i].type == sbpl_arm_planner::ADAPTIVE) {
            motion_cost = 30;
        }
        else {
            motion_cost = 60;
        }

        // put successor on successor list with the proper cost
        succs->push_back(succ_id);
        costs->push_back(motion_cost); //(cost(parent,succ_state,bSuccisGoal));
        if (actions != NULL) {
            actions->push_back(i);
        }
    }
}

/// \brief Create a state entry for a given coordinate and return its id
///
/// If an entry already exists for the coordinate, the id corresponding to that
/// entry is returned; otherwise, a new entry is created and its id returned.
int WorkspaceLattice::createState(const std::vector<int>& coord)
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
    m_state_to_id[entry] = new_id;

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

bool WorkspaceLattice::isGoal(const std::vector<double>& workspace_state)
{
    // check position
    if (fabs(workspace_state[0] - m_goal.xyz[0]) <= m_goal.xyz_tolerance[0] &&
        fabs(workspace_state[1] - m_goal.xyz[1]) <= m_goal.xyz_tolerance[1] &&
        fabs(workspace_state[2] - m_goal.xyz[2]) <= m_goal.xyz_tolerance[2])
    {
        // log the amount of time required for the search to get close to the goal
        if (!near_goal) {
            time_to_goal_region = (clock() - starttime) / (double)CLOCKS_PER_SEC;
            near_goal = true;
            ROS_INFO("search is at the goal position after %0.3f sec", time_to_goal_region);
        }
        // check orientation
        if (fabs(angles::shortest_angular_distance(workspace_state[3], m_goal.rpy[0])) <= m_goal.rpy_tolerance[0] &&
            fabs(angles::shortest_angular_distance(workspace_state[4], m_goal.rpy[1])) <= m_goal.rpy_tolerance[1] &&
            fabs(angles::shortest_angular_distance(workspace_state[5], m_goal.rpy[2])) <= m_goal.rpy_tolerance[2])
        {
            return true;
        }
    }
    return false;
}

void WorkspaceLattice::stateIDToPose(int stateID, int* xyz, int* rpy, int* fangle)
{
    WorkspaceLatticeState* HashEntry = getState(stateID);

    if (stateID == m_goal_state_id) {
        xyz[0] = m_goal_entry->coord[0];
        xyz[1] = m_goal_entry->coord[1];
        xyz[2] = m_goal_entry->coord[2];
        rpy[0] = m_goal_entry->coord[3];
        rpy[1] = m_goal_entry->coord[4];
        rpy[2] = m_goal_entry->coord[5];
        *fangle = m_goal_entry->coord[6];
    }
    else {
        xyz[0] = HashEntry->coord[0];
        xyz[1] = HashEntry->coord[1];
        xyz[2] = HashEntry->coord[2];
        rpy[0] = HashEntry->coord[3];
        rpy[1] = HashEntry->coord[4];
        rpy[2] = HashEntry->coord[5];
        *fangle = HashEntry->coord[6];
    }
}

void WorkspaceLattice::worldPoseToCoord(
    double* wxyz, double* wrpy, double wfangle, std::vector<int>& coord)
{
    int xyz[3] = { 0 }, rpy[3] = { 0 }, fangle = 0;

    worldToDiscXYZ(wxyz, xyz);
    worldToDiscRPY(wrpy, rpy);
    worldToDiscFAngle(wfangle, &fangle);

    coord.resize(m_dof_count);
    coord[0] = xyz[0];
    coord[1] = xyz[1];
    coord[2] = xyz[2];
    coord[3] = rpy[0];
    coord[4] = rpy[1];
    coord[5] = rpy[2];
    coord[6] = fangle;
}

void WorkspaceLattice::robotStateToCoord(
    const std::vector<double>& angles,
    std::vector<int>& coord)
{
    std::vector<double> pose;
    m_robot->computePlanningLinkFK(angles, pose);

    double fangle = angles[m_free_angle_idx];
    double wxyz[3] = { pose[0], pose[1], pose[2] };
    double wrpy[3] = { pose[3], pose[4], pose[5] };

    worldPoseToCoord(wxyz, wrpy, fangle, coord);
}

int WorkspaceLattice::computeMotionCost(
    const std::vector<double>& a,
    const std::vector<double>& b)
{
    double time = 0;
    double time_max = 0;

    for (size_t i = 0; i < a.size(); ++i) {
        time = fabs(angles::normalize_angle(a[i] - b[i])) / prms_.joint_vel_[i];

        ROS_DEBUG("%zu: a: %0.4f b: %0.4f dist: %0.4f vel:%0.4f time: %0.4f", i, a[i], b[i], fabs(angles::normalize_angle(a[i] - b[i])), prms_.joint_vel_[i], time);

        if (time > time_max) {
            time_max = time;
        }
    }

    ROS_DEBUG("motion cost: %d  max_time:%0.4f", (int)(m_params->cost_per_second_ * time_max), time_max);

    return m_params->cost_per_second_ * time_max;
}

void WorkspaceLattice::visualizeState(
    const std::vector<double>& state,
    const std::string& ns)
{
    auto ma = cc_->getCollisionModelVisualization(jvals);
    for (auto& marker : ma.markers) {
        marker.ns = ns;
    }
    m_vpub.publish(ma);
}

/// \brief Convert a discrete position to its continuous counterpart.
inline
void WorkspaceLattice::discToWorldXYZ(const int* gp, double* wp)
{
    m_grid->gridToWorld(gp[0], gp[1], gp[2], wp[0], wp[1], wp[2]);
}

/// \brief Convert a continuous position to its discrete counterpart.
inline
void WorkspaceLattice::worldToDiscXYZ(const double* wp, int* gp)
{
    m_grid->worldToGrid(wp[0], wp[1], wp[2], gp[0], gp[0], gp[0]);
}

/// \brief Convert a discrete rotation to its continuous counterpart.
inline
void WorkspaceLattice::discToWorldRPY(int* rpy, double* wrpy)
{
    wrpy[0] = angles::normalize_angle((double)rpy[0] * m_res[3]);
    wrpy[1] = angles::normalize_angle((double)rpy[1] * m_res[4]);
    wrpy[2] = angles::normalize_angle((double)rpy[2] * m_res[5]);

    ROS_DEBUG("rpy: %d %d %d --> %2.3f %2.3f %2.3f", rpy[0], rpy[1], rpy[2], wrpy[0], wrpy[1], wrpy[2]);
}

/// \brief Convert a continuous rotation to its continuous counterpart.
inline
void WorkspaceLattice::worldToDiscRPY(double* wrpy, int* rpy)
{
    rpy[0] = (int)((angles::normalize_angle_positive(wrpy[0]) + m_res[3] * 0.5) / m_res[3]) % m_val_count[3];
    rpy[1] = (int)((angles::normalize_angle_positive(wrpy[1]) + m_res[4] * 0.5) / m_res[4]) % m_val_count[4];
    rpy[2] = (int)((angles::normalize_angle_positive(wrpy[2]) + m_res[5] * 0.5) / m_res[5]) % m_val_count[5];

    ROS_DEBUG("rpy: %2.3f %2.3f %2.3f --> %d %d %d", wrpy[0], wrpy[1], wrpy[2], rpy[0], rpy[1], rpy[2]);

    if (rpy[0] >= m_val_count[3] || rpy[1] >= m_val_count[4] || rpy[2] >= m_val_count[5]) {
        ROS_ERROR("wrpy: %0.3f %0.3f %0.3f discretized to %d %d %d", wrpy[0], wrpy[1], wrpy[2], rpy[0], rpy[1], rpy[2]);
    }
}

/// \brief Convert a discrete joint variable to its continuous counterpart.
inline
void WorkspaceLattice::discToWorldFAngle(int fangle, double* wfangle)
{
    *wfangle = angles::normalize_angle((double)fangle * m_res[6]);

    ROS_DEBUG("fangle: %d --> %2.3f", fangle, *wfangle);
}

/// \brief Convert a continuous joint variable to its discrete counterpart.
inline
void WorkspaceLattice::worldToDiscFAngle(double wfangle, int* fangle)
{
    *fangle = (int)((angles::normalize_angle_positive(wfangle) + m_res[6] * 0.5) / m_res[6]) % m_val_count[6];

    ROS_DEBUG("fangle: %2.3f --> %d", wfangle, *fangle);

    if (*fangle >= m_val_count[6]) {
        ROS_ERROR("fangle: %0.3f discretized to: %d", wfangle, *fangle);
    }
}

bool WorkspaceLattice::convertWorldPoseToAngles(
    const std::vector<double>& wpose,
    std::vector<double>& angles)
{
    std::vector<double> pose(6, 0), seed(7, 0);

    for (size_t i = 0; i < 6; ++i)
        pose[i] = wpose[i];

    seed[m_free_angle_idx] = wpose[6];

    if (!m_robot->computeFastIK(pose, seed, angles)) {
        ROS_DEBUG("computeFastIK failed to return a solution");

        if (!m_robot->computeIK(pose, seed, angles)) {
            ROS_DEBUG("IK Search found solution");
            return false;
        }
    }

    return true;
}

bool WorkspaceLattice::convertWorldPoseToAngles(
    const std::vector<double>& wpose,
    std::vector<double> seed,
    std::vector<double>& angles)
{
    std::vector<double> pose(6, 0);

    for (size_t i = 0; i < 6; ++i) {
        pose[i] = wpose[i];
    }

    seed[m_free_angle_idx] = wpose[6];

    if (!m_robot->computeFastIK(pose, seed, angles)) {
        ROS_DEBUG("computeFastIK failed to return a solution");
        return false;
        /*
         if(!arm_->computeIK(pose,seed,angles))
         {
         ROS_DEBUG("IK Search found solution");
         return false;
         }
         */
    }

    return true;
}

void WorkspaceLattice::stateIdToRobotState(int state_id, RobotState& state)
{
    WorkspaceLatticeState* state_entry = m_states[state_id];
    state = state_entry->state;
}

void WorkspaceLattice::getActions(
    const RobotState& state,
    std::vector<Action>& actions)
{
    actions.clear();
    actions.reserve(m_prims.size());

    const double short_dist_mprims_thresh = 0.0;
    const bool use_multi_res_prims = false;

    RobotState last_state(m_dof_count);

    for (const auto& mprim : m_prims) {
        if (mp.type == LONG_DISTANCE) {
            if (parent->heur <= short_dist_mprims_thresh &&
                use_multi_res_prims)
            {
                continue;
            }
        }
        else if (mp.type == SHORT_DISTANCE) {
            if (parent->heur > short_dist_mprims_thresh &&
                use_multi_res_prims)
            {
                continue;
            }
        }

        Action action;

        last_state = state;
        for (const RobotState& delta_state : mprim.action) {
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
    RobotState interm_state = parent->state;

    // init_wcoord is the world coords between intermediate waypoints
    WorkspaceState init_wcoord;
    stateCoordToWorkspace(parent->coord, init_wcoord);

    // loop through the waypoints in the motion primitive
    for (size_t j = 0; j < m_prims[i].m.size(); ++j) {
        if (m_prims[i].m[j] == zeros) {
            continue;
        }

        int convert_code = 0;
        WorkspaceState final_wcoord(m_dof_count, 0);
        if ((convert_code = getJointAnglesForMotionPrimWaypoint(m_prims[i].m[j], init_wcoord, interm_state, final_wcoord, wptraj)) < 0) {
            ROS_DEBUG_NAMED(m_params->expands_log_, "  succ: %2d-%1d       mp_type: %10s  convert_code: %2d  Converting to angles failed.", int(i), int(j), to_string(m_prims[i].type).c_str(), convert_code);
            invalid_prim = true;
            break;
        }
        else {
            wp_length = 0;

            if (init_wcoord == final_wcoord) {
                ROS_ERROR("  succ: %2d-%1d WTF...these world coords should not be equal.", int(i), int(j));
            }

            for (size_t q = 0; q < wptraj.size(); ++q) {
                // debug - remove this eventually
                if (wptraj[q] == interm_state)
                    ROS_ERROR("  succ: %2d  waypoint: %1d  sub-waypoint: %2d(%d)  convert_code: %2d   ** Has the same angles as its parent.**", int(i), int(j), int(q), int(wptraj.size()), convert_code);

                // ROS_DEBUG_NAMED(m_params->expands_log_, "    [%d] -> [%d-%d-%d] sub-waypoint: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f ", state_id, int(i), int(j), int(q), wptraj[q][0],wptraj[q][1],wptraj[q][2],wptraj[q][3],wptraj[q][4],wptraj[q][5],wptraj[q][6]);

                // check motion for collisions
                int motion_code = 0;
                int nchecks = 0;
                if ((motion_code = m_cc->isStateToStateValid(interm_state, wptraj[q], path_length, nchecks, dist_temp)) < 0) {
                    ROS_DEBUG_NAMED(m_params->expands_log_, "  succ: %2d-%1d-%1d(%1d)  mp_type: %10s  convert_code: %2d  valid_code: %2d  dist: %3d  # checks: (%2d)    Invalid.", int(i), int(j), int(q), int(wptraj.size()), to_string(m_prims[i].type).c_str(), convert_code, motion_code, int(dist_temp), path_length);
                    dist = dist_temp;
                    invalid_prim = true;
                    break;
                }
                wp_length += path_length;

                if (dist_temp < dist)
                    dist = dist_temp;

                // save joint angles of current waypoint for next sub-waypoint in intermediate waypoint
                interm_state = wptraj[q];
            }

            if (invalid_prim) {
                break;
            }

            ROS_DEBUG_NAMED(m_params->expands_log_, "  succ: %2d-%1d-%1d     mp_type: %10s  convert_code: %2d  valid_code: %2d  dist: %3d  # checks: (%2d)    Valid.", int(i), int(j), int(wptraj.size()), to_string(m_prims[i].type).c_str(), convert_code, motion_code, int(dist_temp), wp_length);
            // update current joint angle vector (sangles)
            RobotState sangles = wptraj.back();

            // update the current world coords
            init_wcoord = final_wcoord;

            // if the FA was changed by the IK search and it is the final waypoint of the mp
            if (convert_code == 3 && j == m_prims[i].m.size() - 1) {
                int fa;
                worldToDiscFAngle(sangles[2], &fa);
                mp_coord[6] = parent->coord[6] - fa;
                ROS_DEBUG("  IK search succeeded. Updated FA in coord: %s -> %d %d %d %d %d %d %d", to_string(m_prims[i].coord).c_str(), to_string(mp_coord).c_str());
            }

            if (dist_temp < dist) {
                dist = dist_temp;
            }

            // add up motion cost for all waypoints in mprim
            motion_cost += computeMotionCost(interm_state, sangles);
        }
    }

    return true;
}

int WorkspaceLattice::getJointAnglesForMotionPrimWaypoint(
    const std::vector<double>& mp_point,
    const std::vector<double>& init_wcoord,
    const std::vector<double>& pangles,
    std::vector<double>& final_wcoord,
    std::vector<std::vector<double>>& angles)
{
    int status = 0;
    std::vector<double> pose(6, 0);
    std::vector<double> seed = pangles;

    for (size_t a = 0; a < 6; ++a) {
        pose[a] = init_wcoord[a] + mp_point[a];
    }

    seed[m_free_angle_idx] = init_wcoord[6] + mp_point[6];
    final_wcoord = pose;
    final_wcoord.push_back(seed[m_free_angle_idx]);

    // orientation solver - for rpy motion
    if (mp_point[0] == 0 && mp_point[1] == 0 && mp_point[2] == 0 && mp_point[6] == 0) {
        if (!rpysolver_->isOrientationFeasible(m_goal.rpy, seed, angles)) {
            status = -solver_types::ORIENTATION_SOLVER;
        }
        else {
            return solver_types::ORIENTATION_SOLVER;
        }
    }

    // analytical IK
    angles.resize(1, std::vector<double>(m_dof_count, 0));
    if (!m_robot->computeFastIK(pose, seed, angles[0])) {
        status = -solver_types::IK;

        // IK search
        if (!m_robot->computeIK(pose, seed, angles[0]))
            status = -solver_types::IK_SEARCH;
        else {
            final_wcoord[6] = angles[0][m_free_angle_idx];
            return solver_types::IK_SEARCH;
        }
    }
    else {
        return solver_types::IK;
    }

    return status;
}

#if !BROKEN

bool WorkspaceLattice::getMotionPrimitive(
    WorkspaceLatticeState* parent,
    MotionPrimitive& mp)
{
    if (mp.type == SNAP_TO_RPY) {
        if (parent->heur > prms_.cost_per_cell_ * 6) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY, parent, mp);
    }
    else if (mp.type == SNAP_TO_XYZRPY) {
        if (parent->heur > prms_.cost_per_cell_ * 10) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_XYZRPY, parent, mp);
    }
    else if (mp.type == SNAP_TO_XYZ_THEN_TO_RPY) {
        if (parent->heur > prms_.cost_per_cell_ * 15) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_XYZ_THEN_TO_RPY, parent, mp);
    }
    else if (mp.type == SNAP_TO_RPY_THEN_TO_XYZ) {
        if (parent->heur > prms_.cost_per_cell_ * 15) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
    }
    else if (mp.type == SNAP_TO_RPY_AT_START) {
        if (parent->heur < prms_.cost_per_cell_ * 3) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY_AT_START, parent, mp);
    }
    else if (mp.type == RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ) {
        if (parent->heur > prms_.cost_per_cell_ * 20) {
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
        ROS_DEBUG_NAMED(m_params->expands_log_, "     [snap_to_rpy_at_start] xyz: %0.3f %0.3f %0.3f    rpy: %0.3f %0.3f %0.3f   fa: %0.3f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->heur);
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
            ROS_DEBUG_NAMED(m_params->expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->heur);
            ROS_DEBUG_NAMED(m_params->expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->heur);
            ROS_DEBUG_NAMED(m_params->expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->heur);
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->heur);
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[2][0], mp.m[2][1], mp.m[2][2], mp.m[2][3], mp.m[2][4], mp.m[2][5], mp.m[2][6], parent->heur);
        }
        else {
            std::vector<double> wcoord(m_dof_count, 0), gwcoord(m_dof_count, 0);
            coordToWorldPose(parent->coord, wcoord);
            coordToWorldPose(m_goal_entry->coord, gwcoord);
            ROS_DEBUG_NAMED(m_params->expands_log_, " [distance_gradiant] %d %d %d", x, y, z);
            ROS_DEBUG_NAMED(m_params->expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->heur);
            ROS_DEBUG_NAMED(m_params->expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->heur);
            ROS_DEBUG_NAMED(m_params->expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->heur);
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-rpy]  -- ");
            ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->heur);
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
        ROS_DEBUG_NAMED(m_params->expands_log_, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->heur);
        ROS_DEBUG_NAMED(m_params->expands_log_, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->heur);
        ROS_DEBUG_NAMED(m_params->expands_log_, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->heur);
        ROS_DEBUG_NAMED(m_params->expands_log_, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->heur);
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
