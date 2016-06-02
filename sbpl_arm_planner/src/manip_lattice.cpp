/*
 * Copyright (c) 2010, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#include <sbpl_arm_planner/manip_lattice.h>

// standard includes
#include <sstream>

// system includes
#include <Eigen/Dense>
#include <angles/angles.h>
#include <leatherman/viz.h>
#include <leatherman/print.h>

#include <sbpl_arm_planner/manip_heuristic.h>

#define DEG2RAD(d) ((d) * (M_PI / 180.0))
#define RAD2DEG(r) ((r) * (180.0 / M_PI))

namespace sbpl {
namespace manip {

ManipLattice::ManipLattice(
    OccupancyGrid* grid,
    RobotModel* rmodel,
    CollisionChecker* cc,
    ActionSet* as,
    PlanningParams* pm)
:
    DiscreteSpaceInformation(),
    grid_(grid),
    rmodel_(rmodel),
    cc_(cc),
    as_(as),
    prm_(pm),
    m_heur(nullptr),
    m_min_limits(),
    m_max_limits(),
    m_continuous(),
    m_near_goal(false),
    m_t_start(),
    m_time_to_goal_region(),
    m_use_7dof_goal(false),
    m_goal(),
    m_goal_7dof(),
    m_goal_entry(nullptr),
    m_start_entry(nullptr),
    m_HashTableSize(32 * 1024),
    m_Coord2StateIDHashTable(nullptr),
    m_states(),
    m_expanded_states(),
    nh_(),
    m_vpub(),
    m_initialized(false)
{
    m_vpub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1);

    m_min_limits.resize(prm_->num_joints_);
    m_max_limits.resize(prm_->num_joints_);
    m_continuous.resize(prm_->num_joints_);
    for (int jidx = 0; jidx < prm_->num_joints_; ++jidx) {
        m_min_limits[jidx] = rmodel->minVarLimit(jidx);
        m_max_limits[jidx] = rmodel->maxVarLimit(jidx);
        m_continuous[jidx] = rmodel->hasVarLimit(jidx);
    }
}

ManipLattice::~ManipLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();

    if (m_Coord2StateIDHashTable) {
        delete [] m_Coord2StateIDHashTable;
        m_Coord2StateIDHashTable = nullptr;
    }
}

bool ManipLattice::InitializeMDPCfg(MDPConfig* MDPCfg)
{
    if (!m_goal_entry || !m_start_entry) {
        return false;
    }
    else {
        MDPCfg->goalstateid = m_goal_entry->stateID;
        MDPCfg->startstateid = m_start_entry->stateID;
        return true;
    }
}

int ManipLattice::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    assert(FromStateID >= 0 && FromStateID < (int)m_states.size());
    assert(ToStateID >= 0 && ToStateID < (int)m_states.size());
    return m_heur->GetFromToHeuristic(FromStateID, ToStateID);
}

int ManipLattice::GetGoalHeuristic(int state_id)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    EnvROBARM3DHashEntry_t* state = m_states[state_id];
    state->heur = m_heur->GetGoalHeuristic(state_id);
    return state->heur;
}

int ManipLattice::GetStartHeuristic(int state_id)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    EnvROBARM3DHashEntry_t* state = m_states[state_id];
    state->heur = m_heur->GetStartHeuristic(state_id);
    return state->heur;
}

int ManipLattice::SizeofCreatedEnv()
{
    return (int)m_states.size();
}

void ManipLattice::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    assert(stateID >= 0 && stateID < (int)m_states.size());

    if (!fOut) {
        fOut = stdout;
    }

    EnvROBARM3DHashEntry_t* HashEntry = m_states[stateID];

    const bool is_goal = (stateID == m_goal_entry->stateID);
    printJointArray(fOut, HashEntry, is_goal, bVerbose);
}

void ManipLattice::PrintEnv_Config(FILE* fOut)
{
    ROS_WARN("PrintEnv_Config unimplemented");
}

void ManipLattice::GetSuccs(
    int SourceStateID,
    std::vector<int>* SuccIDV,
    std::vector<int>* CostV)
{
    assert(SourceStateID >= 0 && SourceStateID < m_states.size());

    SuccIDV->clear();
    CostV->clear();

    ROS_DEBUG_NAMED(prm_->expands_log_, "expanding state %d", SourceStateID);

    //goal state should be absorbing
    if (SourceStateID == m_goal_entry->stateID) {
        return;
    }

    EnvROBARM3DHashEntry_t* parent_entry = m_states[SourceStateID];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= prm_->num_joints_);

    // log expanded state details
    ROS_DEBUG_NAMED(prm_->expands_log_, "  coord: %s", to_string(parent_entry->coord).c_str());
    ROS_DEBUG_NAMED(prm_->expands_log_, "  angles: %s", to_string(parent_entry->state).c_str());
    ROS_DEBUG_NAMED(prm_->expands_log_, "  ee: (%3d, %3d, %3d)", parent_entry->xyz[0], parent_entry->xyz[1], parent_entry->xyz[2]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "  heur: %d", GetGoalHeuristic(SourceStateID));
    ROS_DEBUG_NAMED(prm_->expands_log_, "  gdiff: (%3d, %3d, %3d)",
                abs(m_goal_entry->xyz[0] - parent_entry->xyz[0]),
                abs(m_goal_entry->xyz[1] - parent_entry->xyz[1]),
                abs(m_goal_entry->xyz[2] - parent_entry->xyz[2]));
//    ROS_DEBUG_NAMED(prm_->expands_log_, "  goal dist: %0.3f", grid_->getResolution() * bfs_->getDistance(parent_entry->xyz[0], parent_entry->xyz[1], parent_entry->xyz[2]));

    const std::vector<double>& source_angles = parent_entry->state;
    visualizeState(source_angles, "expansion");

    int n_goal_succs = 0;

    std::vector<Action> actions;
    if (!as_->getActionSet(source_angles, actions)) {
        ROS_WARN("Failed to get actions");
        return;
    }

    ROS_DEBUG_NAMED(prm_->expands_log_, "  actions: %zu", actions.size());

    // check actions for validity
    std::vector<int> succ_coord(prm_->num_joints_, 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(prm_->expands_log_, "    action %zu:", i);
        ROS_DEBUG_NAMED(prm_->expands_log_, "      waypoints: %zu", action.size());

        std::uint32_t violation_mask = 0x00000000;
        int path_length = 0;
        int nchecks = 0;
        double dist = 0.0;

        // check intermediate states for collisions
        for (size_t j = 0; j < action.size(); ++j) {
            ROS_DEBUG_NAMED(prm_->expands_log_, "        %zu: %s", j, to_string(action[j]).c_str());

            // check joint limits
            if (!rmodel_->checkJointLimits(action[j])) {
                ROS_DEBUG_NAMED(prm_->expands_log_, "        -> violates joint limits");
                violation_mask |= 0x00000001;
            }

            // check for collisions
            if (!cc_->isStateValid(
                    action[j], prm_->verbose_collisions_, false, dist))
            {
                ROS_DEBUG_NAMED(prm_->expands_log_, "        -> in collision (dist: %0.3f)", dist);
                violation_mask |= 0x00000002;
            }

            if (violation_mask) {
                break;
            }
        }

        if (violation_mask) {
            continue;
        }

        // check for collisions along path from parent to first waypoint
        if (!cc_->isStateToStateValid(
                source_angles, action[0], path_length, nchecks, dist))
        {
            ROS_DEBUG_NAMED(prm_->expands_log_, "        -> path to first waypoint in collision (dist: %0.3f, path_length: %d)", dist, path_length);
            violation_mask |= 0x00000004;
        }

        if (violation_mask) {
            continue;
        }

        // check for collisions between waypoints
        for (size_t j = 1; j < action.size(); ++j) {
            if (!cc_->isStateToStateValid(
                    action[j - 1], action[j], path_length, nchecks, dist))
            {
                ROS_DEBUG_NAMED(prm_->expands_log_, "        -> path between waypoints %zu and %zu in collision (dist: %0.3f, path_length: %d)", j - 1, j, dist, path_length);
                violation_mask |= 0x00000008;
                break;
            }
        }

        if (violation_mask) {
            continue;
        }

        // compute destination coords
        anglesToCoord(action.back(), succ_coord);

        // get the successor

        // get pose of planning link
        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            continue;
        }

        // discretize planning link pose
        int endeff[3];
        grid_->worldToGrid(tgt_off_pose[0], tgt_off_pose[1], tgt_off_pose[2], endeff[0], endeff[1], endeff[2]);


        // check if this state meets the goal criteria
        const bool succ_is_goal_state = isGoal(action.back(), tgt_off_pose);
        if (succ_is_goal_state) {
            // update goal state
            m_goal_entry->coord = succ_coord;
            m_goal_entry->xyz[0] = endeff[0];
            m_goal_entry->xyz[1] = endeff[1];
            m_goal_entry->xyz[2] = endeff[2];
            m_goal_entry->state = action.back();
            m_goal_entry->dist = dist;
            n_goal_succs++;
        }

        // check if hash entry already exists, if not then create one
        EnvROBARM3DHashEntry_t* succ_entry;
        if (!(succ_entry = getHashEntry(succ_coord, succ_is_goal_state))) {
            succ_entry = createHashEntry(succ_coord, endeff);
            succ_entry->state = action.back();
            succ_entry->dist = dist;
        }

        // put successor on successor list with the proper cost
        SuccIDV->push_back(succ_entry->stateID);
        CostV->push_back(cost(parent_entry, succ_entry, succ_is_goal_state));

        // log successor details
        ROS_DEBUG_NAMED(prm_->expands_log_, "      succ: %zu", i);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        id: %5i", succ_entry->stateID);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        coord: %s", to_string(succ_coord).c_str());
        ROS_DEBUG_NAMED(prm_->expands_log_, "        state: %s", to_string(succ_entry->state).c_str());
        ROS_DEBUG_NAMED(prm_->expands_log_, "        ee: (%3d, %3d, %3d)", endeff[0], endeff[1], endeff[2]);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        pose: %s", to_string(tgt_off_pose).c_str());
        ROS_DEBUG_NAMED(prm_->expands_log_, "        gdiff: (%3d, %3d, %3d)",
                abs(m_goal_entry->xyz[0] - endeff[0]),
                abs(m_goal_entry->xyz[1] - endeff[1]),
                abs(m_goal_entry->xyz[2] - endeff[2]));
        ROS_DEBUG_NAMED(prm_->expands_log_, "        heur: %2d", GetGoalHeuristic(succ_entry->stateID));
        ROS_DEBUG_NAMED(prm_->expands_log_, "        dist: %2d", (int)succ_entry->dist);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        cost: %5d", cost(parent_entry, succ_entry, succ_is_goal_state));
    }

    if (n_goal_succs > 0) {
        ROS_DEBUG("Got %d goal successors!", n_goal_succs);
    }

    m_expanded_states.push_back(SourceStateID);
}

void ManipLattice::GetLazySuccs(
    int SourceStateID,
    std::vector<int>* SuccIDV,
    std::vector<int>* CostV,
    std::vector<bool>* isTrueCost)
{
    assert(SourceStateID >= 0 && SourceStateID < m_states.size());

    SuccIDV->clear();
    CostV->clear();
    isTrueCost->clear();

    ROS_DEBUG_NAMED(prm_->expands_log_, "expanding state %d", SourceStateID);

    if (SourceStateID == m_goal_entry->stateID) {
        return;
    }

    EnvROBARM3DHashEntry_t* state_entry = m_states[SourceStateID];

    assert(state_entry);
    assert(state_entry->coord.size() >= prm_->num_joints_);

    // log expanded state details
    ROS_DEBUG_NAMED(prm_->expands_log_, "  coord: %s", to_string(state_entry->coord).c_str());
    ROS_DEBUG_NAMED(prm_->expands_log_, "  angles: %s", to_string(state_entry->state).c_str());
    ROS_DEBUG_NAMED(prm_->expands_log_, "  ee: (%3d, %3d, %3d)", state_entry->xyz[0], state_entry->xyz[1], state_entry->xyz[2]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "  heur: %d", GetGoalHeuristic(SourceStateID));
    ROS_DEBUG_NAMED(prm_->expands_log_, "  gdiff: (%3d, %3d, %3d)",
                abs(m_goal_entry->xyz[0] - state_entry->xyz[0]),
                abs(m_goal_entry->xyz[1] - state_entry->xyz[1]),
                abs(m_goal_entry->xyz[2] - state_entry->xyz[2]));
//    ROS_DEBUG_NAMED(prm_->expands_log_, "  goal dist: %0.3f", grid_->getResolution() * bfs_->getDistance(state_entry->xyz[0], state_entry->xyz[1], state_entry->xyz[2]));

    const std::vector<double>& source_angles = state_entry->state;
    visualizeState(source_angles, "expansion");

    std::vector<Action> actions;
    if (!as_->getActionSet(source_angles, actions)) {
        ROS_WARN("Failed to get successors");
        return;
    }

    ROS_DEBUG_NAMED(prm_->expands_log_, "  actions: %zu", actions.size());

    std::vector<int> succ_coord(prm_->num_joints_);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(prm_->expands_log_, "    action %zu:", i);
        ROS_DEBUG_NAMED(prm_->expands_log_, "      waypoints: %zu", action.size());

        anglesToCoord(action.back(), succ_coord);

        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            continue;
        }

        int endeff[3];
        grid_->worldToGrid(tgt_off_pose[0], tgt_off_pose[1], tgt_off_pose[2], endeff[0], endeff[1], endeff[2]);

        const bool succ_is_goal_state = isGoal(action.back(), tgt_off_pose);
        if (succ_is_goal_state) {
            // update goal state
            m_goal_entry->coord = succ_coord;
            m_goal_entry->xyz[0] = endeff[0];
            m_goal_entry->xyz[1] = endeff[1];
            m_goal_entry->xyz[2] = endeff[2];
            m_goal_entry->state = action.back();
            m_goal_entry->dist = 0.0; //dist; // TODO: cached distance useful for anything?
        }

        // check if hash entry already exists, if not then create one
        EnvROBARM3DHashEntry_t* succ_entry;
        if (!(succ_entry = getHashEntry(succ_coord, succ_is_goal_state))) {
            succ_entry = createHashEntry(succ_coord, endeff);
            succ_entry->state = action.back();
            succ_entry->dist = 0.0; //dist;
        }

        SuccIDV->push_back(succ_entry->stateID);
        CostV->push_back(cost(state_entry, succ_entry, succ_is_goal_state));
        isTrueCost->push_back(false);

        // log successor details
        ROS_DEBUG_NAMED(prm_->expands_log_, "      succ: %zu", i);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        id: %5i", succ_entry->stateID);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        coord: %s", to_string(succ_coord).c_str());
        ROS_DEBUG_NAMED(prm_->expands_log_, "        state: %s", to_string(succ_entry->state).c_str());
        ROS_DEBUG_NAMED(prm_->expands_log_, "        ee: (%3d, %3d, %3d)", endeff[0], endeff[1], endeff[2]);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        pose: %s", to_string(tgt_off_pose).c_str());
        ROS_DEBUG_NAMED(prm_->expands_log_, "        gdiff: (%3d, %3d, %3d)",
                abs(m_goal_entry->xyz[0] - endeff[0]),
                abs(m_goal_entry->xyz[1] - endeff[1]),
                abs(m_goal_entry->xyz[2] - endeff[2]));
        ROS_DEBUG_NAMED(prm_->expands_log_, "        heur: %2d", GetGoalHeuristic(succ_entry->stateID));
        ROS_DEBUG_NAMED(prm_->expands_log_, "        dist: %2d", (int)succ_entry->dist);
        ROS_DEBUG_NAMED(prm_->expands_log_, "        cost: %5d", cost(state_entry, succ_entry, succ_is_goal_state));
    }

    m_expanded_states.push_back(SourceStateID);
}

int ManipLattice::GetTrueCost(int parentID, int childID)
{
    assert(parentID >= 0 && parentID < (int)m_states.size());
    assert(childID >= 0 && childID < (int)m_states.size());

    EnvROBARM3DHashEntry_t* parent_entry = m_states[parentID];
    EnvROBARM3DHashEntry_t* child_entry = m_states[childID];
    assert(parent_entry && parent_entry->coord.size() >= prm_->num_joints_);
    assert(child_entry && child_entry->coord.size() >= prm_->num_joints_);

    const std::vector<double>& parent_angles = parent_entry->state;
    visualizeState(parent_angles, "expansion");

    std::vector<Action> actions;
    if (!as_->getActionSet(parent_angles, actions)) {
        ROS_WARN("Failed to get actions");
        return -1;
    }

    std::vector<int> succ_coord(prm_->num_joints_);
    for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
        const Action& action = actions[aidx];
        anglesToCoord(action.back(), succ_coord);
        if (succ_coord != child_entry->coord) {
            continue;
        }

        std::uint32_t violation_mask = 0x00000000;
        int path_length = 0;
        int nchecks = 0;
        double dist = 0.0;

        // check intermediate states for collisions
        for (size_t imidx = 0; imidx < action.size(); ++imidx) {
            ROS_DEBUG_NAMED(prm_->expands_log_, "        %zu: %s", imidx, to_string(action[imidx]).c_str());

            // check joint limits
            if (!rmodel_->checkJointLimits(action[imidx])) {
                ROS_DEBUG_NAMED(prm_->expands_log_, "        -> violates joint limits");
                return -1;
            }

            // check for collisions
            if (!cc_->isStateValid(
                    action[imidx], prm_->verbose_collisions_, false, dist))
            {
                ROS_DEBUG_NAMED(prm_->expands_log_, "        -> in collision (dist: %0.3f)", dist);
                return -1;
            }
        }

        // check for collisions along path from parent to first waypoint
        if (!cc_->isStateToStateValid(
                parent_angles, action[0], path_length, nchecks, dist))
        {
            ROS_DEBUG_NAMED(prm_->expands_log_, "        -> path to first waypoint in collision (dist: %0.3f, path_length: %d)", dist, path_length);
            return -1;
        }

        // check for collisions between waypoints
        for (size_t j = 1; j < action.size(); ++j) {
            if (!cc_->isStateToStateValid(
                    action[j - 1], action[j], path_length, nchecks, dist))
            {
                ROS_DEBUG_NAMED(prm_->expands_log_, "        -> path between waypoints %zu and %zu in collision (dist: %0.3f, path_length: %d)", j - 1, j, dist, path_length);
                return -1;
            }
        }

        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            return -1;
        }
        const bool is_goal = isGoal(action.back(), tgt_off_pose);
        return cost(parent_entry, child_entry, is_goal);
    }

    return -1;
}

void ManipLattice::GetPreds(
    int TargetStateID,
    std::vector<int>* PredIDV,
    std::vector<int>* CostV)
{
    ROS_WARN("GetPreds unimplemented");
}

void ManipLattice::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    ROS_ERROR("SetAllActionsandAllOutcomes unimplemented");
}

void ManipLattice::SetAllPreds(CMDPSTATE* state)
{
    ROS_ERROR("SetAllPreds unimplemented");
}

void ManipLattice::printHashTableHist()
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int  j = 0; j < m_HashTableSize; j++) {
        if ((int)m_Coord2StateIDHashTable[j].size() == 0) {
            s0++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 50) {
            s1++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 100) {
            s50++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 200) {
            s100++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 300) {
            s200++;
        }
        else if ((int)m_Coord2StateIDHashTable[j].size() < 400) {
            s300++;
        }
        else {
            slarge++;
        }
    }
    ROS_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d", s0,s1, s50, s100, s200,s300,slarge);
}

EnvROBARM3DHashEntry_t* ManipLattice::getHashEntry(
    const std::vector<int>& coord,
    bool bIsGoal)
{
    // if it is goal
    if (bIsGoal) {
        return m_goal_entry;
    }

    int binid = getHashBin(coord);

#if DEBUG
    if ((int)m_Coord2StateIDHashTable[binid].size() > 500) {
        ROS_WARN("WARNING: Hash table has a bin %d (coord0=%d) of size %d", binid, coord[0], int(m_Coord2StateIDHashTable[binid].size()));
        printHashTableHist();
    }
#endif

    // iterate over the states in the bin and select the perfect match
    for (int ind = 0; ind < (int)m_Coord2StateIDHashTable[binid].size(); ind++) {
        int j = 0;

        for (j = 0; j < int(coord.size()); j++) {
            if (m_Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) {
                break;
            }
        }

        if (j == int(coord.size())) {
            return m_Coord2StateIDHashTable[binid][ind];
        }
    }

    return NULL;
}

bool ManipLattice::isGoal(int state_id) const
{
    return state_id == m_goal_entry->stateID;
}

bool ManipLattice::computePlanningFrameFK(
    const std::vector<double>& state,
    std::vector<double>& pose) const
{
    assert(state.size() == prm_->num_joints_);

    if (!rmodel_->computePlanningLinkFK(state, pose)) {
        return false;
    }

    // pose represents T_planning_eef
    Eigen::Affine3d T_planning_tipoff =  // T_planning_eef * T_eef_tipoff
            Eigen::Translation3d(pose[0], pose[1], pose[2]) *
            Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
            Eigen::Translation3d(
                    m_goal.xyz_offset[0],
                    m_goal.xyz_offset[1],
                    m_goal.xyz_offset[2]);
    const Eigen::Vector3d voff(T_planning_tipoff.translation());
    pose[0] = voff.x();
    pose[1] = voff.y();
    pose[2] = voff.z();

    assert(pose.size() == 6);
    return true;
}

EnvROBARM3DHashEntry_t* ManipLattice::createHashEntry(
    const std::vector<int>& coord,
    int endeff[3])
{
    int i;
    EnvROBARM3DHashEntry_t* HashEntry = new EnvROBARM3DHashEntry_t;

    HashEntry->coord = coord;

    memcpy(HashEntry->xyz, endeff, 3 * sizeof(int));

    // assign a stateID to HashEntry to be used
    HashEntry->stateID = m_states.size();

    // insert into the tables
    m_states.push_back(HashEntry);

    // get the hash table bin
    i = getHashBin(HashEntry->coord);

    // insert the entry into the bin
    m_Coord2StateIDHashTable[i].push_back(HashEntry);

    // insert into and initialize the mappings
    int* entry = new int [NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size()-1) {
        ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
        throw new SBPL_Exception();
    }
    return HashEntry;
}

int ManipLattice::cost(
    EnvROBARM3DHashEntry_t* HashEntry1,
    EnvROBARM3DHashEntry_t* HashEntry2,
    bool bState2IsGoal)
{
    return prm_->cost_multiplier_;
}

bool ManipLattice::initEnvironment(ManipHeuristic* heur)
{
    if (!heur) {
        ROS_ERROR("Heuristic is null");
        return false;
    }

    m_heur = heur;

    ROS_INFO("Initializing environment");

    // initialize environment data
    m_Coord2StateIDHashTable = new std::vector<EnvROBARM3DHashEntry_t*>[m_HashTableSize];
    m_states.clear();

    // create empty start & goal states
    int endeff[3] = { 0 };
    std::vector<int> coord(prm_->num_joints_, 0);
    m_start_entry = nullptr;
    m_goal_entry = createHashEntry(coord, endeff);
    ROS_INFO("  goal state has state ID %d", m_goal_entry->stateID);

    // compute the cost per cell to be used by heuristic
    computeCostPerCell();

    // set 'environment is initialized' flag
    m_initialized = true;
    ROS_INFO("Environment has been initialized.");
    return true;
}

bool ManipLattice::isGoalState(
    const std::vector<double>& pose, // tgt_off_pose
    GoalConstraint& goal)
{
    if (goal.type == XYZ_RPY_GOAL) {
        if (fabs(pose[0] - goal.tgt_off_pose[0]) <= goal.xyz_tolerance[0] &&
            fabs(pose[1] - goal.tgt_off_pose[1]) <= goal.xyz_tolerance[1] &&
            fabs(pose[2] - goal.tgt_off_pose[2]) <= goal.xyz_tolerance[2])
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                m_time_to_goal_region = (clock() - m_t_start) / (double)CLOCKS_PER_SEC;
                m_near_goal = true;
                ROS_INFO("Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %.4f sec. (after %d expansions)",
                        pose[0], pose[1], pose[2], goal.xyz_tolerance[0], goal.tgt_off_pose[0], goal.tgt_off_pose[1], goal.tgt_off_pose[2], m_time_to_goal_region, (int)m_expanded_states.size());
            }
            const double droll = fabs(angles::shortest_angular_distance(pose[3], goal.tgt_off_pose[3]));
            const double dpitch = fabs(angles::shortest_angular_distance(pose[4], goal.tgt_off_pose[4]));
            const double dyaw = fabs(angles::shortest_angular_distance(pose[5], goal.tgt_off_pose[5]));
            ROS_DEBUG("Near goal! (%0.3f, %0.3f, %0.3f)", droll, dpitch, dyaw);
            if (droll < goal.rpy_tolerance[0] &&
                dpitch < goal.rpy_tolerance[1] &&
                dyaw < goal.rpy_tolerance[2])
            {
                return true;
            }
        }
    }
    else if (goal.type == XYZ_GOAL) {
        if (fabs(pose[0] - goal.tgt_off_pose[0]) <= goal.xyz_tolerance[0] &&
            fabs(pose[1] - goal.tgt_off_pose[1]) <= goal.xyz_tolerance[1] &&
            fabs(pose[2] - goal.tgt_off_pose[2]) <= goal.xyz_tolerance[2])
        {
            return true;
        }
    }
    else {
        ROS_ERROR("Unknown goal type.");
    }

    return false;
}

bool ManipLattice::isGoalState(
    const std::vector<double>& angles,
    GoalConstraint7DOF& goal)
{
    if (!m_use_7dof_goal) {
        SBPL_WARN("using 7dof isGoalState checking, but not using 7dof goal!");
    }

    for (int i = 0; i < goal.angles.size(); i++) {
        if (fabs(angles[i] - goal.angles[i]) > goal.angle_tolerances[i]) {
            return false;
        }
    }
    return true;
}

bool ManipLattice::isGoal(
    const std::vector<double>& angles,
    const std::vector<double>& pose)
{
    return (!m_use_7dof_goal && isGoalState(pose, m_goal)) ||
            (m_use_7dof_goal && isGoalState(angles, m_goal_7dof));
}

int ManipLattice::getActionCost(
    const std::vector<double>& from_config,
    const std::vector<double>& to_config,
    int dist)
{
    int num_prims = 0, cost = 0;
    double diff = 0, max_diff = 0;

    if (from_config.size() != to_config.size()) {
        return -1;
    }

    /* NOTE: Not including forearm roll OR wrist roll movement to calculate mprim cost */

    for (size_t i = 0; i < 6; i++) {
        if (i == 4) {
            continue;
        }

        diff = fabs(angles::shortest_angular_distance(from_config[i], to_config[i]));
        if (max_diff < diff) {
            max_diff = diff;
        }
    }

    num_prims = max_diff / prm_->max_mprim_offset_ + 0.5;
    cost = num_prims * prm_->cost_multiplier_;

    std::vector<double> from_config_norm(from_config.size());
    for (size_t i = 0; i < from_config.size(); ++i) {
        from_config_norm[i] = angles::normalize_angle(from_config[i]);
    }
    ROS_DEBUG_NAMED("search", "from: %s", to_string(from_config_norm).c_str());
    ROS_DEBUG_NAMED("search", "  to: %s diff: %0.2f num_prims: %d cost: %d (mprim_size: %0.3f)", to_string(to_config).c_str(), max_diff, num_prims, cost, prm_->max_mprim_offset_);

    return cost;
}

bool ManipLattice::setStartConfiguration(
    const std::vector<double>& angles)
{
    ROS_INFO("Setting the start state");

    if ((int)angles.size() < prm_->num_joints_) {
        ROS_ERROR("Start state does not contain enough enough joint positions.");
        return false;
    }

    ROS_INFO("  angles: %s", to_string(angles).c_str());

    //get joint positions of starting configuration
    std::vector<double> pose(6, 0.0);
    if (!computePlanningFrameFK(angles, pose)) {
        ROS_WARN(" -> unable to compute forward kinematics");
        return false;
    }
    ROS_INFO("  planning link pose: { x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f }", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    // check joint limits of starting configuration
    if (!rmodel_->checkJointLimits(angles, true)) {
        ROS_WARN(" -> violates the joint limits");
        return false;
    }

    // check if the start configuration is in collision
    double dist = 0;
    if (!cc_->isStateValid(angles, true, false, dist)) {
        ROS_WARN(" -> in collision (distance to nearest obstacle %0.3fm)", dist * grid_->getResolution());
        return false;
    }

    visualizeState(angles, "start_config");

    //get arm position in environment
    std::vector<int> start_coord(prm_->num_joints_);
    anglesToCoord(angles, start_coord);
    ROS_INFO("  coord: %s", to_string(start_coord).c_str());

    int endeff[3];
    grid_->worldToGrid(pose[0], pose[1], pose[2], endeff[0], endeff[1], endeff[2]);
    ROS_INFO("  pose: (%d, %d, %d)", endeff[0], endeff[1], endeff[2]);
    // TODO: check for within grid bounds?

    EnvROBARM3DHashEntry_t* start_entry;
    if (!(start_entry = getHashEntry(start_coord, false))) {
        start_entry = createHashEntry(start_coord, endeff);
        start_entry->state = angles;
        start_entry->dist = dist;
    }

    m_start_entry = start_entry;

    return true;
}

bool ManipLattice::setGoalConfiguration(
    const std::vector<double>& goal,
    const std::vector<double>& goal_tolerances)
{
    if (!m_initialized) {
        ROS_ERROR("Cannot set goal position because environment is not initialized.");
        return false;
    }

    // compute the goal pose
    std::vector<std::vector<double>> goals_6dof;
    std::vector<double> pose;
    if (!computePlanningFrameFK(goal, pose)) {
        SBPL_WARN("Could not compute planning link FK for given goal configuration!");
        return false;
    }
    goals_6dof.push_back(pose);

    std::vector<std::vector<double>> offsets_6dof(1, std::vector<double>(3, 0.0));

    // made up goal tolerance (it should not be used in with 7dof goals anyways)
    std::vector<std::vector<double>> tolerances_6dof(1, std::vector<double>(6, 0.05));

    if (!setGoalPosition(goals_6dof, offsets_6dof, tolerances_6dof)) {
	   ROS_WARN("Failed to set goal position");
	   return false;
    }

    // fill in m_goal
    m_goal_7dof.angles = goal;
    m_goal_7dof.angle_tolerances = goal_tolerances;
    m_use_7dof_goal = true;
    return true;
}

bool ManipLattice::setGoalPosition(
    const std::vector<std::vector<double>>& goals,
    const std::vector<std::vector<double>>& offsets,
    const std::vector<std::vector<double>>& tolerances)
{
    if (!m_initialized) {
        ROS_ERROR("Cannot set goal position because environment is not initialized.");
        return false;
    }

    // check arguments

    if (goals.empty()) {
        ROS_ERROR("goals vector is empty");
        return false;
    }

    for (const auto& goal : goals) {
        if (goal.size() != 7) {
            ROS_ERROR("goal element has incorrect format");
            return false;
        }
    }

    if (offsets.size() != goals.size()) {
        ROS_ERROR("setGoalPosition requires as many offset elements as goal elements");
        return false;
    }

    for (const auto& offset : offsets) {
        if (offset.size() != 3) {
            ROS_ERROR("offset element has incorrect format");
            return false;
        }
    }

    if (tolerances.size() != goals.size()) {
        ROS_ERROR("setGoalPosition requires as many tolerance elements as goal elements");
        return false;
    }

    for (const auto& tol : tolerances) {
        if (tol.size() != 6) {
            ROS_ERROR("tolerance element has incorrect format");
            return false;
        }
    }

    m_use_7dof_goal = false;

    m_goal.pose.resize(6, 0.0);
    m_goal.pose[0] = goals[0][0];
    m_goal.pose[1] = goals[0][1];
    m_goal.pose[2] = goals[0][2];
    m_goal.pose[3] = goals[0][3];
    m_goal.pose[4] = goals[0][4];
    m_goal.pose[5] = goals[0][5];

    m_goal.xyz_offset[0] = offsets[0][0];
    m_goal.xyz_offset[1] = offsets[0][1];
    m_goal.xyz_offset[2] = offsets[0][2];

    m_goal.xyz_tolerance[0] = tolerances[0][0];
    m_goal.xyz_tolerance[1] = tolerances[0][1];
    m_goal.xyz_tolerance[2] = tolerances[0][2];
    m_goal.rpy_tolerance[0] = tolerances[0][3];
    m_goal.rpy_tolerance[1] = tolerances[0][4];
    m_goal.rpy_tolerance[2] = tolerances[0][5];
    m_goal.type = (GoalType)((int)goals[0][6]);

    std::vector<double> tgt_off_pose = getTargetOffsetPose(m_goal.pose);
    m_goal.tgt_off_pose = tgt_off_pose;

    auto goal_markers = viz::getPosesMarkerArray(
            { tgt_off_pose },
            grid_->getReferenceFrame(),
            "target_goal");
    m_vpub.publish(goal_markers);

    int eexyz[3];
    grid_->worldToGrid(
            m_goal.pose[0], m_goal.pose[1], m_goal.pose[2],
            eexyz[0], eexyz[1], eexyz[2]);

    // set goal hash entry
    grid_->worldToGrid(
            tgt_off_pose[0], tgt_off_pose[1], tgt_off_pose[2],
            m_goal_entry->xyz[0], m_goal_entry->xyz[1], m_goal_entry->xyz[2]);

    for (int i = 0; i < prm_->num_joints_; i++) {
        m_goal_entry->coord[i] = 0;
    }

    ROS_DEBUG_NAMED(prm_->expands_log_, "time: %f", clock() / (double)CLOCKS_PER_SEC);
    ROS_DEBUG_NAMED(prm_->expands_log_, "A new goal has been set.");
    ROS_DEBUG_NAMED(prm_->expands_log_, "    grid (cells): (%d, %d, %d)", m_goal_entry->xyz[0], m_goal_entry->xyz[1], m_goal_entry->xyz[2]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "    xyz (meters): (%0.2f, %0.2f, %0.2f)", m_goal.pose[0], m_goal.pose[1], m_goal.pose[2]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "    tol (meters): %0.3f", m_goal.xyz_tolerance[0]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "    rpy (radians): (%0.2f, %0.2f, %0.2f)", m_goal.pose[3], m_goal.pose[4], m_goal.pose[5]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "    tol (radians): %0.3f", m_goal.rpy_tolerance[0]);

    m_near_goal = false;
    m_t_start = clock();
    return true;
}

void ManipLattice::StateID2Angles(
    int stateID,
    std::vector<double>& angles) const
{
    EnvROBARM3DHashEntry_t* HashEntry = m_states[stateID];

    if (stateID == m_goal_entry->stateID) {
        coordToAngles(m_goal_entry->coord, angles);
    }
    else {
        coordToAngles(HashEntry->coord, angles);
    }

    for (size_t i = 0; i < angles.size(); i++) {
        if (angles[i] >= M_PI) {
            angles[i] = -2.0 * M_PI + angles[i];
        }
    }
}

void ManipLattice::printJointArray(
    FILE* fOut,
    EnvROBARM3DHashEntry_t* HashEntry,
    bool bGoal,
    bool bVerbose)
{
    std::vector<double> angles(prm_->num_joints_, 0.0);

    if (bGoal) {
        coordToAngles(m_goal_entry->coord, angles);
    }
    else {
        coordToAngles(HashEntry->coord, angles);
    }

    std::stringstream ss;
    if (bVerbose) {
        ss << "angles: ";
    }
    ss << "{ ";
    for (size_t i = 0; i < angles.size(); ++i) {
        ss << std::setprecision(3) << angles[i];
        if (i != angles.size() - 1) {
            ss << ", ";
        }
    }
    ss << " }";

    if (fOut == stdout) {
        ROS_INFO("%s", ss.str().c_str());
    }
    else if (fOut == stderr) {
        ROS_WARN("%s", ss.str().c_str());
    }
    else {
        fprintf(fOut, "%s\n", ss.str().c_str());
    }
}

void ManipLattice::getExpandedStates(
    std::vector<std::vector<double>>& states) const
{
    std::vector<double> angles(prm_->num_joints_,0);
    std::vector<double> state(7, 0); // { x, y, z, r, p, y, heur }

    for (size_t i = 0; i < m_expanded_states.size(); ++i) {
        StateID2Angles(m_expanded_states[i], angles);
        computePlanningFrameFK(angles, state);
        state[6] = m_states[m_expanded_states[i]]->heur;
        states.push_back(state);
        ROS_DEBUG("[%d] id: %d  xyz: %s", int(i), m_expanded_states[i], to_string(state).c_str());
    }
}

void ManipLattice::computeCostPerCell()
{
    ROS_WARN("Cell Cost: Uniform 100");
    prm_->cost_per_cell_ = 100;
}

void ManipLattice::convertStateIDPathToJointAnglesPath(
    const std::vector<int>& idpath,
    std::vector<std::vector<double>>& path) const
{

}

bool ManipLattice::convertStateIDPathToJointTrajectory(
    const std::vector<int>& idpath,
    trajectory_msgs::JointTrajectory& traj) const
{
    if (idpath.empty()) {
        return false;
    }

    traj.header.frame_id = prm_->planning_frame_;
    traj.joint_names = prm_->planning_joints_;
    traj.points.resize(idpath.size());

    std::vector<double> angles;
    for (size_t i = 0; i < idpath.size(); ++i) {
        traj.points[i].positions.resize(prm_->num_joints_);
        StateID2Angles(idpath[i], angles);

        for (int p = 0; p < prm_->num_joints_; ++p) {
            traj.points[i].positions[p] = angles::normalize_angle(angles[p]);
        }
    }
    return true;
}

void ManipLattice::convertStateIDPathToShortenedJointAnglesPath(
    const std::vector<int>& idpath,
    std::vector<std::vector<double>>& path,
    std::vector<int>& idpath_short)
{
}

double ManipLattice::getStartDistance(double x, double y, double z)
{
    return m_heur->getMetricStartDistance(x, y, z);
}

double ManipLattice::getStartDistance(const std::vector<double>& pose)
{
    std::vector<double> tipoff_pose = getTargetOffsetPose(pose);
    return getStartDistance(tipoff_pose[0], tipoff_pose[1], tipoff_pose[2]);
}

double ManipLattice::getGoalDistance(double x, double y, double z)
{
    return m_heur->getMetricGoalDistance(x, y, z);
}

double ManipLattice::getGoalDistance(const std::vector<double>& pose)
{
    std::vector<double> tipoff_pose = getTargetOffsetPose(pose);
    return getGoalDistance(tipoff_pose[0], tipoff_pose[1], tipoff_pose[2]);
}

const EnvROBARM3DHashEntry_t* ManipLattice::getHashEntry(
    int state_id) const
{
    if (state_id < 0 || state_id >= m_states.size()) {
        return nullptr;
    }

    return m_states[state_id];
}

int ManipLattice::getGoalStateID() const
{
    return m_goal_entry ? m_goal_entry->stateID : -1;
}

int ManipLattice::getStartStateID() const
{
    return m_start_entry ? m_start_entry->stateID : -1;
}

const std::vector<double>& ManipLattice::getGoal() const
{
    return m_goal.pose;
}

std::vector<double> ManipLattice::getTargetOffsetPose(
    const std::vector<double>& tip_pose) const
{
    // pose represents T_planning_eef
    Eigen::Affine3d T_planning_tipoff = // T_planning_eef * T_eef_tipoff
            Eigen::Translation3d(tip_pose[0], tip_pose[1], tip_pose[2]) *
            Eigen::AngleAxisd(tip_pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(tip_pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(tip_pose[3], Eigen::Vector3d::UnitX()) *
            Eigen::Translation3d(
                    m_goal.xyz_offset[0],
                    m_goal.xyz_offset[1],
                    m_goal.xyz_offset[2]);
    const Eigen::Vector3d voff(T_planning_tipoff.translation());
    return { voff.x(), voff.y(), voff.z(), tip_pose[3], tip_pose[4], tip_pose[5] };
}

const GoalConstraint& ManipLattice::getCartesianGoal() const
{
    return m_goal;
}

std::vector<double> ManipLattice::getGoalConfiguration() const
{
    return m_goal_7dof.angles;
}

const GoalConstraint7DOF& ManipLattice::getJointGoal() const
{
    return m_goal_7dof;
}

std::vector<double> ManipLattice::getStartConfiguration() const
{
    if (m_start_entry) {
        return m_start_entry->state;
    }
    else {
        return std::vector<double>();
    }
}

void ManipLattice::visualizeState(
    const std::vector<double>& jvals,
    const std::string& ns) const
{
    visualization_msgs::MarkerArray ma =
            cc_->getCollisionModelVisualization(jvals);
    for (auto& marker : ma.markers) {
        marker.ns = ns;
    }
    m_vpub.publish(ma);
}

inline
unsigned int ManipLattice::intHash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

inline
unsigned int ManipLattice::getHashBin(
    const std::vector<int>& coord)
{
    int val = 0;

    for (size_t i = 0; i < coord.size(); i++) {
        val += intHash(coord[i]) << i;
    }

    return intHash(val) & (m_HashTableSize - 1);
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline
void ManipLattice::coordToAngles(
    const std::vector<int>& coord,
    std::vector<double>& angles) const
{
    angles.resize(coord.size());
    for (size_t i = 0; i < coord.size(); i++) {
        if (m_continuous[i]) {
            angles[i] = coord[i] * prm_->coord_delta_[i];
        }
        else {
            angles[i] = m_min_limits[i] + coord[i] * prm_->coord_delta_[i];
        }
    }
}

inline
void ManipLattice::anglesToCoord(
    const std::vector<double>& angle,
    std::vector<int>& coord) const
{
    assert((int)angle.size() == prm_->num_joints_ &&
            (int)coord.size() == prm_->num_joints_);

    double pos_angle;

    for (size_t i = 0; i < angle.size(); i++) {
        if (m_continuous[i]) {
            pos_angle = angle[i];
            if (pos_angle < 0.0) {
                pos_angle += 2 * M_PI;
            }

            coord[i] = (int)((pos_angle + prm_->coord_delta_[i] * 0.5) / prm_->coord_delta_[i]);

            if (coord[i] == prm_->coord_vals_[i]) {
                coord[i] = 0;
            }
        }
        else {
            coord[i] = (int)(((angle[i] - m_min_limits[i]) / prm_->coord_delta_[i]) + 0.5);
        }
    }
}

} // namespace manip
} // namespace sbpl
