////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#include <smpl/search/arastar.h>

// system includes
#include <ros/console.h>
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>

namespace sbpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

ARAStar::ARAStar(
    DiscreteSpaceInformation* space,
    Heuristic* heur)
:
    SBPLPlanner(),
    m_space(space),
    m_heur(heur),
    m_states(),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_graph_to_search_map(),
    m_open(),
    m_eps(1.0),
    m_iteration(1),
    m_call_number(0),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1),
    m_last_eps(1.0),
    m_expand_count(0),
    m_search_time(clock::duration::zero())
{
    environment_ = space;

    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimingParameters::TIME;
    m_time_params.min_expansions = 0;
    m_time_params.max_expansions = 0;
    m_time_params.min_allowed_time = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
}

ARAStar::~ARAStar()
{
    for (SearchState* s : m_states) {
        delete s;
    }
}

int ARAStar::replan(
    double allowed_time,
    std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

// decide whether to start the search from scratch
//
// if start changed
//     clear the search tree
//     clear the open list
//     clear the closed list
//     set g values to inf
//     set f values to inf
//     set g(start) to 0
//     set f(start) to g + eps * h
//     insert s_start into OPEN with priority f(start)
// if goal changed
//     reevaluate heuristics
//     reorder the open list
//
// case scenario_hasnt_changed (start and goal the same)
//   case have solution for previous epsilon
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           pass
//   case dont have solution
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           reevaluate heuristics and reorder the open list
// case scenario_changed
//
// assumptions:
//
//      state ids for states are constant
//
//          this is not the case if, for instance, states are generated
//          incrementally and assigned incremental ids
//
//      changes to the goal are reflected in changes to the goal state id
//
//          since the search algorithm requires exactly one id to represent
//          the goal state, most representations that support underdefined
//          goals will offer up a single state id for the abstract goal
//          state that never changes
//
//      heuristic values haven't changed
//
//          currently no way to tell whether this is the case other than
//          via changes to the goal state id. it is assumed that heuristics
//          remain constant within the context of a single goal
int ARAStar::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    m_time_params.max_allowed_time = to_duration(allowed_time);

    ROS_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0 || m_goal_state_id < 0) {
        ROS_ERROR_NAMED(SLOG, "Start or goal state not set");
        return 1;
    }

    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    if (m_start_state_id != m_last_start_state_id) {
        ROS_DEBUG_NAMED(SLOG, "Reinitialize search");
        m_open.clear();
        ++m_call_number; // trigger state reinitializations

        reinitSearchState(start_state);
        reinitSearchState(goal_state);

        start_state->g = 0;
        start_state->f = computeKey(start_state);
        m_open.push(start_state);

        m_iteration = 1;

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_last_start_state_id = m_start_state_id;
    }

    if (m_goal_state_id != m_last_goal_state_id) {
        ROS_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        recomputeHeuristics();
        recomputeKeys();
        reorderOpenList();

        m_last_goal_state_id = m_goal_state_id;
    }

    auto start_time = clock::now();
    int expansion_count = 0;

    std::vector<int> succs;
    std::vector<int> costs;

    while (!m_open.empty()) {
        SearchState* min_state = m_open.min();

        auto now = clock::now();
        auto elapsed = now - start_time;

        // path to goal found
        if (min_state->f >= goal_state->f || min_state == goal_state) {
            ROS_DEBUG_NAMED(SLOG, "Found path to goal");
            extractPath(goal_state, *solution, *cost);
            m_search_time += elapsed;
            m_expand_count += expansion_count;
            return true;
        }

        if (timedOut(expansion_count, elapsed)) {
            ROS_DEBUG_NAMED(SLOG, "Ran out of time");
            break;
        }

        ROS_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

        m_open.pop();

        min_state->iteration_closed = m_iteration;

        succs.clear();
        costs.clear();
        m_space->GetSuccs(min_state->state_id, &succs, &costs);

        ROS_DEBUG_NAMED(SELOG, "  %zu successors", succs.size());

        for (size_t sidx = 0; sidx < succs.size(); ++sidx) {
            int succ_state_id = succs[sidx];
            int cost = costs[sidx];

            SearchState* succ_state = getSearchState(succ_state_id);
            reinitSearchState(succ_state);

            if (succ_state->iteration_closed == m_iteration) {
                continue;
            }

            int new_cost = min_state->g + cost;
            ROS_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
            if (new_cost < succ_state->g) {
                succ_state->g = new_cost;
                succ_state->f = computeKey(succ_state);
                succ_state->bp = min_state;
                if (m_open.contains(succ_state)) {
                    m_open.decrease(succ_state);
                } else {
                    m_open.push(succ_state);
                }
            }
        }

        ++expansion_count;
    }

    m_search_time += (clock::now() - start_time);
    m_expand_count += expansion_count;
    return false;
}

int ARAStar::replan(
    std::vector<int>* solution,
    ReplanParams params)
{
    int cost;
    return replan(params.max_time, solution, &cost);
}

int ARAStar::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* cost)
{
    return replan(params.max_time, solution, cost);
}

int ARAStar::force_planning_from_scratch_and_free_memory()
{
    force_planning_from_scratch();
    m_open.clear();
    m_graph_to_search_map.clear();
    m_graph_to_search_map.shrink_to_fit();
    for (SearchState* s : m_states) {
        delete s;
    }
    m_states.clear();
    m_states.shrink_to_fit();
    return 0;
}

double ARAStar::get_solution_eps() const
{
    return m_eps;
}

int ARAStar::get_n_expands() const
{
    return m_expand_count;
}

double ARAStar::get_initial_eps()
{
    return m_eps;
}

double ARAStar::get_initial_eps_planning_time()
{
    return to_seconds(m_search_time);
}

double ARAStar::get_final_eps_planning_time()
{
    return to_seconds(m_search_time);
}

int ARAStar::get_n_expands_init_solution()
{
    return m_expand_count;
}

double ARAStar::get_final_epsilon()
{
    return m_eps;
}

void ARAStar::get_search_stats(std::vector<PlannerStats>* s)
{
    PlannerStats stats;
    stats.eps = m_eps;
    stats.cost;
    stats.expands = m_expand_count;
    stats.time = to_seconds(m_search_time);
    s->push_back(stats);
}

void ARAStar::set_initialsolution_eps(double eps)
{
    m_eps = eps;
}

int ARAStar::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

int ARAStar::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

int ARAStar::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    return 0;
}

int ARAStar::set_search_mode(bool first_solution_unbounded)
{
    return 0;
}

void ARAStar::costs_changed(const StateChangeQuery& state_change)
{
    force_planning_from_scratch();
}

void ARAStar::recomputeHeuristics()
{
    for (SearchState* s : m_states) {
        s->h = m_heur->GetGoalHeuristic(s->state_id);
    }
}

bool ARAStar::timedOut(
    int elapsed_expansions,
    const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimingParameters::EXPANSIONS:
        return elapsed_expansions >= m_time_params.max_expansions;
    case TimingParameters::TIME:
        return elapsed_time >= m_time_params.max_allowed_time;
    default:
        return true;
    }

    return true;
}

void ARAStar::recomputeKeys()
{
    for (SearchState* s : m_states) {
        s->f = computeKey(s);
    }
}

void ARAStar::reorderOpenList()
{
    m_open.make();
}

int ARAStar::computeKey(SearchState* s) const
{
    return s->g + (unsigned int)(m_eps * s->h);
}

ARAStar::SearchState*
ARAStar::getSearchState(int state_id)
{
    if (m_graph_to_search_map.size() <= state_id) {
        m_graph_to_search_map.resize(state_id + 1, -1);
    }

    if (m_graph_to_search_map[state_id] == -1) {
        return createState(state_id);
    } else {
        return m_states[m_graph_to_search_map[state_id]];
    }
}

ARAStar::SearchState*
ARAStar::createState(int state_id)
{
    assert(state_id < m_graph_to_search_map.size());

    m_graph_to_search_map[state_id] = (int)m_states.size();

    SearchState* ss = new SearchState;
    ss->state_id = state_id;
    ss->call_number = 0;
    m_states.push_back(ss);

    return ss;
}

void ARAStar::reinitSearchState(SearchState* state)
{
    if (state->call_number != m_call_number) {
        ROS_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        state->h = m_heur->GetGoalHeuristic(state->state_id);
        state->f = INFINITECOST;
        state->iteration_closed = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
    }
}

void ARAStar::extractPath(
    SearchState* to_state,
    std::vector<int>& solution,
    int& cost) const
{
    for (SearchState* s = to_state; s; s = s->bp) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

} // namespace sbpl
