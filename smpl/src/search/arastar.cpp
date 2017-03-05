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
    m_time_params(),
    m_initial_eps(1.0),
    m_final_eps(1.0),
    m_delta_eps(1.0),
    m_allow_partial_solutions(false),
    m_states(),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_graph_to_search_map(),
    m_open(),
    m_incons(),
    m_curr_eps(1.0),
    m_iteration(1),
    m_call_number(0),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1),
    m_last_eps(1.0),
    m_expand_count_init(0),
    m_expand_count(0),
    m_search_time_init(clock::duration::zero()),
    m_search_time(clock::duration::zero()),
    m_satisfied_eps(std::numeric_limits<double>::infinity())
{
    environment_ = space;

    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimingParameters::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
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
//     reset the search to its initial state
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
enum ReplanResultCode
{
    SUCCESS = 0,
    START_NOT_SET,
    GOAL_NOT_SET,
    TIMED_OUT,
    EXHAUSTED_OPEN_LIST
};

int ARAStar::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    ROS_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0) {
        ROS_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal_state_id < 0) {
        ROS_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    m_time_params.max_allowed_time = to_duration(allowed_time);
    m_time_params.max_allowed_time_init = std::max(m_time_params.max_allowed_time_init, to_duration(allowed_time));

    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    if (m_start_state_id != m_last_start_state_id) {
        ROS_DEBUG_NAMED(SLOG, "Reinitialize search");
        m_open.clear();
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations

        reinitSearchState(start_state);
        reinitSearchState(goal_state);

        start_state->g = 0;
        start_state->f = computeKey(start_state);
        m_open.push(start_state);

        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;
    }

    if (m_goal_state_id != m_last_goal_state_id) {
        ROS_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        recomputeHeuristics();
        reorderOpen();

        m_last_goal_state_id = m_goal_state_id;
    }

    auto start_time = clock::now();
    int num_expansions = 0;
    clock::duration elapsed_time = clock::duration::zero();

    int err;
    while (m_satisfied_eps > m_final_eps) {
        if (m_curr_eps == m_satisfied_eps && m_time_params.improve) {
            // begin a new search iteration
            ++m_iteration;
            m_curr_eps -= m_delta_eps;
            m_curr_eps = std::max(m_curr_eps, m_final_eps);
            for (SearchState* s : m_incons) {
                s->incons = false;
                m_open.push(s);
            }
            reorderOpen();
            m_incons.clear();
            ROS_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
        }
        err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
        if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
        }
        if (err) {
            break;
        }
        ROS_DEBUG_NAMED(SLOG, "Improved solution");
        m_satisfied_eps = m_curr_eps;
    }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

    if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        return !err;
    }

    extractPath(goal_state, *solution, *cost);
    return !SUCCESS;
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

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
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

/// Return the suboptimality bound of the current solution for the current search.
double ARAStar::get_solution_eps() const
{
    return m_satisfied_eps;
}

/// Return the number of expansions made in progress to the final solution.
int ARAStar::get_n_expands() const
{
    return m_expand_count;
}

/// Return the initial suboptimality bound
double ARAStar::get_initial_eps()
{
    return m_initial_eps;
}

/// Return the time consumed by the search in progress to the initial solution.
double ARAStar::get_initial_eps_planning_time()
{
    return to_seconds(m_search_time_init);
}

/// Return the time consumed by the search in progress to the final solution.
double ARAStar::get_final_eps_planning_time()
{
    return to_seconds(m_search_time);
}

/// Return the number of expansions made in progress to the initial solution.
int ARAStar::get_n_expands_init_solution()
{
    return m_expand_count_init;
}

/// Return the final suboptimality bound.
double ARAStar::get_final_epsilon()
{
    return m_final_eps;
}

/// Return statistics for each completed search iteration.
void ARAStar::get_search_stats(std::vector<PlannerStats>* s)
{
    PlannerStats stats;
    stats.eps = m_curr_eps;
    stats.cost;
    stats.expands = m_expand_count;
    stats.time = to_seconds(m_search_time);
    s->push_back(stats);
}

/// Set the desired suboptimality bound for the initial solution.
void ARAStar::set_initialsolution_eps(double eps)
{
    m_initial_eps = eps;
}

/// Set the goal state.
int ARAStar::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

/// Set the start state.
int ARAStar::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

/// Force the search to forget previous search efforts and start from scratch.
int ARAStar::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    return 0;
}

/// Set whether the number of expansions is bounded by time or total expansions
/// per call to replan().
int ARAStar::set_search_mode(bool first_solution_unbounded)
{
    m_time_params.bounded = !first_solution_unbounded;
    return 0;
}

/// Notify the search of changes to edge costs in the graph.
void ARAStar::costs_changed(const StateChangeQuery& changes)
{
    force_planning_from_scratch();
}

// Recompute heuristics for all states.
void ARAStar::recomputeHeuristics()
{
    for (SearchState* s : m_states) {
        s->h = m_heur->GetGoalHeuristic(s->state_id);
    }
}

// Test whether the search has run out of time.
bool ARAStar::timedOut(
    int elapsed_expansions,
    const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimingParameters::EXPANSIONS:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= m_time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= m_time_params.max_expansions;
        }
    case TimingParameters::TIME:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= m_time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= m_time_params.max_allowed_time;
        }
    default:
        ROS_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
int ARAStar::improvePath(
    const clock::time_point& start_time,
    SearchState* goal_state,
    int& elapsed_expansions,
    clock::duration& elapsed_time)
{
    std::vector<int> succs;
    std::vector<int> costs;
    while (!m_open.empty()) {
        SearchState* min_state = m_open.min();

        auto now = clock::now();
        elapsed_time = now - start_time;

        // path to goal found
        if (min_state->f >= goal_state->f || min_state == goal_state) {
            ROS_DEBUG_NAMED(SLOG, "Found path to goal");
            return SUCCESS;
        }

        if (timedOut(elapsed_expansions, elapsed_time)) {
            ROS_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }

        ROS_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

        m_open.pop();

        assert(min_state->iteration_closed != m_iteration);
        assert(min_state->g != INFINITECOST);

        min_state->iteration_closed = m_iteration;
        min_state->eg = min_state->g;

        expand(min_state);

        ++elapsed_expansions;
    }

    return EXHAUSTED_OPEN_LIST;
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and INCONS list appropriately.
void ARAStar::expand(SearchState* s)
{
    std::vector<int> succs;
    std::vector<int> costs;
    m_space->GetSuccs(s->state_id, &succs, &costs);

    ROS_DEBUG_NAMED(SELOG, "  %zu successors", succs.size());

    for (size_t sidx = 0; sidx < succs.size(); ++sidx) {
        int succ_state_id = succs[sidx];
        int cost = costs[sidx];

        SearchState* succ_state = getSearchState(succ_state_id);
        reinitSearchState(succ_state);

        int new_cost = s->eg + cost;
        ROS_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        if (new_cost < succ_state->g) {
            succ_state->g = new_cost;
            succ_state->bp = s;
            if (succ_state->iteration_closed != m_iteration) {
                succ_state->f = computeKey(succ_state);
                if (m_open.contains(succ_state)) {
                    m_open.decrease(succ_state);
                } else {
                    m_open.push(succ_state);
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }
        }
    }
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void ARAStar::reorderOpen()
{
    for (auto it = m_open.begin(); it != m_open.end(); ++it) {
        (*it)->f = computeKey(*it);
    }
    m_open.make();
}

int ARAStar::computeKey(SearchState* s) const
{
    return s->g + (unsigned int)(m_curr_eps * s->h);
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
ARAStar::SearchState* ARAStar::getSearchState(int state_id)
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

// Create a new search state for a graph state.
ARAStar::SearchState* ARAStar::createState(int state_id)
{
    assert(state_id < m_graph_to_search_map.size());

    m_graph_to_search_map[state_id] = (int)m_states.size();

    SearchState* ss = new SearchState;
    ss->state_id = state_id;
    ss->call_number = 0;
    m_states.push_back(ss);

    return ss;
}

// Lazily (re)initialize a search state.
void ARAStar::reinitSearchState(SearchState* state)
{
    if (state->call_number != m_call_number) {
        ROS_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        state->h = m_heur->GetGoalHeuristic(state->state_id);
        state->f = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
        state->incons = false;
    }
}

// Extract the path from the start state up to a new state.
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
