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

#ifndef SMPL_ARASTAR_H
#define SMPL_ARASTAR_H

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/intrusive_heap.h>
#include <smpl/time.h>

namespace sbpl {

class ARAStar : public SBPLPlanner
{
public:

    ARAStar(DiscreteSpaceInformation* space, Heuristic* heuristic);
    ~ARAStar();

    /// \name Required Functions from SBPLPlanner
    ///@{
    int replan(double allowed_time_secs, std::vector<int>* solution) override;
    int replan(double allowed_time_secs, std::vector<int>* solution, int* solcost) override;
    int set_goal(int state_id) override;
    int set_start(int state_id) override;
    int force_planning_from_scratch() override;
    int set_search_mode(bool bSearchUntilFirstSolution) override;
    void costs_changed(const StateChangeQuery& stateChange) override;
    ///@}

    /// \name Reimplemented Functions from SBPLPlanner
    ///@{
    int replan(std::vector<int>* solution, ReplanParams params) override;
    int replan(std::vector<int>* solution, ReplanParams params, int* solcost) override;
    int force_planning_from_scratch_and_free_memory() override;
    double get_solution_eps() const override;
    int get_n_expands() const override;
    double get_initial_eps() override;
    double get_initial_eps_planning_time() override;
    double get_final_eps_planning_time() override;
    int get_n_expands_init_solution() override;
    double get_final_epsilon() override;
    void get_search_stats(std::vector<PlannerStats>* s) override;
    void set_initialsolution_eps(double eps) override;
    ///@}

private:

    struct SearchState : public heap_element
    {
        int state_id;
        unsigned int g;
        unsigned int h;
        unsigned int f;
        unsigned short iteration_closed;
        unsigned short call_number;
        SearchState* bp;
    };

    struct SearchStateCompare
    {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            return s1.f < s2.f;
        }
    };

    DiscreteSpaceInformation* m_space;
    Heuristic* m_heur;

    // parameters for controlling how long the search runs
    struct TimingParameters
    {
        bool bounded;
        bool improve;
        enum TimingType { EXPANSIONS, TIME } type;
        int min_expansions;
        int max_expansions;
        clock::duration min_allowed_time;
        clock::duration max_allowed_time;
    } m_time_params;

    std::vector<SearchState*> m_states;

    int m_start_state_id;   // graph state id for the start state
    int m_goal_state_id;    // graph state id for the goal state

    // map from graph state id to search state id, incrementally expanded
    // as states are encountered during the search
    std::vector<int> m_graph_to_search_map;

    // search state (not including the values of g, f, back pointers, and
    // closed list from m_stats)
    intrusive_heap<SearchState, SearchStateCompare> m_open;
    double m_eps;
    int m_iteration;

    int m_call_number;          // for lazy reinitialization of search states
    int m_last_start_state_id;  // for lazy reinitialization of the search tree
    int m_last_goal_state_id;   // for updating the search tree when the goal changes
    double m_last_eps;          // for updating the search tree when heuristics change

    int m_expand_count;
    clock::duration m_search_time;

    bool timedOut(
        int elapsed_expansions,
        const clock::duration& elapsed_time) const;

    void recomputeHeuristics();
    void recomputeKeys();
    void reorderOpenList();
    int computeKey(SearchState* s) const;

    SearchState* getSearchState(int state_id);
    SearchState* createState(int state_id);
    void reinitSearchState(SearchState* state);

    void extractPath(
        SearchState* to_state,
        std::vector<int>& solution,
        int& cost) const;
};

} // namespace sbpl

#endif
