#ifndef SMPL_EGRAPH_PLANNER_H
#define SMPL_EGRAPH_PLANNER_H

// system includes
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/intrusive_heap.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace sbpl {
namespace motion {

class ExperienceGraphPlanner : public SBPLPlanner
{
public:

    ExperienceGraphPlanner(
        const RobotPlanningSpacePtr& pspace,
        const RobotHeuristicPtr& heur);

    ~ExperienceGraphPlanner();

    /// \name Reimplemented Public Functions from SBPLPlanner
    ///@{
    int replan(std::vector<int>* solution, ReplanParams params) override;
    int replan(std::vector<int>* solution, ReplanParams params, int* cost) override;

    int     force_planning_from_scratch_and_free_memory() override;
    double  get_solution_eps() const override;
    int     get_n_expands() const override;
    double  get_initial_eps() override;
    double  get_initial_eps_planning_time() override;
    double  get_final_eps_planning_time() override;
    int     get_n_expands_init_solution() override;
    double  get_final_epsilon() override;
    void    get_search_stats(std::vector<PlannerStats>* s) override;
    void    set_initialsolution_eps(double initialsolution_eps) override;
    ///@}

    /// \name Required Public Functions from SBPLPlanner
    ///@{
    int replan(double allowed_time, std::vector<int>* solution) override;
    int replan(double allowed_time, std::vector<int>* solution, int* cost) override;
    int set_goal(int goal_state_id) override;
    int set_start(int start_state_id) override;
    int force_planning_from_scratch() override;
    int set_search_mode(bool first_solution_unbounded);
    void costs_changed(const StateChangeQuery& state_change);
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

    RobotPlanningSpacePtr m_pspace;
    RobotHeuristicPtr m_heur;

    std::vector<SearchState*> m_states;
    SearchState* m_start_state;
    SearchState* m_goal_state;

    std::vector<int> m_graph_to_search_map;

    intrusive_heap<SearchState, SearchStateCompare> m_open;

    int m_call_number;
    double m_eps;

    int m_expand_count;

    SearchState* getSearchState(int state_id);
    SearchState* createState(int state_id);
    void reinitSearchState(SearchState* state);
};

} // namespace motion
} // namespace sbpl

#endif
