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

#include <smpl/search/adaptive_planner.h>

// standard includes
#include <chrono>

// project includes
#include <smpl/time.h>

namespace sbpl {
namespace motion {

AdaptivePlanner::AdaptivePlanner(
    const RobotPlanningSpacePtr& pspace,
    const RobotHeuristicPtr& heur)
:

    m_planner(pspace.get(), true),
    m_tracker(pspace.get(), true),
    m_adaptive_graph(nullptr),
    m_start_id(-1),
    m_goal_id(-1),
    m_eps_track(1.0)
{
    m_adaptive_graph = pspace->getExtension<AdaptiveGraphExtension>();
    if (!m_adaptive_graph) {
        ROS_WARN("Adaptive Planner recommends Adaptive Graph Extension");
    }
}

AdaptivePlanner::~AdaptivePlanner()
{

}

int AdaptivePlanner::replan(std::vector<int>* solution, ReplanParams params)
{
    return 0;
}

int AdaptivePlanner::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* cost)
{
    return 0;
}

int AdaptivePlanner::force_planning_from_scratch_and_free_memory()
{
    return 0;
}

double AdaptivePlanner::get_solution_eps() const
{
    return 0.0;
}

int AdaptivePlanner::get_n_expands() const
{
    return 0;
}

double AdaptivePlanner::get_initial_eps()
{
    return 0.0;
}

double AdaptivePlanner::get_initial_eps_planning_time()
{
    return 0.0;
}

double AdaptivePlanner::get_final_eps_planning_time()
{
    return 0.0;
}

int AdaptivePlanner::get_n_expands_init_solution()
{
    return 0;
}

double AdaptivePlanner::get_final_epsilon()
{
    return 0.0;
}

void AdaptivePlanner::get_search_stats(std::vector<PlannerStats>* s)
{

}

void AdaptivePlanner::set_initialsolution_eps(double initialsolution_eps)
{

}

int AdaptivePlanner::replan(double allowed_time, std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

int AdaptivePlanner::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    if (m_start_id == -1 || m_goal_id == -1) {
        ROS_ERROR("Start or goal state is not set");
        return 1;
    }

    m_adaptive_graph->addHighDimRegion(m_start_id);
    m_adaptive_graph->addHighDimRegion(m_goal_id);

    double time_remaining = allowed_time;

    std::vector<int> path;
    bool done =  false;
    int iter_count = 0;
    int res;
    while (!done) {
        ++iter_count;

        // search G^ad for a least-cost path
        if (!m_planner.set_start(m_start_id) ||
            !m_planner.set_goal(m_goal_id))
        {
            ROS_ERROR("Failed to set start or goal states for plan phase");
            return 0;
        }

        path.clear();
        int plan_cost = 0;
        auto plan_start = clock::now();
        res = m_planner.replan(allowed_time, &path, &plan_cost);
        auto plan_finish = clock::now();

        time_remaining = std::chrono::duration<double>(plan_finish - plan_start).count();
        time_remaining = std::min(0.0, time_remaining);

        if (time_remaining == 0.0) {
            ROS_WARN("No time to track!");
            return 0;
        }

        if (!res) {
            ROS_WARN("Failed to find least-cost path in G^ad");
            return 0;
        }

        if (m_adaptive_graph->isExecutable(path)) {
            ROS_INFO("Found path during planning on iteration %d", iter_count);
            return 1;
        }

        m_adaptive_graph->setTunnel(path);
        path.clear();
        int track_cost = 0;
        auto track_start = clock::now();
        res = m_tracker.replan(allowed_time, &path, &track_cost);
        auto track_finish = clock::now();

        time_remaining = std::chrono::duration<double>(plan_finish - plan_start).count();
        time_remaining = std::min(0.0, time_remaining);

        if (!res) {
            if (time_remaining == 0.0) {
                ROS_WARN("Ran out of time after tracking phase");
                return 0;
            }
//            m_tracker.getBestState();
        } else if (track_cost > m_eps_track * plan_cost) {
            if (time_remaining == 0.0) {
                ROS_WARN("Ran out of time after tracking phase");
                return 0;
            }
        } else {

        }

    }

    ROS_INFO("Failed to find solution");
    return 0;
}

int AdaptivePlanner::set_goal(int goal_state_id)
{
    return 0;
}

int AdaptivePlanner::set_start(int start_state_id)
{
    return 0;
}

int AdaptivePlanner::force_planning_from_scratch()
{
    return 0;
}

int AdaptivePlanner::set_search_mode(bool first_solution_unbounded)
{
    return 0;
}

void AdaptivePlanner::costs_changed(const StateChangeQuery& state_change)
{

}

} // namespace motion
} // namespace sbpl
