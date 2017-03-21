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

#include <smpl/ros/adaptive_planner_allocator.h>

// project includes
#include <smpl/search/adaptive_planner.h>

namespace sbpl {
namespace motion {

static const char *PI_LOGGER = "simple";

SBPLPlannerPtr AdaptivePlannerAllocator::allocate(
    const RobotPlanningSpacePtr& pspace,
    const RobotHeuristicPtr& heuristic)
{
    auto search = std::make_shared<AdaptivePlanner>(pspace, heuristic);

    double epsilon_plan;
    pspace->params()->param("epsilon_plan", epsilon_plan, 1.0);
    search->set_plan_eps(epsilon_plan);

    double epsilon_track;
    pspace->params()->param("epsilon_track", epsilon_track, 1.0);
    search->set_track_eps(epsilon_track);

    AdaptivePlanner::TimeParameters tparams;
    tparams.planning.bounded = true;
    tparams.planning.improve = false;
    tparams.planning.type = ARAStar::TimeParameters::TIME;
    tparams.planning.max_allowed_time_init = clock::duration::zero();
    tparams.planning.max_allowed_time = clock::duration::zero();

    tparams.tracking.bounded = true;
    tparams.tracking.improve = false;
    tparams.tracking.type = ARAStar::TimeParameters::TIME;
    tparams.tracking.max_allowed_time_init = std::chrono::seconds(5);
    tparams.tracking.max_allowed_time = clock::duration::zero();

    search->set_time_parameters(tparams);

    return search;
}

} // namespace motion
} // namespace sbpl
