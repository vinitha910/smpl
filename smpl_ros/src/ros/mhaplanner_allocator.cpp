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

#include <smpl/ros/mhaplanner_allocator.h>

// system includes
#include <sbpl/planners/mhaplanner.h>

namespace sbpl {
namespace motion {

SBPLPlannerPtr MHAPlannerAllocator::allocate(
    const RobotPlanningSpacePtr& pspace,
    const RobotHeuristicPtr& heuristic)
{
    m_heur_vec.clear();
    m_heur_vec.push_back(heuristic.get());

    const bool forward_search = true;
    auto search = std::make_shared<MHAPlanner>(
            pspace.get(), m_heur_vec[0], &m_heur_vec[0], m_heur_vec.size());

    double mha_eps;
    pspace->params()->param("epsilon_mha", mha_eps, 1.0);
    search->set_initial_mha_eps(mha_eps);

    double epsilon;
    pspace->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    pspace->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);
    return search;
}

} // namespace motion
} // namespace sbpl
