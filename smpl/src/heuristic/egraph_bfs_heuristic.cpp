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

#include <smpl/heuristic/egraph_bfs_heuristic.h>

// system includes
#include <boost/regex.hpp>

namespace sbpl {
namespace motion {

EgraphBfsHeuristic::EgraphBfsHeuristic(
    const RobotPlanningSpacePtr& ps,
    const OccupancyGrid* grid)
:
    RobotHeuristic(ps, grid),
    m_pp(nullptr)
{
    m_pp = ps->getExtension<PointProjectionExtension>();
}

bool EgraphBfsHeuristic::loadExperienceGraph(const std::string& path)
{
    std::ifstream fin(path);
    if (!fin.is_open()) {
        return false;
    }

    std::string line;
    std::getline(fin, line);

    // The experience graphs file format is a csv format with the expected lines
    // 1. optionally, the first line specifies the order of the joint variables
    // listed on successive lines
    // 2. if the joint variable line is not there, the joint variables are
    // assumed to be in the order listed in the RobotModel
    // 3. each successive line consists of a sequence of doubles

    planningSpace()->robot()->getPlanningJoints();

    return true;
}

double EgraphBfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    return 0.0;
}

double EgraphBfsHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    return 0.0;
}

void EgraphBfsHeuristic::updateGoal(const GoalConstraint& goal)
{
}

int EgraphBfsHeuristic::GetGoalHeuristic(int state_id)
{
    return 0;
}

int EgraphBfsHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int EgraphBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

} // namespace motion
} // namespace sbpl
