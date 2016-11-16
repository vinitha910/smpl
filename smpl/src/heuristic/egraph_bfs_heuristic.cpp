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

// project includes
#include <smpl/csv_parser.h>

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
    m_num_cells_x = grid->numCellsX() + 2;
    m_num_cells_y = grid->numCellsY() + 2;
    m_num_cells_z = grid->numCellsZ() + 2;
    const int num_cells = m_num_cells_x * m_num_cells_y * m_num_cells_z;
    m_dist_grid.resize(num_cells);

    auto add_wall = [&](int x, int y, int z) {
        m_dist_grid[cellToIndex(x, y, z)] = std::numeric_limits<int>::max();
    };

    for (int y = 0; y < m_num_cells_y; ++y) {
        for (int z = 0; z < m_num_cells_z; ++z) {
            add_wall(0, y, z);
            add_wall(m_num_cells_x - 1, y, z);
        }
    }
    for (int x = 1; x < m_num_cells_x - 1; ++x) {
        for (int z = 0; z < m_num_cells_z; ++z) {
            add_wall(x, 0, z);
            add_wall(x, m_num_cells_y - 1, z);
        }
    }
    for (int x = 1; x < m_num_cells_x - 1; ++x) {
        for (int y = 1; y < m_num_cells_y - 1; ++y) {
            add_wall(x, y, 0);
            add_wall(x, y, m_num_cells_z - 1);
        }
    }
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

inline
int EgraphBfsHeuristic::cellToIndex(int x, int y, int z) const
{
    return x * m_num_cells_y * m_num_cells_z + y * m_num_cells_z + z;
}

} // namespace motion
} // namespace sbpl
