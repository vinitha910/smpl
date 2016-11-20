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
    size_t num_cells_x = grid->numCellsX() + 2;
    size_t num_cells_y = grid->numCellsY() + 2;
    size_t num_cells_z = grid->numCellsZ() + 2;

    m_dist_grid.assign(num_cells_x, num_cells_y, num_cells_z, Cell(Unknown));

    auto add_wall = [&](int x, int y, int z) {
        m_dist_grid(x, y, z).dist = Wall;
    };

    for (int y = 0; y < num_cells_y; ++y) {
        for (int z = 0; z < num_cells_z; ++z) {
            add_wall(0, y, z);
            add_wall(num_cells_x - 1, y, z);
        }
    }
    for (int x = 1; x < num_cells_x - 1; ++x) {
        for (int z = 0; z < num_cells_z; ++z) {
            add_wall(x, 0, z);
            add_wall(x, num_cells_y - 1, z);
        }
    }
    for (int x = 1; x < num_cells_x - 1; ++x) {
        for (int y = 1; y < num_cells_y - 1; ++y) {
            add_wall(x, y, 0);
            add_wall(x, y, num_cells_z - 1);
        }
    }

    // TODO/NOTE: down-projection (through PointProjectionInterface) requires
    // state ids of states on experience graph...express this through
    // requirements on experience graph extension interface
    //
    // TODO: project experience graph into 3d space (projections of adjacent
    // nodes in the experience graph impose additional edges in 3d, cost equal
    // to the cheapest transition):
    //
    // (1) lookup transitions on-demand when a grid cell is expanded, loop
    // through all experience graph states and their neighbors (method used by
    // origin experience graph code)
    //
    // (2) embed an adjacency list in the dense grid structure as a
    // precomputation
    //
    // (3) maintain an external adjacency list mapping cells with projections
    // from experience graph states to adjacent cells
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
    Eigen::Vector3d p;
    m_pp->projectToPoint(state_id, p);

    int gx, gy, gz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), gx, gy, gz);

    if (!grid()->isInBounds(gx, gy, gz)) {
        return Infinity;
    }

    if (m_dist_grid(gx, gy, gz).dist == Wall) {
        return Infinity;
    }

    while (m_dist_grid(gx, gy, gz).dist == Unknown && !m_open.empty()) {
        // TODO: dijkstra expansions within
    }

    if (m_dist_grid(gx, gy, gz).dist > Infinity) {
        return Infinity;
    }
    return m_dist_grid(gx, gy, gz).dist;
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
