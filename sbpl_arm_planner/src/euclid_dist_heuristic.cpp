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

#include <sbpl_arm_planner/euclid_dist_heuristic.h>

// standard includes
#include <math.h>

namespace sbpl {
namespace manip {

static inline
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

EuclidDistHeuristic::EuclidDistHeuristic(
    ManipLattice* pspace,
    const OccupancyGrid* grid)
:
    RobotHeuristic(pspace, grid)
{
}

double EuclidDistHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    const std::vector<double>& goal_pose = planningSpace()->goal().pose;
    return EuclideanDistance(x, y, z, goal_pose[0], goal_pose[1], goal_pose[2]);
}

double EuclidDistHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: implement
    return 0.0;
}

int EuclidDistHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    ManipLattice* manip_lattice = (ManipLattice*)planningSpace();

    const std::vector<double>& gp = planningSpace()->goal().pose;
    const ManipLatticeState* state = manip_lattice->getHashEntry(state_id);
    double x, y, z;
    grid()->gridToWorld(state->xyz[0], state->xyz[1], state->xyz[2], x, y, z);

    int h;
    h = EuclideanDistance(x, y, z, gp[0], gp[1], gp[2]);
    h *= params()->cost_per_meter_;
    h *= 500;
    return h;
}

int EuclidDistHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int EuclidDistHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    ManipLattice* manip_lattice = (ManipLattice*)planningSpace();
    const ManipLatticeState* from_entry = manip_lattice->getHashEntry(from_id);
    const ManipLatticeState* to_entry = manip_lattice->getHashEntry(to_id);

    double fx, fy, fz, tx, ty, tz;
    grid()->gridToWorld(
            from_entry->xyz[0], from_entry->xyz[1], from_entry->xyz[2],
            fx, fy, fz);
    grid()->gridToWorld(
            to_entry->xyz[0], to_entry->xyz[1], to_entry->xyz[2],
            tx, ty, tz);

    int h;
    h = EuclideanDistance(fx, fy, fz, tx, ty, tz);
    h *= params()->cost_per_meter_;
    h *= 500;
    h /= grid()->getResolution();
    return h;
}

} // namespace manip
} // namespace sbpl
