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

#include <ros/console.h>

#include "collision_operations.h"

namespace sbpl {
namespace collision {

static const char* CC_LOGGER = "collision";

bool CheckSphereCollision(
    const OccupancyGrid& grid,
    RobotCollisionState& state,
    double padding,
    const SphereIndex& sidx,
    double& dist)
{
    state.updateSphereState(sidx);
    const CollisionSphereState& ss = state.sphereState(sidx);

    int gx, gy, gz;
    grid.worldToGrid(ss.pos.x(), ss.pos.y(), ss.pos.z(), gx, gy, gz);

    // check bounds
    if (!grid.isInBounds(gx, gy, gz)) {
        const CollisionSphereModel* sm = ss.model;
        ROS_DEBUG_NAMED(CC_LOGGER, "Sphere '%s' with center at (%0.3f, %0.3f, %0.3f) (%d, %d, %d) is out of bounds.", sm->name.c_str(), ss.pos.x(), ss.pos.y(), ss.pos.z(), gx, gy, gz);
        dist = 0.0;
        return false;
    }

    // check for collision with world
    double obs_dist = grid.getDistance(gx, gy, gz);
    const double effective_radius =
            ss.model->radius + 0.5 * grid.getResolution() + padding;

    if (obs_dist <= effective_radius) {
        dist = obs_dist;
        return false;
    }

    dist = obs_dist;
    return true;
}

/// \brief Gather all sphere indices for a given group
///
/// The resulting sequence of sphere indices are already sorted by their
/// corresponding sphere priorities.
std::vector<SphereIndex> GatherSphereIndices(
    const RobotCollisionState& state,
    int gidx)
{
    std::vector<SphereIndex> sphere_indices;

    std::vector<int> ss_indices = state.groupSpheresStateIndices(gidx);
    for (int ssidx : ss_indices) {
        const CollisionSpheresState& spheres_state = state.spheresState(ssidx);
        std::vector<int> s_indices(spheres_state.spheres.size());
        int n = 0;
        std::generate(s_indices.begin(), s_indices.end(), [&]() { return n++; });
        for (int sidx : s_indices) {
            sphere_indices.emplace_back(ssidx, sidx);
        }
    }

    // sort sphere indices by priority
    std::sort(sphere_indices.begin(), sphere_indices.end(),
            [&](const SphereIndex& sidx1, const SphereIndex& sidx2)
            {
                const CollisionSphereState& ss1 = state.sphereState(sidx1);
                const CollisionSphereState& ss2 = state.sphereState(sidx2);
                const CollisionSphereModel* sph1 = ss1.model;
                const CollisionSphereModel* sph2 = ss2.model;
                return sph1->priority < sph2->priority;
            });

    return sphere_indices;
}

} // namespace collision
} // namespace sbpl
