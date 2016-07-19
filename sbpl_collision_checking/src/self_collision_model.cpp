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

namespace sbpl {
namespace collision {

class SelfCollisionModelImpl
{
public:

    SelfCollisionModelImpl(
        OccupancyGrid* grid,
        RobotCollisionState* state);

    ~SelfCollisionModelImpl();

    bool checkCollision();

private:

    OccupancyGrid* m_grid;
    RobotCollisionState* m_state;
};

SelfCollisionModelImpl::SelfCollisionModelImpl(
    OccupancyGrid* grid,
    RobotCollisionState* state)
:
    m_grid(grid),
    m_state(state)
{
}

bool SelfCollisionModelImpl::checkCollision()
{
    bool self_collision = false;
    // check self collisions
    for (size_t sidx1 = 0; sidx1 < m_sphere_indices.size(); ++sidx1) {
        const CollisionSphereState& ss1 = m_state.sphereState(sidx1);
        const CollisionSphereModel& smodel1 = *ss1.model;

        for (size_t sidx2 = 0; sidx2 < m_sphere_indices.size(); ++sidx2) {
            const CollisionSphereState& ss2 = m_state.sphereState(sidx2);
            const CollisionSphereModel& smodel2 = *ss2.model;

            Eigen::Vector3d dx = ss2.pos - ss1.pos;
            const double radius_combined = smodel1.radius + smodel2.radius;
            if (dx.squaredNorm() < radius_combined * radius_combined) {
                collision_detection::AllowedCollision::Type type;
                if (!m_acm.getEntry(smodel1.name, smodel2.name, type)) {
                    ROS_ERROR_NAMED(CC_LOGGER, "An allowed collisions entry wasn't found for a collision sphere");
                }
                if (type == collision_detection::AllowedCollision::NEVER) {
                    if (visualize) {
                        self_collision = true;
                        Sphere s1, s2;
                        s1.center = ss1.pos;
                        s1.radius = smodel1.radius;
                        s2.center = ss2.pos;
                        s2.radius = smodel2.radius;
                        m_collision_spheres.push_back(s1);
                        m_collision_spheres.push_back(s2);
                    }
                    else {
                        return false;
                    }
                }
            }
        }
    }

    return !self_collision;
}

///////////////////////////////////////
// SelfCollisionModel Implementation //
///////////////////////////////////////

SelfCollisionModel::SelfCollisionModel(
    OccupancyGrid* grid,
    RobotCollisionState* collision_state)
:
    m_impl(new SelfCollisionModelImpl(grid, collision_state))
{
}

SelfCollisionModelImpl::~SelfCollisionModel()
{
}

bool SelfCollisionModel::checkCollision()
{
    return m_impl->checkCollision();
}

} // namespace collision
} // namespace sbpl
