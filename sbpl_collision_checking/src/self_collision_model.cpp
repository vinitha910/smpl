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

#include <sbpl_collision_checking/self_collision_model.h>

#include "collision_operations.h"

namespace sbpl {
namespace collision {

static const char* SCM_LOGGER = "self";

class SelfCollisionModelImpl
{
public:

    SelfCollisionModelImpl(
        OccupancyGrid* grid,
        const RobotCollisionModel* model,
        const AttachedBodiesCollisionModel* ab_model);

    ~SelfCollisionModelImpl();

    void setAllowedCollisionMatrix(
        const AllowedCollisionMatrix& acm);
    void setPadding(double padding);

    bool checkCollision(
        RobotCollisionState& state,
        const std::string& group_name,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        const int gidx,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const std::string& group_name,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const int gidx,
        double& dist);

private:

    OccupancyGrid*                          m_grid;
    const RobotCollisionModel*              m_model;
    const AttachedBodiesCollisionModel*     m_ab_model;

    RobotCollisionState                     m_state;
    int                                     m_gidx;
    std::vector<int>                        m_voxels_indices;
    std::vector<SphereIndex>                m_sphere_indices;

    AllowedCollisionMatrix m_acm;
    double m_padding;

    // switch to checking for a new collision group; removes voxels from groups
    // that are inside the new collision group and add voxels that are outside
    // the new collision group
    void switchGroup(int gidx);

    void copyState(const RobotCollisionState& state);

    void updateVoxelsStates();

    // update the state of outside-group voxels and check for collisions between
    // spheres and occupied voxels
    bool checkVoxelsStateCollisions(double& dist);

    // check for collisions between inside-group spheres
    bool checkSpheresStateCollisions(double& dist);
};

SelfCollisionModelImpl::SelfCollisionModelImpl(
    OccupancyGrid* grid,
    const RobotCollisionModel* model,
    const AttachedBodiesCollisionModel* ab_model)
:
    m_grid(grid),
    m_model(model),
    m_ab_model(ab_model),
    m_state(m_model),
    m_gidx(-1),
    m_voxels_indices(),
    m_acm(),
    m_padding(0.0)
{
}

SelfCollisionModelImpl::~SelfCollisionModelImpl()
{
}

void SelfCollisionModelImpl::setAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    m_acm = acm;
}

void SelfCollisionModelImpl::setPadding(double padding)
{
    m_padding = padding;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    const std::string& group_name,
    double& dist)
{
    if (state.model() != m_model) {
        return false;
    }

    if (!m_model->hasGroup(group_name)) {
        return false;
    }

    int gidx = m_model->groupIndex(group_name);
    if (gidx != m_gidx) {
        switchGroup(gidx);
    }

    copyState(state);

    if (!checkVoxelsStateCollisions(dist)) {
        return false;
    }

    if (!checkSpheresStateCollisions(dist)) {
        return false;
    }

    return true;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    const int gidx,
    double& dist)
{
    if (state.model() != m_model) {
        return false;
    }

    return true;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const std::string& group_name,
    double& dist)
{
    if (state.model() != m_model) {
        return false;
    }

    if (!m_model->hasGroup(group_name)) {
        return false;
    }

    int gidx = m_model->groupIndex(group_name);
    if (gidx != m_gidx) {
        switchGroup(gidx);
    }

    return true;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx,
    double& dist)
{
    if (state.model() != m_model) {
        return false;
    }

    return true;
}

void SelfCollisionModelImpl::switchGroup(int gidx)
{
    std::vector<int> old_outside_indices_sorted = m_voxels_indices;

    std::sort(
            old_outside_indices_sorted.begin(),
            old_outside_indices_sorted.end());

    std::vector<int> new_outside_indices_sorted =
            m_state.groupSpheresStateIndices(gidx);
    std::sort(
            new_outside_indices_sorted.begin(),
            new_outside_indices_sorted.end());

    // get the indices of the voxels states that were outside the group but are
    // now inside and must be removed
    std::vector<int> outside_index_removals;
    std::set_difference(
            old_outside_indices_sorted.begin(),
            old_outside_indices_sorted.end(),
            new_outside_indices_sorted.begin(),
            new_outside_indices_sorted.end(),
            std::back_inserter(outside_index_removals));

    // get the indices of the voxels states that were inside the group but are
    // now outside and must be inserted
    std::vector<int> outside_index_insertions;
    std::set_difference(
            new_outside_indices_sorted.begin(),
            new_outside_indices_sorted.end(),
            old_outside_indices_sorted.begin(),
            old_outside_indices_sorted.end(),
            std::back_inserter(outside_index_removals));

    // gather the voxels to be removed
    std::vector<Eigen::Vector3d> voxel_removals;
    for (int vsidx : outside_index_removals) {
        const CollisionVoxelsState& vs = m_state.voxelsState(vsidx);
        voxel_removals.insert(
            voxel_removals.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // gather the voxels to be inserted
    std::vector<Eigen::Vector3d> voxel_insertions;
    for (int vsidx : outside_index_insertions) {
        const CollisionVoxelsState& vs = m_state.voxelsState(vsidx);
        voxel_insertions.insert(
                voxel_insertions.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // insert/remove the voxels
    if (!voxel_removals.empty()) {
        m_grid->removePointsFromField(voxel_removals);
    }
    if (!voxel_insertions.empty()) {
        m_grid->addPointsToField(voxel_insertions);
    }

    m_sphere_indices.clear();
    std::vector<int> ss_indices = m_state.groupSpheresStateIndices(gidx);
    for (int ssidx : ss_indices) {
        const CollisionSpheresState& spheres_state =
                m_state.spheresState(ssidx);
        std::vector<int> s_indices(spheres_state.spheres.size());
        int n = 0;
        std::generate(s_indices.begin(), s_indices.end(), [&]() { return n++; });
        for (int sidx : s_indices) {
            m_sphere_indices.emplace_back(ssidx, sidx);
        }
    }

    // sort sphere m_state indices by priority
    std::sort(m_sphere_indices.begin(), m_sphere_indices.end(),
            [&](const SphereIndex& sidx1, const SphereIndex& sidx2)
            {
                const CollisionSphereState& ss1 = m_state.sphereState(sidx1);
                const CollisionSphereState& ss2 = m_state.sphereState(sidx2);
                const CollisionSphereModel* sph1 = ss1.model;
                const CollisionSphereModel* sph2 = ss2.model;
                return sph1->priority < sph2->priority;
            });

    m_gidx = gidx;
}

void SelfCollisionModelImpl::copyState(const RobotCollisionState& state)
{
    for (size_t vidx = 0; vidx < m_state.model()->jointVarCount(); ++vidx) {
        const double p = state.jointVarPosition(vidx);
        m_state.setJointVarPosition(vidx, p);
    }
}

void SelfCollisionModelImpl::updateVoxelsStates()
{
    // update voxel groups; gather voxels before updating so as to impose only
    // a single distance field update (TODO: does the distance field recompute
    // with every call to insert/remove/update points?)
    std::vector<Eigen::Vector3d> voxel_removals;
    std::vector<Eigen::Vector3d> voxel_insertions;
    for (int vsidx : m_voxels_indices) {
        if (m_state.voxelsStateDirty(vsidx)) {
            const CollisionVoxelsState& voxels_state = m_state.voxelsState(vsidx);

            // copy over voxels to be removed before updating
            voxel_removals.insert(
                    voxel_removals.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            m_state.updateVoxelsState(vsidx);

            // copy over voxels to be inserted
            voxel_insertions.insert(
                    voxel_insertions.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            ROS_DEBUG_NAMED(SCM_LOGGER, "Updating Occupancy Grid with change to Collision Voxels State (%zu displaced)", voxel_removals.size());
        }
    }

    // TODO: check for voxel state updates from attached objects

    // update occupancy grid with new voxel data
    if (!voxel_removals.empty()) {
        m_grid->removePointsFromField(voxel_removals);
    }
    if (!voxel_insertions.empty()) {
        m_grid->addPointsToField(voxel_insertions);
    }
}

bool SelfCollisionModelImpl::checkVoxelsStateCollisions(double& dist)
{
    updateVoxelsStates();

    for (const auto& sidx : m_sphere_indices) {
        double obs_dist;
        if (!CheckSphereCollision(*m_grid, m_state, m_padding, sidx, obs_dist))
        {
            const CollisionSphereModel* sm = m_state.sphereState(sidx).model;
            ROS_DEBUG_NAMED(SCM_LOGGER, "    *collision* idx: %s, name: %s, radius: %0.3fm, dist: %0.3fm", to_string(sidx).c_str(), sm->name.c_str(), sm->radius, obs_dist);
            dist = obs_dist;
            return false;
        }

        if (obs_dist < dist) {
            dist = obs_dist;
        }
    }

    return true;
}

bool SelfCollisionModelImpl::checkSpheresStateCollisions(double& dist)
{
    // check self collisions
    for (const SphereIndex& sidx1 : m_sphere_indices) {
        const CollisionSphereState& ss1 = m_state.sphereState(sidx1);
        const CollisionSphereModel& smodel1 = *ss1.model;

        for (const SphereIndex& sidx2 : m_sphere_indices) {
            const CollisionSphereState& ss2 = m_state.sphereState(sidx2);
            const CollisionSphereModel& smodel2 = *ss2.model;

            Eigen::Vector3d dx = ss2.pos - ss1.pos;
            const double radius_combined = smodel1.radius + smodel2.radius;
            if (dx.squaredNorm() < radius_combined * radius_combined) {
                collision_detection::AllowedCollision::Type type;
                if (!m_acm.getEntry(smodel1.name, smodel2.name, type)) {
                    ROS_ERROR_NAMED(SCM_LOGGER, "An allowed collisions entry wasn't found for a collision sphere");
                }
                if (type == collision_detection::AllowedCollision::NEVER) {
                    return false;
                }
            }
        }
    }

    return true;
}

///////////////////////////////////////
// SelfCollisionModel Implementation //
///////////////////////////////////////

SelfCollisionModel::SelfCollisionModel(
    OccupancyGrid* grid,
    const RobotCollisionModel* model,
    const AttachedBodiesCollisionModel* ab_model)
:
    m_impl(new SelfCollisionModelImpl(grid, model, ab_model))
{
}

SelfCollisionModel::~SelfCollisionModel()
{
}

void SelfCollisionModel::setAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    return m_impl->setAllowedCollisionMatrix(acm);
}

void SelfCollisionModel::setPadding(double padding)
{
    return m_impl->setPadding(padding);
}

bool SelfCollisionModel::checkCollision(
    RobotCollisionState& state,
    const std::string& group_name,
    double& dist)
{
    return m_impl->checkCollision(state, group_name, dist);
}

bool SelfCollisionModel::checkCollision(
    RobotCollisionState& state,
    const int gidx,
    double& dist)
{
    return m_impl->checkCollision(state, gidx, dist);
}

bool SelfCollisionModel::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const std::string& group_name,
    double& dist)
{
    return m_impl->checkCollision(state, ab_state, group_name, dist);
}

bool SelfCollisionModel::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx,
    double& dist)
{
    return m_impl->checkCollision(state, ab_state, gidx, dist);
}

} // namespace collision
} // namespace sbpl
