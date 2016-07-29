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
        const RobotCollisionModel* rcm,
        const AttachedBodiesCollisionModel* ab_model);

    ~SelfCollisionModelImpl();

    const AllowedCollisionMatrix& allowedCollisionMatrix() const;
    void updateAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
    void setAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);

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

    double collisionDistance(
        RobotCollisionState& state,
        const std::string& group_name);

    double collisionDistance(
        RobotCollisionState& state,
        const int gidx);

    double collisionDistance(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const std::string& group_name);

    double collisionDistance(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const int gidx);

private:

    OccupancyGrid*                          m_grid;
    const RobotCollisionModel*              m_rcm;
    const AttachedBodiesCollisionModel*     m_abcm;

    RobotCollisionState                     m_rcs;
    AttachedBodiesCollisionState            m_abcs;

    // cached group information updated when a collision check for a different
    // group is made
    int                                     m_gidx;
    std::vector<int>                        m_voxels_indices;
    std::vector<SphereIndex>                m_sphere_indices;

    int                                     m_ab_gidx;
    std::vector<int>                        m_ab_voxel_indices;
    std::vector<int>                        m_ab_sphere_indices;

    AllowedCollisionMatrix                  m_acm;
    double                                  m_padding;

    void initAllowedCollisionMatrix();

    // switch to checking for a new collision group; removes voxels from groups
    // that are inside the new collision group and add voxels that are outside
    // the new collision group
    void updateGroup(int gidx);
    void switchAttachedBodyGroup(int ab_gidx);

    void copyState(const RobotCollisionState& state);

    void updateVoxelsStates();

    // update the state of outside-group voxels and check for collisions between
    // spheres and occupied voxels
    bool checkVoxelsStateCollisions(double& dist);
    bool checkAttachedBodyVoxelsStateCollisions(double& dist);

    // check for collisions between inside-group spheres
    bool checkSpheresStateCollisions(double& dist);
    bool checkAttachedBodySpheresStateCollisions(double& dist);
};

SelfCollisionModelImpl::SelfCollisionModelImpl(
    OccupancyGrid* grid,
    const RobotCollisionModel* rcm,
    const AttachedBodiesCollisionModel* ab_model)
:
    m_grid(grid),
    m_rcm(rcm),
    m_abcm(ab_model),
    m_rcs(rcm),
    m_abcs(ab_model, &m_rcs),
    m_gidx(-1),
    m_voxels_indices(),
    m_sphere_indices(),
    m_acm(),
    m_padding(0.0)
{
    initAllowedCollisionMatrix();
    m_acm.print(std::cout);
}

void SelfCollisionModelImpl::initAllowedCollisionMatrix()
{
//    // add allowed collisions between spheres on the same link
//    for (const auto& spheres_config : config.spheres_models) {
//        for (size_t i = 0; i < spheres_config.spheres.size(); ++i) {
//            const std::string& s1_name = spheres_config.spheres[i].name;
//            if (!m_acm.hasEntry(s1_name)) {
//                ROS_INFO_NAMED(CC_LOGGER, "Adding entry '%s' to the ACM", s1_name.c_str());
//                m_acm.setEntry(s1_name, false);
//            }
//            for (size_t j = i + 1; j < spheres_config.spheres.size(); ++j) {
//                const std::string& s2_name = spheres_config.spheres[j].name;
//                if (!m_acm.hasEntry(s2_name)) {
//                    ROS_INFO_NAMED(CC_LOGGER, "Adding entry '%s' to the ACM", s2_name.c_str());
//                    m_acm.setEntry(s2_name, false);
//                }
//
//                ROS_INFO_NAMED(CC_LOGGER, "Spheres '%s' and '%s' attached to the same link...allowing collision", s1_name.c_str(), s2_name.c_str());
//                m_acm.setEntry(s1_name, s2_name, true);
//            }
//        }
//    }
//
//    // add in additional allowed collisions from config
//    std::vector<std::string> config_entries;
//    config.acm.getAllEntryNames(config_entries);
//    for (size_t i = 0; i < config_entries.size(); ++i) {
//        const std::string& entry1 = config_entries[i];
//        if (!m_acm.hasEntry(entry1)) {
//            ROS_WARN_NAMED(CC_LOGGER, "Configured allowed collision entry '%s' was not found in the collision model", entry1.c_str());
//            continue;
//        }
//        for (size_t j = i; j < config_entries.size(); ++j) {
//            const std::string& entry2 = config_entries[j];
//            if (!m_acm.hasEntry(entry2)) {
//                ROS_WARN_NAMED(CC_LOGGER, "Configured allowed collision entry '%s' was not found in the collision model", entry2.c_str());
//                continue;
//            }
//
//            if (!config.acm.hasEntry(entry1, entry2)) {
//                continue;
//            }
//
//            collision_detection::AllowedCollision::Type type;
//            config.acm.getEntry(entry1, entry2, type);
//            switch (type) {
//            case collision_detection::AllowedCollision::NEVER:
//                // NOTE: not that it matters, but this disallows config freeing
//                // collisions
//                break;
//            case collision_detection::AllowedCollision::ALWAYS:
//                ROS_INFO_NAMED(CC_LOGGER, "Configuration allows spheres '%s' and '%s' to be in collision", entry1.c_str(), entry2.c_str());
//                m_acm.setEntry(entry1, entry2, true);
//                break;
//            case collision_detection::AllowedCollision::CONDITIONAL:
//                ROS_WARN_NAMED(CC_LOGGER, "Conditional collisions not supported in SBPL Collision Detection");
//                break;
//            }
//        }
//    }
}

SelfCollisionModelImpl::~SelfCollisionModelImpl()
{
}

const AllowedCollisionMatrix&
SelfCollisionModelImpl::allowedCollisionMatrix() const
{
    return m_acm;
}

void SelfCollisionModelImpl::updateAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
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
    if (state.model() != m_rcm) {
        return false;
    }

    if (!m_rcm->hasGroup(group_name)) {
        return false;
    }

    int gidx = m_rcm->groupIndex(group_name);
    updateGroup(gidx);

    copyState(state);

    if (!checkVoxelsStateCollisions(dist) ||
        !checkSpheresStateCollisions(dist))
    {
        return false;
    }

    return true;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    const int gidx,
    double& dist)
{
    if (state.model() != m_rcm) {
        return false;
    }

    if (gidx < 0 || gidx >= m_rcm->groupCount()) {
        return false;
    }

    updateGroup(gidx);
    copyState(state);

    if (!checkVoxelsStateCollisions(dist) ||
        !checkSpheresStateCollisions(dist))
    {
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
    if (state.model() != m_rcm || ab_state.model() != m_abcm) {
        return false;
    }

    if (!m_rcm->hasGroup(group_name) || !m_abcm->hasGroup(group_name)) {
        return false;
    }

    const int gidx = m_rcm->groupIndex(group_name);
    updateGroup(gidx);
    const int ab_gidx = m_abcm->groupIndex(group_name);
    if (ab_gidx != m_gidx) {
        switchAttachedBodyGroup(ab_gidx);
    }

    copyState(state);

    if (!checkVoxelsStateCollisions(dist) ||
        !checkAttachedBodyVoxelsStateCollisions(dist) ||
        !checkSpheresStateCollisions(dist) ||
        !checkAttachedBodySpheresStateCollisions(dist))
    {
        return false;
    }

    return true;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx,
    double& dist)
{
    if (state.model() != m_rcm || ab_state.model() != m_abcm) {
        return false;
    }

    if (gidx < 0 || gidx >= m_rcm->groupCount() || gidx >= m_abcm->groupCount())
    {
        return false;
    }

    updateGroup(gidx);
    if (gidx != m_ab_gidx) {
        switchAttachedBodyGroup(gidx);
    }

    copyState(state);

    if (!checkVoxelsStateCollisions(dist) ||
        !checkAttachedBodyVoxelsStateCollisions(dist) ||
        !checkSpheresStateCollisions(dist) ||
        !checkAttachedBodySpheresStateCollisions(dist))
    {
        return false;
    }

    return true;
}

double SelfCollisionModelImpl::collisionDistance(
    RobotCollisionState& state,
    const std::string& group_name)
{
    return 0.0;
}

double SelfCollisionModelImpl::collisionDistance(
    RobotCollisionState& state,
    const int gidx)
{
    return 0.0;
}

double SelfCollisionModelImpl::collisionDistance(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const std::string& group_name)
{
    return 0.0;
}

double SelfCollisionModelImpl::collisionDistance(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx)
{
    return 0.0;
}

void SelfCollisionModelImpl::updateGroup(int gidx)
{
    if (gidx == m_gidx) {
        return;
    }

    // switch to new voxels state context

    std::vector<int> old_ov_indices = m_voxels_indices;

    std::sort(old_ov_indices.begin(), old_ov_indices.end());

    std::vector<int> new_ov_indices = m_rcs.groupOutsideVoxelsStateIndices(gidx);
    std::sort(new_ov_indices.begin(), new_ov_indices.end());

    // get the indices of the voxels states that were outside the group but are
    // now inside and must be removed
    std::vector<int> ovidx_rem;
    std::set_difference(
            old_ov_indices.begin(), old_ov_indices.end(),
            new_ov_indices.begin(), new_ov_indices.end(),
            std::back_inserter(ovidx_rem));

    // get the indices of the voxels states that were inside the group but are
    // now outside and must be inserted
    std::vector<int> ovidx_ins;
    std::set_difference(
            new_ov_indices.begin(), new_ov_indices.end(),
            old_ov_indices.begin(), old_ov_indices.end(),
            std::back_inserter(ovidx_rem));

    // gather the voxels to be removed
    std::vector<Eigen::Vector3d> v_rem;
    for (int vsidx : ovidx_rem) {
        const CollisionVoxelsState& vs = m_rcs.voxelsState(vsidx);
        v_rem.insert(v_rem.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // gather the voxels to be inserted
    std::vector<Eigen::Vector3d> v_ins;
    for (int vsidx : ovidx_ins) {
        const CollisionVoxelsState& vs = m_rcs.voxelsState(vsidx);
        v_ins.insert(v_ins.end(), vs.voxels.begin(), vs.voxels.end());
    }

    // insert/remove the voxels
    if (!v_rem.empty()) {
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        m_grid->addPointsToField(v_ins);
    }

    // prepare voxels indices

    m_voxels_indices = std::move(new_ov_indices);

    // prepare sphere indices

    m_sphere_indices.clear();
    std::vector<int> ss_indices = m_rcs.groupSpheresStateIndices(gidx);
    for (int ssidx : ss_indices) {
        const CollisionSpheresState& spheres_state = m_rcs.spheresState(ssidx);
        std::vector<int> s_indices(spheres_state.spheres.size());
        int n = 0;
        std::generate(s_indices.begin(), s_indices.end(), [&]() { return n++; });
        for (int sidx : s_indices) {
            m_sphere_indices.emplace_back(ssidx, sidx);
        }
    }

    // sort sphere indices by priority
    std::sort(m_sphere_indices.begin(), m_sphere_indices.end(),
            [&](const SphereIndex& sidx1, const SphereIndex& sidx2)
            {
                const CollisionSphereState& ss1 = m_rcs.sphereState(sidx1);
                const CollisionSphereState& ss2 = m_rcs.sphereState(sidx2);
                const CollisionSphereModel* sph1 = ss1.model;
                const CollisionSphereModel* sph2 = ss2.model;
                return sph1->priority < sph2->priority;
            });

    // activate the group
    m_gidx = gidx;
}

void SelfCollisionModelImpl::switchAttachedBodyGroup(int ab_gidx)
{

}

void SelfCollisionModelImpl::copyState(const RobotCollisionState& state)
{
    for (size_t vidx = 0; vidx < m_rcs.model()->jointVarCount(); ++vidx) {
        const double p = state.jointVarPosition(vidx);
        m_rcs.setJointVarPosition(vidx, p);
    }
}

void SelfCollisionModelImpl::updateVoxelsStates()
{
    // update voxel groups; gather voxels before updating so as to impose only
    // a single distance field update (TODO: does the distance field recompute
    // with every call to insert/remove/update points?)
    std::vector<Eigen::Vector3d> v_rem;
    std::vector<Eigen::Vector3d> v_ins;
    for (int vsidx : m_voxels_indices) {
        if (m_rcs.voxelsStateDirty(vsidx)) {
            const CollisionVoxelsState& voxels_state = m_rcs.voxelsState(vsidx);

            // copy over voxels to be removed before updating
            v_rem.insert(
                    v_rem.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            m_rcs.updateVoxelsState(vsidx);

            // copy over voxels to be inserted
            v_ins.insert(
                    v_ins.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            ROS_DEBUG_NAMED(SCM_LOGGER, "Updating Occupancy Grid with change to Collision Voxels State (%zu displaced)", v_rem.size());
        }
    }

    // TODO: check for voxel state updates from attached objects

    // update occupancy grid with new voxel data
    if (!v_rem.empty()) {
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        m_grid->addPointsToField(v_ins);
    }
}

bool SelfCollisionModelImpl::checkVoxelsStateCollisions(double& dist)
{
    updateVoxelsStates();

    for (const auto& sidx : m_sphere_indices) {
        double obs_dist;
        if (!CheckSphereCollision(*m_grid, m_rcs, m_padding, sidx, obs_dist))
        {
            const CollisionSphereModel* sm = m_rcs.sphereState(sidx).model;
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

bool SelfCollisionModelImpl::checkAttachedBodyVoxelsStateCollisions(
    double& dist)
{
    return false;
}

bool SelfCollisionModelImpl::checkSpheresStateCollisions(double& dist)
{
    // check self collisions
    for (const SphereIndex& sidx1 : m_sphere_indices) {
        const CollisionSphereState& ss1 = m_rcs.sphereState(sidx1);
        const CollisionSphereModel& smodel1 = *ss1.model;

        for (const SphereIndex& sidx2 : m_sphere_indices) {
            const CollisionSphereState& ss2 = m_rcs.sphereState(sidx2);
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

bool SelfCollisionModelImpl::checkAttachedBodySpheresStateCollisions(
    double& dist)
{
    return false;
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

const AllowedCollisionMatrix& SelfCollisionModel::allowedCollisionMatrix() const
{
    return m_impl->allowedCollisionMatrix();
}

void SelfCollisionModel::updateAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    return m_impl->updateAllowedCollisionMatrix(acm);
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

double SelfCollisionModel::collisionDistance(
    RobotCollisionState& state,
    const std::string& group_name)
{
    return m_impl->collisionDistance(state, group_name);
}

double SelfCollisionModel::collisionDistance(
    RobotCollisionState& state,
    const int gidx)
{
    return m_impl->collisionDistance(state, gidx);
}

double SelfCollisionModel::collisionDistance(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const std::string& group_name)
{
    return m_impl->collisionDistance(state, ab_state, group_name);
}

double SelfCollisionModel::collisionDistance(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx)
{
    return m_impl->collisionDistance(state, ab_state, gidx);
}

} // namespace collision
} // namespace sbpl
