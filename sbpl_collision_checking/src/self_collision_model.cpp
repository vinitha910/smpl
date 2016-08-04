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

// system includes
#include <leatherman/print.h>

// project includes
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
        const int gidx,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const int gidx,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        const AllowedCollisionsInterface& aci,
        const int gidx,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const int gidx,
        double& dist);

    double collisionDistance(
        RobotCollisionState& state,
        const int gidx);

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

    // set of spheres state pairs that should be checked for self collisions
    std::vector<std::pair<int, int>>        m_checked_spheres_states;

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
    bool checkSpheresStateCollisions(
        const AllowedCollisionsInterface& aci,
        double& dist);
    bool checkSpheresStateCollision(
        const int ss1i, const int ss2i,
        const CollisionSpheresState& ss1,
        const CollisionSpheresState& ss2,
        double& dist);
    bool checkAttachedBodySpheresStateCollisions(double& dist);

    void updateCheckedSpheresIndices();
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
//    m_acm.print(std::cout);
}

void SelfCollisionModelImpl::initAllowedCollisionMatrix()
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Creating adjacent link entries in the allowed collision matrix");
    for (size_t lidx = 0; lidx < m_rcm->linkCount(); ++lidx) {
        const std::string& link_name = m_rcm->linkName(lidx);
        if (!m_acm.hasEntry(link_name)) {
            m_acm.setEntry(link_name, false);
        }

        int pjidx = m_rcm->linkParentJointIndex(lidx);
        if (pjidx != 0) {
            const int plidx = m_rcm->jointParentLinkIndex(pjidx);
            const std::string& parent_link_name = m_rcm->linkName(plidx);
            if (!m_acm.hasEntry(parent_link_name)) {
                m_acm.setEntry(parent_link_name, false);
            }

            m_acm.setEntry(link_name, parent_link_name, true);
        }

        for (int cjidx : m_rcm->linkChildJointIndices(lidx)) {
            int clidx = m_rcm->jointChildLinkIndex(cjidx);
            const std::string& child_link_name = m_rcm->linkName(clidx);
            if (!m_acm.hasEntry(child_link_name)) {
                m_acm.setEntry(child_link_name, false);
            }

            m_acm.setEntry(link_name, child_link_name, true);
        }
    }
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
    ROS_DEBUG_NAMED(SCM_LOGGER, "Update allowed collision matrix");
    std::vector<std::string> all_entries;

    acm.getAllEntryNames(all_entries);

    collision_detection::AllowedCollision::Type type;
    for (size_t i = 0; i < all_entries.size(); ++i) {
        for (size_t j = i + 1; j < all_entries.size(); ++j) {
            const std::string& entry1 = all_entries[i];
            const std::string& entry2 = all_entries[j];
            if (acm.getEntry(entry1, entry2, type)) {
                if (type != collision_detection::AllowedCollision::NEVER) {
                    m_acm.setEntry(entry1, entry2, false);
                }
                else {
                    m_acm.setEntry(entry1, entry2, true);
                }
            }
        }
    }
    updateCheckedSpheresIndices();
}

void SelfCollisionModelImpl::setAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Overwrite allowed collision matrix");
    m_acm = acm;
    updateCheckedSpheresIndices();
}

void SelfCollisionModelImpl::setPadding(double padding)
{
    m_padding = padding;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    const int gidx,
    double& dist)
{
    if (state.model() != m_rcm) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Collision State is not derived from appropriate Collision Model");
        return false;
    }

    if (gidx < 0 || gidx >= m_rcm->groupCount()) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Self collision check is for non-existent group");
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
    const int gidx,
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "checkCollision(RobotCollisionState& state, AttachedBodiesCollisionState&, const int, double&)");
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

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    const AllowedCollisionsInterface& aci,
    const int gidx,
    double& dist)
{
    if (state.model() != m_rcm) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Collision State is not derived from appropriate Collision Model");
        return false;
    }

    if (gidx < 0 || gidx >= m_rcm->groupCount()) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Self collision check is for non-existent group");
        return false;
    }

    updateGroup(gidx);
    copyState(state);

    if (!checkVoxelsStateCollisions(dist) ||
        !checkSpheresStateCollisions(aci, dist))
    {
        return false;
    }

    return true;
}

bool SelfCollisionModelImpl::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const AllowedCollisionsInterface& aci,
    const int gidx,
    double& dist)
{
    // TODO: implement me
    return false;
}

double SelfCollisionModelImpl::collisionDistance(
    RobotCollisionState& state,
    const int gidx)
{
    // TODO: implement me
    return 0.0;
}

double SelfCollisionModelImpl::collisionDistance(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx)
{
    // TODO: implement me
    return 0.0;
}

void SelfCollisionModelImpl::updateGroup(int gidx)
{
    if (gidx == m_gidx) {
        return;
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "Update Self Collision Model from group %d to group %d", m_gidx, gidx);

    // switch to new voxels state context

    std::vector<int> old_ov_indices = m_voxels_indices;

    std::sort(old_ov_indices.begin(), old_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "Old outside voxels indices: %s", to_string(old_ov_indices).c_str());

    std::vector<int> new_ov_indices = m_rcs.groupOutsideVoxelsStateIndices(gidx);
    std::sort(new_ov_indices.begin(), new_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "New outside voxels indices: %s", to_string(new_ov_indices).c_str());

    // get the indices of the voxels states that were outside the group but are
    // now inside and must be removed
    std::vector<int> ovidx_rem;
    std::set_difference(
            old_ov_indices.begin(), old_ov_indices.end(),
            new_ov_indices.begin(), new_ov_indices.end(),
            std::back_inserter(ovidx_rem));
    ROS_DEBUG_NAMED(SCM_LOGGER, "ovidx_rem: %s", to_string(ovidx_rem).c_str());

    // get the indices of the voxels states that were inside the group but are
    // now outside and must be inserted
    std::vector<int> ovidx_ins;
    std::set_difference(
            new_ov_indices.begin(), new_ov_indices.end(),
            old_ov_indices.begin(), old_ov_indices.end(),
            std::back_inserter(ovidx_ins));
    ROS_DEBUG_NAMED(SCM_LOGGER, "ovidx_ins: %s", to_string(ovidx_ins).c_str());

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
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Remove %zu voxels from old voxels models", v_rem.size());
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Insert %zu voxels from new voxels models", v_ins.size());
        m_grid->addPointsToField(v_ins);
    }

    // prepare voxels indices

    m_voxels_indices = std::move(new_ov_indices);

    // prepare sphere indices

    m_sphere_indices = GatherSphereIndices(m_rcs, gidx);

    // activate the group
    m_gidx = gidx;

    // prepare the set of spheres states that should be checked for collision
    updateCheckedSpheresIndices();
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
    ROS_DEBUG("Update voxels states");
    // update voxel groups; gather voxels before updating so as to impose only
    // a single distance field update (TODO: does the distance field recompute
    // with every call to insert/remove/update points?)
    std::vector<Eigen::Vector3d> v_rem;
    std::vector<Eigen::Vector3d> v_ins;
    for (int vsidx : m_voxels_indices) {
        if (m_rcs.voxelsStateDirty(vsidx)) {
            const CollisionVoxelsState& voxels_state = m_rcs.voxelsState(vsidx);

            // copy over voxels to be removed before updating
            const size_t prev_size = v_rem.size();
            v_rem.insert(
                    v_rem.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());
            const size_t curr_size = v_rem.size();

            m_rcs.updateVoxelsState(vsidx);

            // copy over voxels to be inserted
            v_ins.insert(
                    v_ins.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            ROS_DEBUG_NAMED(SCM_LOGGER, "  Update Occupancy Grid with change to Collision Voxels State (%zu displaced)", curr_size - prev_size);
        }
    }

    // TODO: check for voxel state updates from attached objects

    // update occupancy grid with new voxel data
    if (!v_rem.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Remove %zu voxels", v_rem.size());
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Insert %zu voxels", v_ins.size());
        m_grid->addPointsToField(v_ins);
    }
}

bool SelfCollisionModelImpl::checkVoxelsStateCollisions(double& dist)
{
    updateVoxelsStates();

    for (const auto& sidx : m_sphere_indices) {
        double obs_dist;
        if (!CheckSphereCollision(*m_grid, m_rcs, m_padding, sidx, obs_dist)) {
            const CollisionSphereModel* sm = m_rcs.sphereState(sidx).model;
            ROS_DEBUG_NAMED(SCM_LOGGER, "    *collision* idx: %s, name: %s, radius: %0.3fm, dist: %0.3fm", to_string(sidx).c_str(), sm->name.c_str(), sm->radius, obs_dist);
            dist = obs_dist;
            return false;
        }

        if (obs_dist < dist) {
            dist = obs_dist;
        }
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "No voxels collisions");
    return true;
}

bool SelfCollisionModelImpl::checkAttachedBodyVoxelsStateCollisions(
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check attached body self collisions against voxels states");
    return true;
}

bool SelfCollisionModelImpl::checkSpheresStateCollisions(double& dist)
{
    for (const auto& ss_pair : m_checked_spheres_states) {
        int ss1idx = ss_pair.first;
        int ss2idx = ss_pair.second;
        const CollisionSpheresState& ss1 = m_rcs.spheresState(ss1idx);
        const CollisionSpheresState& ss2 = m_rcs.spheresState(ss2idx);

        if (!checkSpheresStateCollision(ss1idx, ss2idx, ss1, ss2, dist)) {
            return false;
        }
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "No spheres collisions");
    return true;
}

bool SelfCollisionModelImpl::checkSpheresStateCollisions(
    const AllowedCollisionsInterface& aci,
    double& dist)
{
    const auto& group_link_indices = m_rcm->groupLinkIndices(m_gidx);
    for (int l1 = 0; l1 < group_link_indices.size(); ++l1) {
        const int lidx1 = group_link_indices[l1];
        if (!m_rcm->hasSpheresModel(lidx1)) {
            continue;
        }

        const std::string& l1_name = m_rcm->linkName(lidx1);
        for (int l2 = l1 + 1; l2 < group_link_indices.size(); ++l2) {
            const int lidx2 = group_link_indices[l2];
            if (!m_rcm->hasSpheresModel(lidx2)) {
                continue;
            }
            const std::string& l2_name = m_rcm->linkName(lidx2);

            AllowedCollision::Type type;
            if (aci.getEntry(l2_name, l1_name, type) &&
                type == AllowedCollision::Type::ALWAYS)
            {
                // collisions allowed between this pair of links
                continue;
            }

            const int ss1i = m_rcs.linkSpheresStateIndex(lidx1);
            const int ss2i = m_rcs.linkSpheresStateIndex(lidx2);
            const CollisionSpheresState& ss1 = m_rcs.spheresState(ss1i);
            const CollisionSpheresState& ss2 = m_rcs.spheresState(ss2i);
            if (!checkSpheresStateCollision(ss1i, ss2i, ss1, ss2, dist)) {
                return false;
            }
        }
    }

    ROS_DEBUG_NAMED(SCM_LOGGER, "No spheres collisions");
    return true;
}

bool SelfCollisionModelImpl::checkSpheresStateCollision(
    int ss1i,
    int ss2i,
    const CollisionSpheresState& ss1,
    const CollisionSpheresState& ss2,
    double& dist)
{
    for (size_t s1idx = 0; s1idx < ss1.spheres.size(); ++s1idx) {
        for (size_t s2idx = 0; s2idx < ss2.spheres.size(); ++s2idx) {
            // lazily update the state of the queried spheres
            const SphereIndex si1(ss1i, s1idx);
            const SphereIndex si2(ss2i, s2idx);
            m_rcs.updateSphereState(si1);
            m_rcs.updateSphereState(si2);

            const CollisionSphereState& sphere_state_1 = m_rcs.sphereState(si1);
            const CollisionSphereModel& sphere_model_1 = *sphere_state_1.model;

            const CollisionSphereState& sphere_state_2 = m_rcs.sphereState(si2);
            const CollisionSphereModel& sphere_model_2 = *sphere_state_2.model;

            Eigen::Vector3d dx = sphere_state_2.pos - sphere_state_1.pos;
            const double radius_combined = sphere_model_1.radius + sphere_model_2.radius;
            if (dx.squaredNorm() < radius_combined * radius_combined) {
                // check for an entry for these particular spheres
                collision_detection::AllowedCollision::Type type;
                if (m_acm.getEntry(sphere_model_1.name, sphere_model_2.name, type)) {
                    if (type != collision_detection::AllowedCollision::ALWAYS) {
                        ROS_DEBUG_NAMED(SCM_LOGGER, "  *collision* '%s' x '%s'", sphere_model_1.name.c_str(), sphere_model_2.name.c_str());
                        return false;
                    }
                }
                else {
                    ROS_DEBUG_NAMED(SCM_LOGGER, "  *collision* '%s' x '%s'", sphere_model_1.name.c_str(), sphere_model_2.name.c_str());
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
    ROS_DEBUG_NAMED(SCM_LOGGER, "Check attached body self collisions against spheres states");
    return true;
}

void SelfCollisionModelImpl::updateCheckedSpheresIndices()
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Update checked sphere indices");
    m_checked_spheres_states.clear();

    if (m_gidx == -1) {
        return;
    }

    const std::vector<int>& group_link_indices = m_rcm->groupLinkIndices(m_gidx);
    for (int l1 = 0; l1 < m_rcm->linkCount(); ++l1) {
        bool l1_in_group = std::find(
                group_link_indices.begin(),
                group_link_indices.end(),
                l1) != group_link_indices.end();
        bool l1_has_spheres = m_rcm->hasSpheresModel(l1);
        if (!l1_in_group || !l1_has_spheres) {
            continue;
        }
        const std::string& l1_name = m_rcm->linkName(l1);
        for (int l2 = l1 + 1; l2 < m_rcm->linkCount(); ++l2) {
            const bool l2_in_group = std::find(
                    group_link_indices.begin(),
                    group_link_indices.end(), l2) != group_link_indices.end();
            const bool l2_has_spheres = m_rcm->hasSpheresModel(l2);
            if (!l2_in_group || !l2_has_spheres) {
                continue;
            }
            const std::string& l2_name = m_rcm->linkName(l2);

            collision_detection::AllowedCollision::Type type;
            if (m_acm.getEntry(l1_name, l2_name, type)) {
                if (type != collision_detection::AllowedCollision::ALWAYS) {
                    m_checked_spheres_states.emplace_back(
                            m_rcs.linkSpheresStateIndex(l1),
                            m_rcs.linkSpheresStateIndex(l2));
                }
            }
            else {
                m_checked_spheres_states.emplace_back(
                        m_rcs.linkSpheresStateIndex(l1),
                        m_rcs.linkSpheresStateIndex(l2));
            }
        }
    }
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
    const int gidx,
    double& dist)
{
    return m_impl->checkCollision(state, gidx, dist);
}

bool SelfCollisionModel::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx,
    double& dist)
{
    return m_impl->checkCollision(state, ab_state, gidx, dist);
}

bool SelfCollisionModel::checkCollision(
    RobotCollisionState& state,
    const AllowedCollisionsInterface& aci,
    const int gidx,
    double& dist)
{
    return m_impl->checkCollision(state, aci, gidx, dist);
}

bool SelfCollisionModel::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const AllowedCollisionsInterface& aci,
    const int gidx,
    double& dist)
{
    return m_impl->checkCollision(state, ab_state, aci, gidx, dist);
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
    const int gidx)
{
    return m_impl->collisionDistance(state, ab_state, gidx);
}

} // namespace collision
} // namespace sbpl
