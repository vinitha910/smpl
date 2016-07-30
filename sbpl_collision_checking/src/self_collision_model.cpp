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
//    m_acm.print(std::cout);
}

void SelfCollisionModelImpl::initAllowedCollisionMatrix()
{
    for (size_t lidx = 0; lidx < m_rcm->linkCount(); ++lidx) {
        const std::string& link_name = m_rcm->linkName(lidx);
        if (!m_acm.hasEntry(link_name)) {
            ROS_DEBUG_NAMED(SCM_LOGGER, "Adding entry '%s' to the ACM", link_name.c_str());
            m_acm.setEntry(link_name, false);
        }

        int pjidx = m_rcm->linkParentJointIndex(lidx);
        if (pjidx != 0) {
            const int plidx = m_rcm->jointParentLinkIndex(pjidx);
            const std::string& parent_link_name = m_rcm->linkName(plidx);
            if (!m_acm.hasEntry(parent_link_name)) {
                ROS_DEBUG_NAMED(SCM_LOGGER, "Adding entry '%s' to the ACM", parent_link_name.c_str());
                m_acm.setEntry(parent_link_name, false);
            }

            ROS_DEBUG_NAMED(SCM_LOGGER, "Allowing collisions between adjacent links '%s' and '%s'", link_name.c_str(), parent_link_name.c_str());
            m_acm.setEntry(link_name, parent_link_name, true);
        }

        for (int cjidx : m_rcm->linkChildJointIndices(lidx)) {
            int clidx = m_rcm->jointChildLinkIndex(cjidx);
            const std::string& child_link_name = m_rcm->linkName(clidx);
            if (!m_acm.hasEntry(child_link_name)) {
                ROS_DEBUG_NAMED(SCM_LOGGER, "Adding entry '%s' to the ACM", child_link_name.c_str());
                m_acm.setEntry(child_link_name, false);
            }

            ROS_DEBUG_NAMED(SCM_LOGGER, "Allowing collisions between adjacent links '%s' and '%s'", link_name.c_str(), child_link_name.c_str());
            m_acm.setEntry(link_name, child_link_name, true);
        }
    }

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
        ROS_ERROR_NAMED(SCM_LOGGER, "Collision State is not derived from appropriate Collision Model");
        return false;
    }

    if (!m_rcm->hasGroup(group_name)) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Self Collision Check is for non-existent group");
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
        ROS_ERROR_NAMED(SCM_LOGGER, "Collision State is not derived from appropriate Collision Model");
        return false;
    }

    if (gidx < 0 || gidx >= m_rcm->groupCount()) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Self Collision Check is for non-existent group");
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
    ROS_DEBUG_NAMED(SCM_LOGGER, "checkCollision(RobotCollisionState&, AttachedBodiesCollisionState&, const std::string&, double&)");
    if (state.model() != m_rcm) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Collision State is not derived from appropriate Collision Model");
        return false;
    }
    if (ab_state.model() != m_abcm) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Attached Bodies Collision State is not derived from appropriate Attached Bodies Collision Model");
        return false;
    }

    if (!m_rcm->hasGroup(group_name) || !m_abcm->hasGroup(group_name)) {
        ROS_ERROR_NAMED(SCM_LOGGER, "Self Collision Check is for non-existent group");
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

    ROS_DEBUG_NAMED(SCM_LOGGER, "Updating Self Collision Model from group %d to group %d", m_gidx, gidx);

    // switch to new voxels state context

    std::vector<int> old_ov_indices = m_voxels_indices;

    std::sort(old_ov_indices.begin(), old_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "Old Outside Voxels Indices: %s", to_string(old_ov_indices).c_str());

    std::vector<int> new_ov_indices = m_rcs.groupOutsideVoxelsStateIndices(gidx);
    std::sort(new_ov_indices.begin(), new_ov_indices.end());
    ROS_DEBUG_NAMED(SCM_LOGGER, "New Outside Voxels Indices: %s", to_string(new_ov_indices).c_str());

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

    ROS_DEBUG_NAMED(SCM_LOGGER, "  Removing %zu voxels from old voxels models", v_rem.size());
    ROS_DEBUG_NAMED(SCM_LOGGER, "  Inserting %zu voxels from new voxels models", v_ins.size());

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

    m_sphere_indices = GatherSphereIndices(m_rcs, gidx);

    // prepare the set of spheres states that should be checked for collision
    m_checked_spheres_states.clear();
    const std::vector<int>& group_link_indices = m_rcm->groupLinkIndices(gidx);
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
    ROS_DEBUG("Updating Voxels States");
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

            ROS_DEBUG_NAMED(SCM_LOGGER, "  Updating Occupancy Grid with change to Collision Voxels State (%zu displaced)", curr_size - prev_size);
        }
    }

    // TODO: check for voxel state updates from attached objects

    // update occupancy grid with new voxel data
    if (!v_rem.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Removing %zu voxels", v_rem.size());
        m_grid->removePointsFromField(v_rem);
    }
    if (!v_ins.empty()) {
        ROS_DEBUG_NAMED(SCM_LOGGER, "  Inserting %zu voxels", v_ins.size());
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

    return true;
}

bool SelfCollisionModelImpl::checkAttachedBodyVoxelsStateCollisions(
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Checking Attached Body Self Collisions against Voxels States");
    return true;
}

bool SelfCollisionModelImpl::checkSpheresStateCollisions(double& dist)
{
    for (const auto& ss_pair : m_checked_spheres_states) {
        int ss1idx = ss_pair.first;
        int ss2idx = ss_pair.second;
        const CollisionSpheresState& ss1 = m_rcs.spheresState(ss1idx);
        const CollisionSpheresState& ss2 = m_rcs.spheresState(ss2idx);

        for (size_t s1idx = 0; s1idx < ss1.spheres.size(); ++s1idx) {
            for (size_t s2idx = 0; s2idx < ss2.spheres.size(); ++s2idx) {
                // lazily update the state of the queried spheres
                const SphereIndex si1(ss1idx, s1idx);
                const SphereIndex si2(ss2idx, s2idx);
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
    }

    return true;
}

bool SelfCollisionModelImpl::checkAttachedBodySpheresStateCollisions(
    double& dist)
{
    ROS_DEBUG_NAMED(SCM_LOGGER, "Checking Attached Body Self Collisions against Spheres States");
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
