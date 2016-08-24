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

#ifndef sbpl_collision_robot_collision_state_h
#define sbpl_collision_robot_collision_state_h

// standard includes
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_collision_checking/base_collision_states.h>
#include <sbpl_collision_checking/robot_collision_model.h>

namespace sbpl {
namespace collision {

class RobotCollisionStateImpl;

class RobotCollisionState
{
public:

    RobotCollisionState(const RobotCollisionModel* model);

    const RobotCollisionModel* model() const;

    /// \name Robot State
    ///@{
    auto   worldToModelTransform() const -> const Eigen::Affine3d&;
    bool   setWorldToModelTransform(const Eigen::Affine3d& transform);

    auto   jointVarPositions() const -> const std::vector<double>&;
    auto   linkTransforms() const -> const Affine3dVector&;

    double jointVarPosition(const std::string& var_name) const;
    double jointVarPosition(int vidx) const;

    const double* getJointVarPositions() const;

    bool   setJointVarPosition(const std::string& var_name, double position);
    bool   setJointVarPosition(int vidx, double position);

    bool   setJointVarPositions(const double* positions);

    auto   linkTransform(const std::string& link_name) const ->
            const Eigen::Affine3d&;
    auto   linkTransform(int lidx) const -> const Eigen::Affine3d&;

    bool   linkTransformDirty(const std::string& link_name) const;
    bool   linkTransformDirty(int lidx) const;

    /// \brief Update the transforms of all links in the kinematic tree
    /// \return Whether the transform required updating; all link transforms
    ///         will be up to date in all cases afterwards
    bool   updateLinkTransforms();

    /// \brief Update the transform of a link in the kinematic tree
    /// \return Whether the transform required updating; the link transform will
    ///         be up to date in all cases afterwards
    bool   updateLinkTransform(int lidx);
    bool   updateLinkTransform(const std::string& link_name);
    ///@}

    /// \name CollisionState
    ///@{
    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    int  linkSpheresStateIndex(int lidx) const;
    auto spheresState(int ssidx) const -> const CollisionSpheresState&;

    auto sphereState(const SphereIndex& sidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(const SphereIndex& sidx) const;
    bool updateSphereStates();
    bool updateSphereStates(int ssidx);
    bool updateSphereState(const SphereIndex& sidx);

    /// \brief Return the indices of the collision sphere states belonging to
    ///        this group
    auto   groupSpheresStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupSpheresStateIndices(int gidx) const ->
            const std::vector<int>&;

    /// \brief Return the indices of the collision voxels states NOT belonging
    ///        to this group.
    auto  groupOutsideVoxelsStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;
    ///@}

    auto getVisualization() const -> visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const ->
            visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const -> visualization_msgs::MarkerArray;

private:

    const RobotCollisionModel*              m_model;

    /// \name Robot State
    ///@{
    std::vector<double>                     m_jvar_positions;
    std::vector<int>                        m_jvar_joints;
    std::vector<double*>                    m_joint_var_offsets;
    std::vector<bool>                       m_dirty_joint_transforms;
    Affine3dVector                          m_joint_transforms;
    std::vector<bool>                       m_dirty_link_transforms;
    Affine3dVector                          m_link_transforms;
    std::vector<int>                        m_link_transform_versions;
    ///@}

    /// \name Collision State
    ///@{
    // per spheres state sphere indices offset
    std::vector<int>                        m_sphere_offsets;

    // per spheres model
    std::vector<CollisionSpheresState>      m_spheres_states;

    // per voxels model
    std::vector<bool>                       m_dirty_voxels_states;
    std::vector<CollisionVoxelsState>       m_voxels_states;

    // per group model
    std::vector<CollisionGroupState>        m_group_states;

    // per-link references to corresponding spheres and voxels states
    std::vector<CollisionVoxelsState*>      m_link_voxels_states;
    std::vector<CollisionSpheresState*>     m_link_spheres_states;

    std::vector<int> m_q;
    std::vector<int> m_ancestors;
    ///@}

    void initRobotState();
    void initCollisionState();

    bool checkCollisionStateReferences() const;
};

static const char* RCS_LOGGER = "robot_state";

inline
RobotCollisionState::RobotCollisionState(
    const RobotCollisionModel* model)
:
    m_model(model),
    m_jvar_positions(),
    m_joint_var_offsets(),
    m_dirty_link_transforms(),
    m_link_transforms(),
    m_link_transform_versions(),
    m_sphere_offsets(),
    m_spheres_states(),
    m_dirty_voxels_states(),
    m_voxels_states(),
    m_group_states(),
    m_link_voxels_states(),
    m_link_spheres_states()
{
    initRobotState();
    initCollisionState();
}

inline
const RobotCollisionModel* RobotCollisionState::model() const
{
    return m_model;
}

inline
const Eigen::Affine3d& RobotCollisionState::worldToModelTransform() const
{
    return m_link_transforms[0];
}

inline
bool RobotCollisionState::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    if (!transform.isApprox(m_link_transforms[0], 0.0)) {
        m_link_transforms[0] = transform;
        std::fill(m_dirty_link_transforms.begin(), m_dirty_link_transforms.end(), true);
        m_dirty_link_transforms[0] = false;
        std::fill(m_dirty_voxels_states.begin(), m_dirty_voxels_states.end(), true);
        return true;
    }
    else {
        return false;
    }
}

inline
const std::vector<double>& RobotCollisionState::jointVarPositions() const
{
    return m_jvar_positions;
}

inline
const Affine3dVector& RobotCollisionState::linkTransforms() const
{
    return m_link_transforms;
}

inline
double RobotCollisionState::jointVarPosition(const std::string& var_name) const
{
    const int vidx = m_model->jointVarIndex(var_name);
    return m_jvar_positions[vidx];
}

inline
double RobotCollisionState::jointVarPosition(int vidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, vidx);
    return m_jvar_positions[vidx];
}

inline
const double* RobotCollisionState::getJointVarPositions() const
{
    return m_jvar_positions.data();
}

inline
bool RobotCollisionState::setJointVarPosition(
    const std::string& var_name,
    double position)
{
    const int vidx = m_model->jointVarIndex(var_name);
    return setJointVarPosition(vidx, position);
}

inline
const Eigen::Affine3d& RobotCollisionState::linkTransform(
    const std::string& link_name) const
{
    const int lidx = m_model->linkIndex(link_name);
    return m_link_transforms[lidx];
}

inline
const Eigen::Affine3d& RobotCollisionState::linkTransform(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_transforms, lidx);
    return m_link_transforms[lidx];
}

inline
bool RobotCollisionState::linkTransformDirty(const std::string& link_name) const
{
    const int lidx = m_model->linkIndex(link_name);
    return m_dirty_link_transforms[lidx];
}

inline
bool RobotCollisionState::linkTransformDirty(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    return m_dirty_link_transforms[lidx];
}

inline
bool RobotCollisionState::updateLinkTransforms()
{
    ROS_DEBUG_NAMED(RCS_LOGGER, "Updating all link transforms");
    bool updated = false;
    for (size_t lidx = 0; lidx < m_model->linkCount(); ++lidx) {
        updated |= updateLinkTransform(lidx);
    }
    return updated;
}

inline
bool RobotCollisionState::updateLinkTransform(int lidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    if (!m_dirty_link_transforms[lidx]) {
        return false;
    }

    int pjidx = m_model->linkParentJointIndex(lidx);
    int plidx = m_model->jointParentLinkIndex(pjidx);

    ROS_DEBUG_NAMED(RCS_LOGGER, "Updating transform for link '%s'. parent joint = %d, parent link = %d", m_model->linkName(lidx).c_str(), pjidx, plidx);

    if (plidx >= 0) {
        // TODO: optimize out this recursion
        updateLinkTransform(plidx);
        const Eigen::Affine3d& T_world_parent = m_link_transforms[plidx];

        if (m_dirty_joint_transforms[pjidx]) {
            JointTransformFunction fn = m_model->jointTransformFn(pjidx);
            const Eigen::Affine3d& joint_origin = m_model->jointOrigin(pjidx);
            const Eigen::Vector3d& joint_axis = m_model->jointAxis(pjidx);
            double* variables = m_joint_var_offsets[pjidx];
            m_joint_transforms[pjidx] = fn(joint_origin, joint_axis, variables);
            m_dirty_joint_transforms[pjidx] = false;
        }
        const Eigen::Affine3d& T_parent_link = m_joint_transforms[pjidx];
        m_link_transforms[lidx] = T_world_parent * T_parent_link;
    }

    ROS_DEBUG_NAMED(RCS_LOGGER, " -> %s", AffineToString(m_link_transforms[lidx]).c_str());

    m_dirty_link_transforms[lidx] = false;
    ++m_link_transform_versions[lidx];
    return true;
}

inline
bool RobotCollisionState::updateLinkTransform(const std::string& link_name)
{
    const int lidx = m_model->linkIndex(link_name);
    return updateLinkTransform(lidx);
}

inline
const CollisionVoxelsState& RobotCollisionState::voxelsState(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    return m_voxels_states[vsidx];
}

inline
bool RobotCollisionState::voxelsStateDirty(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);
    return m_dirty_voxels_states[vsidx];
}

inline
bool RobotCollisionState::updateVoxelsStates()
{
    ROS_DEBUG_NAMED(RCS_LOGGER, "Updating all voxels states");
    bool updated = false;
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        updated |= updateVoxelsState(vsidx);
    }
    return updated;
}

inline
bool RobotCollisionState::updateVoxelsState(int vsidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);

    if (!m_dirty_voxels_states[vsidx]) {
        return false;
    }

    CollisionVoxelsState& state = m_voxels_states[vsidx];

    const int lidx = state.model->link_index;
    updateLinkTransform(lidx);

    const Eigen::Affine3d& T_model_link = m_link_transforms[lidx];

    // transform voxels into the model frame
    std::vector<Eigen::Vector3d> new_voxels(state.model->voxels.size());
    for (size_t i = 0; i < state.model->voxels.size(); ++i) {
        new_voxels[i] = T_model_link * state.model->voxels[i];
    }

    state.voxels = std::move(new_voxels);
    m_dirty_voxels_states[vsidx] = false;
    return true;
}

inline
int RobotCollisionState::linkSpheresStateIndex(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_spheres_states, lidx);
    const CollisionSpheresState* ss = m_link_spheres_states[lidx];
    if (ss) {
        return std::distance(m_spheres_states.data(), ss);
    }
    else {
        return -1;
    }
}

inline
const CollisionSpheresState& RobotCollisionState::spheresState(
    int ssidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_states, ssidx);
    return m_spheres_states[ssidx];
}

inline
const CollisionSphereState& RobotCollisionState::sphereState(
    const SphereIndex& sidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_states, sidx.ss);
    ASSERT_VECTOR_RANGE(m_spheres_states[sidx.ss].spheres, sidx.s);
    return m_spheres_states[sidx.ss].spheres[sidx.s];
}

inline
bool RobotCollisionState::sphereStateDirty(const SphereIndex& sidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_states, sidx.ss);
    const CollisionSpheresState& spheres_state = m_spheres_states[sidx.ss];
    const int lidx = spheres_state.model->link_index;
    const int link_version = m_link_transform_versions[lidx];
    return m_dirty_link_transforms[lidx] || spheres_state.spheres[sidx.s].version != link_version;
}

inline
bool RobotCollisionState::updateSphereStates()
{
    ROS_DEBUG_NAMED(RCS_LOGGER, "Updating all sphere positions");
    bool updated = false;
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        updated |= updateSphereStates(ssidx);
    }
    return updated;
}

inline
bool RobotCollisionState::updateSphereStates(int ssidx)
{
    bool updated = false;
    const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
    for (size_t sidx = 0; sidx < spheres_state.spheres.size(); ++sidx) {
        updated |= updateSphereState(SphereIndex(ssidx, sidx));
    }
    return updated;
}

inline
bool RobotCollisionState::updateSphereState(const SphereIndex& sidx)
{
    CollisionSpheresState& spheres_state = m_spheres_states[sidx.ss];
    const int lidx = spheres_state.model->link_index;
    const int link_version = m_link_transform_versions[lidx];
    CollisionSphereState& sphere_state = spheres_state.spheres[sidx.s];

    if (!m_dirty_link_transforms[lidx] && sphere_state.version == link_version) {
        return false;
    }

    updateLinkTransform(lidx);

    ROS_DEBUG_NAMED(RCS_LOGGER, "Updating position of sphere '%s'", sphere_state.model->name.c_str());
    const Eigen::Affine3d& T_model_link = m_link_transforms[lidx];
    sphere_state.pos = T_model_link * sphere_state.model->center;

    // version may have updated since before
    sphere_state.version = m_link_transform_versions[lidx]; //link_version;
    return true;
}

inline
const std::vector<int>& RobotCollisionState::groupSpheresStateIndices(
    const std::string& group_name) const
{
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>& RobotCollisionState::groupSpheresStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>& RobotCollisionState::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].voxels_indices;
}

inline
const std::vector<int>& RobotCollisionState::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].voxels_indices;
}

inline
visualization_msgs::MarkerArray
RobotCollisionState::getVisualization() const
{
    // TODO: cull out spheres duplicated between groups
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < m_model->groupCount(); ++i) {
        auto marr = getVisualization(i);
        auto prev_size = ma.markers.size();
        ma.markers.insert(ma.markers.end(), marr.markers.begin(), marr.markers.end());
        for (size_t i = prev_size; i < ma.markers.size(); ++i) {
            ma.markers[i].id = i;
        }
    }
    return ma;
}

inline
visualization_msgs::MarkerArray
RobotCollisionState::getVisualization(
    const std::string& group_name) const
{
    const int gidx = m_model->groupIndex(group_name);
    return getVisualization(gidx);
}

typedef std::shared_ptr<RobotCollisionState> RobotCollisionStatePtr;
typedef std::shared_ptr<const RobotCollisionState> RobotCollisionStateConstPtr;

} // namespace collision
} // namespace sbpl

#endif
