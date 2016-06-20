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

#ifndef sbpl_collision_detail_collision_model_impl_h
#define sbpl_collision_detail_collision_model_impl_h

#include <assert.h>

// COMPILE-TIME ASSERT = no assert
// RUNTIME UNRECOVERABLE ASSERT = assert
// RUNTIME RECOVERABLE ASSERT = exception

#define RANGE_ASSERT_COMPILE_TIME 0             // no assertion
#define RANGE_ASSERT_RUNTIME_UNRECOVERABLE 1    // assert macro
#define RANGE_ASSERT_RUNTIME_RECOVERABLE 2      // std::out_of_range exception
#define RANGE_ASSERT_METHOD RANGE_ASSERT_RUNTIME_RECOVERABLE

#if RANGE_ASSERT_METHOD == RANGE_ASSERT_RUNTIME_RECOVERABLE
#define ASSERT_RANGE(cond) \
{\
    if (!(cond)) {\
        throw std::out_of_range(#cond);\
    }\
}
#elif RANGE_ASSERT_METHOD == RANGE_ASSERT_RUNTIME_UNRECOVERABLE
#define ASSERT_RANGE(cond) \
{\
    assert(cond);\
}
#else
#define ASSERT_RANGE(cond)
#endif

#define ASSERT_VECTOR_RANGE(vector, index) \
ASSERT_RANGE(index >= 0 && index < vector.size());

namespace sbpl {
namespace collision {

inline
const std::string& CollisionModelImpl::name() const
{
    return m_name;
}

inline
const std::string& CollisionModelImpl::modelFrame() const
{
    return m_model_frame;
}

inline
size_t CollisionModelImpl::jointVarCount() const
{
    return m_jvar_names.size();
}

inline
const std::vector<std::string>& CollisionModelImpl::jointVarNames() const
{
    return m_jvar_names;
}

inline
bool CollisionModelImpl::hasJointVar(const std::string& joint_name) const
{
    return m_jvar_name_to_index.find(joint_name) != m_jvar_name_to_index.end();
}

inline
int CollisionModelImpl::jointVarIndex(const std::string& joint_name) const
{
    auto it = m_jvar_name_to_index.find(joint_name);
    ASSERT_RANGE(it != m_jvar_name_to_index.end());
    assert(it->second >= 0 && it->second < m_jvar_names.size());
    return it->second;
}

inline
const std::string& CollisionModelImpl::jointVarName(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_names, jidx);
    return m_jvar_names[jidx];
}

inline
bool CollisionModelImpl::jointVarIsContinuous(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_continuous[jidx];
}

inline
bool CollisionModelImpl::jointVarHasPositionBounds(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_has_position_bounds[jidx];
}

inline
double CollisionModelImpl::jointVarMaxPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_max_positions[jidx];
}

inline
double CollisionModelImpl::jointVarMinPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_min_positions[jidx];
}

inline
bool CollisionModelImpl::jointVarIsContinuous(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_continuous, jidx);
    return m_jvar_continuous[jidx];
}

inline
bool CollisionModelImpl::jointVarHasPositionBounds(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_has_position_bounds, jidx);
    return m_jvar_has_position_bounds[jidx];
}

inline
double CollisionModelImpl::jointVarMinPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_min_positions, jidx);
    return m_jvar_min_positions[jidx];
}

inline
double CollisionModelImpl::jointVarMaxPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_max_positions, jidx);
    return m_jvar_max_positions[jidx];
}

inline
size_t CollisionModelImpl::linkCount() const
{
    return m_link_names.size();
}

inline
const std::vector<std::string>& CollisionModelImpl::linkNames() const
{
    return m_link_names;
}

inline
bool CollisionModelImpl::hasLink(const std::string& link_name) const
{
    return m_link_name_to_index.find(link_name) != m_link_name_to_index.end();
}

inline
int CollisionModelImpl::linkIndex(const std::string& link_name) const
{
    auto it = m_link_name_to_index.find(link_name);
    ASSERT_RANGE(it != m_link_name_to_index.end());
    assert(it->second >= 0 && it->second < m_link_names.size());
    return it->second;
}

inline
const std::string& CollisionModelImpl::linkName(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_names, lidx);
    return m_link_names[lidx];
}

inline
size_t CollisionModelImpl::sphereModelCount() const
{
    return m_sphere_models.size();
}

inline
const CollisionSphereModel& CollisionModelImpl::sphereModel(int smidx) const
{
    ASSERT_VECTOR_RANGE(m_sphere_models, smidx);
    return m_sphere_models[smidx];
}

inline
bool CollisionModelImpl::hasSpheresModel(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_spheres_models[lidx] != nullptr;
}

inline
bool CollisionModelImpl::hasSpheresModel(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_spheres_models, lidx);
    return m_link_spheres_models[lidx] != nullptr;
}

inline
bool CollisionModelImpl::hasVoxelsModel(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_voxels_models[lidx] != nullptr;
}

inline
bool CollisionModelImpl::hasVoxelsModel(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_voxels_models, lidx);
    return m_link_voxels_models[lidx];
}

inline
size_t CollisionModelImpl::voxelsModelCount() const
{
    return m_voxels_models.size();
}

inline
const CollisionVoxelsModel& CollisionModelImpl::voxelsModel(int vmidx) const
{
    ASSERT_RANGE(vmidx >= 0 && vmidx < m_voxels_models.size());
    return m_voxels_models[vmidx];
}

inline
size_t CollisionModelImpl::groupCount() const
{
    return m_group_models.size();
}

inline
const std::vector<CollisionGroupModel>& CollisionModelImpl::groups() const
{
    return m_group_models;
}

inline
bool CollisionModelImpl::hasGroup(const std::string& group_name) const
{
    return m_group_name_to_index.find(group_name) !=
            m_group_name_to_index.end();
}

inline
int CollisionModelImpl::groupIndex(const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    assert(it->second >= 0 && it->second < m_group_models.size());
    return it->second;
}

inline
const std::string& CollisionModelImpl::groupName(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].name;
}

inline
const std::vector<int>& CollisionModelImpl::groupLinkIndices(
    const std::string& group_name) const
{
    const int gidx = groupIndex(group_name);
    return m_group_models[gidx].link_indices;
}

inline
const std::vector<int>& CollisionModelImpl::groupLinkIndices(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].link_indices;
}

inline
const std::vector<int>& CollisionModelImpl::groupSphereStateIndices(
    const std::string& group_name) const
{
    const int gidx = groupIndex(group_name);
    return m_group_states[gidx].sphere_indices;
}

inline
const std::vector<int>& CollisionModelImpl::groupSphereStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].sphere_indices;
}

inline
const std::vector<int>& CollisionModelImpl::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    const int gidx = groupIndex(group_name);
    return m_group_states[gidx].voxels_indices;
}

inline
const std::vector<int>& CollisionModelImpl::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].voxels_indices;
}

inline
const Eigen::Affine3d& CollisionModelImpl::worldToModelTransform() const
{
    return m_joint_origins[0];
}

inline
const std::vector<double>& CollisionModelImpl::jointPositions() const
{
    return m_jvar_positions;
}

inline
const Affine3dVector& CollisionModelImpl::linkTransforms() const
{
    return m_link_transforms;
}

inline
double CollisionModelImpl::jointPosition(const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_positions[jidx];
}

inline
double CollisionModelImpl::jointPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, jidx);
    return m_jvar_positions[jidx];
}

inline
bool CollisionModelImpl::setJointPosition(
    const std::string& joint_name,
    double position)
{
    const int jidx = jointVarIndex(joint_name);
    return setJointPosition(jidx, position);
}

inline
const Eigen::Affine3d& CollisionModelImpl::linkTransform(
    const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_transforms[lidx];
}

inline
const Eigen::Affine3d& CollisionModelImpl::linkTransform(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_transforms, lidx);
    return m_link_transforms[lidx];
}

inline
bool CollisionModelImpl::linkTransformDirty(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_dirty_link_transforms[lidx];
}

inline
bool CollisionModelImpl::linkTransformDirty(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    return m_dirty_link_transforms[lidx];
}

inline
bool CollisionModelImpl::updateLinkTransform(const std::string& link_name)
{
    const int lidx = linkIndex(link_name);
    return updateLinkTransform(lidx);
}

inline
const CollisionVoxelsState& CollisionModelImpl::voxelsState(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    return m_voxels_states[vsidx];
}

inline
bool CollisionModelImpl::voxelsStateDirty(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);
    return m_dirty_voxels_states[vsidx];
}

inline
const CollisionSphereState& CollisionModelImpl::sphereState(int ssidx) const
{
    ASSERT_VECTOR_RANGE(m_sphere_states, ssidx);
    return m_sphere_states[ssidx];
}

inline
bool CollisionModelImpl::sphereStateDirty(int ssidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, ssidx);
    return m_dirty_sphere_states[ssidx];
}

} // namespace collision
} // namespace sbpl

#endif
