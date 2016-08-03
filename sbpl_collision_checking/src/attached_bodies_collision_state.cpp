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

#include <algorithm>

#include <sbpl_collision_checking/attached_bodies_collision_state.h>
#include "debug.h"

namespace sbpl {
namespace collision {

class AttachedBodiesCollisionStateImpl
{
public:

    AttachedBodiesCollisionStateImpl(
        const AttachedBodiesCollisionModel* model,
        RobotCollisionState* state);
    ~AttachedBodiesCollisionStateImpl();

    const AttachedBodiesCollisionModel* model();
    RobotCollisionState* state();

    ///\name Attached Bodies State
    ///@{
    auto attachedBodyTransform(const std::string& id) const
            -> const Eigen::Affine3d&;
    auto attachedBodyTransform(int abidx) const -> const Eigen::Affine3d&;

    bool attachedBodyTransformDirty(const std::string& id) const;
    bool attachedBodyTransformDirty(int abidx) const;

    bool updateAttachedBodyTransforms();
    bool updateAttachedBodyTransform(const std::string& id);
    bool updateAttachedBodyTransform(int abidx);
    ///@}

    /// \name Attached Bodies Collision State
    ///@{
    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto spheresState(int ssidx) const -> const CollisionSpheresState&;

    auto sphereState(const SphereIndex& sidx) const
            -> const CollisionSphereState&;
    bool sphereStateDirty(const SphereIndex& sidx) const;
    bool updateSphereStates();
    bool updateSphereStates(int ssidx);
    bool updateSphereState(const SphereIndex& sidx);

    auto groupSpheresStateIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto groupSpheresStateIndices(int gidx) const -> const std::vector<int>&;

    auto groupOutsideVoxelsStateIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto groupOutsideVoxelsStateIndices(int gidx) const
            -> const std::vector<int>&;
    ///@}

    auto getVisualization() const -> visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const
            -> visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const -> visualization_msgs::MarkerArray;

private:

    const AttachedBodiesCollisionModel*     m_model;
    RobotCollisionState*                    m_state;

    std::vector<bool>                       m_dirty_sphere_states;
    std::vector<int>                        m_sphere_offsets;
    std::vector<CollisionSpheresState>      m_spheres_states;
    std::vector<bool>                       m_dirty_voxels_states;
    std::vector<CollisionVoxelsState>       m_voxels_states;
    std::vector<CollisionGroupState>        m_group_states;

    hash_map<int, CollisionSpheresState*>   m_attached_body_spheres_states;
    hash_map<int, CollisionVoxelsState*>    m_attached_body_voxels_states;

    int m_version;

    void reinitCollisionState() const;
    void reinitCollisionState();

    int sphereIndex(const SphereIndex& sidx) const;
};

AttachedBodiesCollisionStateImpl::AttachedBodiesCollisionStateImpl(
    const AttachedBodiesCollisionModel* model,
    RobotCollisionState* state)
:
    m_model(model),
    m_state(state),
    m_dirty_sphere_states(),
    m_sphere_offsets(),
    m_dirty_voxels_states(),
    m_voxels_states(),
    m_group_states(),
    m_attached_body_spheres_states(),
    m_attached_body_voxels_states(),
    m_version(-1) // -1 to force first initial reinitialization
{
}

AttachedBodiesCollisionStateImpl::~AttachedBodiesCollisionStateImpl()
{
}

inline
const AttachedBodiesCollisionModel* AttachedBodiesCollisionStateImpl::model()
{
    return m_model;
}

inline
RobotCollisionState* AttachedBodiesCollisionStateImpl::state()
{
    return m_state;
}

inline
const Eigen::Affine3d& AttachedBodiesCollisionStateImpl::attachedBodyTransform(
    const std::string& id) const
{
    reinitCollisionState();
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransform(lidx);
}

inline
const Eigen::Affine3d& AttachedBodiesCollisionStateImpl::attachedBodyTransform(
    int abidx) const
{
    reinitCollisionState();
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransform(lidx);
}

inline
bool AttachedBodiesCollisionStateImpl::attachedBodyTransformDirty(
    const std::string& id) const
{
    reinitCollisionState();
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransformDirty(lidx);
}

inline
bool AttachedBodiesCollisionStateImpl::attachedBodyTransformDirty(
    int abidx) const
{
    reinitCollisionState();
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransformDirty(lidx);
}

inline
bool AttachedBodiesCollisionStateImpl::updateAttachedBodyTransforms()
{
    reinitCollisionState();
    return m_state->updateLinkTransforms();
}

inline
bool AttachedBodiesCollisionStateImpl::updateAttachedBodyTransform(
    const std::string& id)
{
    reinitCollisionState();
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->updateLinkTransform(lidx);
}

inline
bool AttachedBodiesCollisionStateImpl::updateAttachedBodyTransform(int abidx)
{
    reinitCollisionState();
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->updateLinkTransform(lidx);
}

inline
const CollisionVoxelsState& AttachedBodiesCollisionStateImpl::voxelsState(
    int vsidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    return m_voxels_states[vsidx];
}

inline
bool AttachedBodiesCollisionStateImpl::voxelsStateDirty(int vsidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    return m_dirty_voxels_states[vsidx];
}

inline
bool AttachedBodiesCollisionStateImpl::updateVoxelsStates()
{
    reinitCollisionState();
    bool updated = false;
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        updated |= updateVoxelsState(vsidx);
    }
    return updated;
}

inline
bool AttachedBodiesCollisionStateImpl::updateVoxelsState(int vsidx)
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);

    if (!m_dirty_voxels_states[vsidx]) {
        return false;
    }

    CollisionVoxelsState& state = m_voxels_states[vsidx];

    const int bidx = state.model->link_index;
    updateAttachedBodyTransform(bidx);

    const Eigen::Affine3d& T_model_body = attachedBodyTransform(bidx);

    // transform voxels into the model frame
    std::vector<Eigen::Vector3d> new_voxels(state.model->voxels.size());
    for (size_t i = 0; i < state.model->voxels.size(); ++i) {
        new_voxels[i] = T_model_body * state.model->voxels[i];
    }

    state.voxels = std::move(new_voxels);
    m_dirty_voxels_states[vsidx] = false;
    return true;
}

inline
const CollisionSpheresState& AttachedBodiesCollisionStateImpl::spheresState(
    int ssidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_spheres_states, ssidx);
    return m_spheres_states[ssidx];
}

inline
const CollisionSphereState& AttachedBodiesCollisionStateImpl::sphereState(
    const SphereIndex& sidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_spheres_states, sidx.ss);
    ASSERT_VECTOR_RANGE(m_spheres_states[sidx.ss].spheres, sidx.s);
    return m_spheres_states[sidx.ss].spheres[sidx.s];
}

inline
bool AttachedBodiesCollisionStateImpl::sphereStateDirty(
    const SphereIndex& sidx) const
{
    reinitCollisionState();
    const int idx = sphereIndex(sidx);
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, idx);
    return m_dirty_sphere_states[idx];
}

inline
bool AttachedBodiesCollisionStateImpl::updateSphereStates()
{
    reinitCollisionState();
    bool updated = false;
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        updated |= updateSphereStates(ssidx);
    }
    return updated;
}

inline
bool AttachedBodiesCollisionStateImpl::updateSphereStates(int ssidx)
{
    reinitCollisionState();
    bool updated = false;
    const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
    for (size_t sidx = 0; sidx < spheres_state.spheres.size(); ++sidx) {
        updated |= updateSphereState(SphereIndex(ssidx, sidx));
    }
    return updated;
}

inline
bool AttachedBodiesCollisionStateImpl::updateSphereState(
    const SphereIndex& sidx)
{
    reinitCollisionState();
    const int idx = sphereIndex(sidx);
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, idx);

    if (!m_dirty_sphere_states[idx]) {
        return false;
    }

    CollisionSphereState& sphere_state = m_spheres_states[sidx.ss].spheres[sidx.s];

    const int abidx = sphere_state.parent_state->model->link_index;
    updateAttachedBodyTransform(abidx);

    const Eigen::Affine3d& T_model_body = attachedBodyTransform(abidx);
    sphere_state.pos = T_model_body * sphere_state.model->center;

    m_dirty_sphere_states[idx] = false;
    return true;
}

inline
const std::vector<int>&
AttachedBodiesCollisionStateImpl::groupSpheresStateIndices(
    const std::string& group_name) const
{
    reinitCollisionState();
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>&
AttachedBodiesCollisionStateImpl::groupSpheresStateIndices(int gidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>&
AttachedBodiesCollisionStateImpl::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    reinitCollisionState();
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].voxels_indices;
}

inline
const std::vector<int>&
AttachedBodiesCollisionStateImpl::groupOutsideVoxelsStateIndices(int gidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].voxels_indices;
}

inline
visualization_msgs::MarkerArray
AttachedBodiesCollisionStateImpl::getVisualization() const
{
    reinitCollisionState();
    return visualization_msgs::MarkerArray();
}

inline
visualization_msgs::MarkerArray
AttachedBodiesCollisionStateImpl::getVisualization(
    const std::string& group_name) const
{
    reinitCollisionState();
    return visualization_msgs::MarkerArray();
}

inline
visualization_msgs::MarkerArray
AttachedBodiesCollisionStateImpl::getVisualization(int gidx) const
{
    reinitCollisionState();
    return visualization_msgs::MarkerArray();
}

inline
int AttachedBodiesCollisionStateImpl::sphereIndex(const SphereIndex& sidx) const
{
    return m_sphere_offsets[sidx.ss] + sidx.s;
}

inline
void AttachedBodiesCollisionStateImpl::reinitCollisionState() const
{
    AttachedBodiesCollisionStateImpl* mm =
            const_cast<AttachedBodiesCollisionStateImpl*>(this);
    return mm->reinitCollisionState();
}

inline
void AttachedBodiesCollisionStateImpl::reinitCollisionState()
{
    if (m_version == m_model->version()) {
        return;
    }

    m_dirty_sphere_states.assign(m_model->sphereModelCount(), true);
    m_sphere_offsets.assign(m_model->spheresModelCount(), 0);
    m_spheres_states.assign(
            m_model->spheresModelCount(),
            CollisionSpheresState());
    int offset = 0;
    for (size_t i = 0; i < m_model->spheresModelCount(); ++i) {
        m_sphere_offsets[i] = offset;
        const CollisionSpheresModel& spheres_model = m_model->spheresModel(i);
        CollisionSpheresState& spheres_state = m_spheres_states[i];

        spheres_state.model = &spheres_model;

        spheres_state.spheres.buildFrom(&spheres_state);
        offset += spheres_model.spheres.size();
    }

    // initialize voxels states
    m_dirty_voxels_states.assign(m_model->voxelsModelCount(), true);
    m_voxels_states.assign(m_model->voxelsModelCount(), CollisionVoxelsState());

    for (size_t i = 0; i < m_model->voxelsModelCount(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
        CollisionVoxelsState& voxels_state = m_voxels_states[i];
        // duplicate voxels from voxels model
        voxels_state.model = &voxels_model;
        voxels_state.voxels = voxels_model.voxels;
    }

    // initialize group states
    m_group_states.assign(m_model->groupCount(), CollisionGroupState());
    for (size_t i = 0; i < m_model->groupCount(); ++i) {
        const CollisionGroupModel& group_model = m_model->group(i);
        CollisionGroupState& group_state = m_group_states[i];
        // map group state -> group model
        group_state.model = &group_model;
    }

    // map group state -> spheres states
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        // get the link this spheres state is attached to
        const int lidx = spheres_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) !=
                group_model.link_indices.end())
            {
                m_group_states[gidx].spheres_indices.push_back(ssidx);
            }
        }
    }

    // map group state -> (outside) voxels states
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        const CollisionVoxelsState& voxels_state = m_voxels_states[vsidx];
        // get the link this spheres state is attached to
        const int lidx = voxels_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) ==
                group_model.link_indices.end())
            {
                m_group_states[gidx].voxels_indices.push_back(vsidx);
            }
        }
    }

    // TODO: initialize attached bodies voxels states
    // TODO: initialize attached bodies spheres states

    m_version = m_model->version();
}

/////////////////////////////////////////////////
// AttachedBodiesCollisionState Implementation //
/////////////////////////////////////////////////

AttachedBodiesCollisionState::AttachedBodiesCollisionState(
    const AttachedBodiesCollisionModel* model,
    RobotCollisionState* state)
:
    m_impl(new AttachedBodiesCollisionStateImpl(model, state))
{
}

AttachedBodiesCollisionState::~AttachedBodiesCollisionState()
{
}

const AttachedBodiesCollisionModel* AttachedBodiesCollisionState::model()
{
    return m_impl->model();
}

RobotCollisionState* AttachedBodiesCollisionState::state()
{
    return m_impl->state();
}

const Eigen::Affine3d& AttachedBodiesCollisionState::attachedBodyTransform(
    const std::string& id) const
{
    return m_impl->attachedBodyTransform(id);
}

const Eigen::Affine3d& AttachedBodiesCollisionState::attachedBodyTransform(
    int abidx) const
{
    return m_impl->attachedBodyTransform(abidx);
}

bool AttachedBodiesCollisionState::attachedBodyTransformDirty(
    const std::string& id) const
{
    return m_impl->attachedBodyTransformDirty(id);
}

bool AttachedBodiesCollisionState::attachedBodyTransformDirty(int abidx) const
{
    return m_impl->attachedBodyTransformDirty(abidx);
}

bool AttachedBodiesCollisionState::updateAttachedBodyTransforms()
{
    return m_impl->updateAttachedBodyTransforms();
}

bool AttachedBodiesCollisionState::updateAttachedBodyTransform(
    const std::string& id)
{
    return m_impl->updateAttachedBodyTransform(id);
}

bool AttachedBodiesCollisionState::updateAttachedBodyTransform(int abidx)
{
    return m_impl->updateAttachedBodyTransform(abidx);
}

const CollisionVoxelsState& AttachedBodiesCollisionState::voxelsState(
    int vsidx) const
{
    return m_impl->voxelsState(vsidx);
}

bool AttachedBodiesCollisionState::voxelsStateDirty(int vsidx) const
{
    return m_impl->voxelsStateDirty(vsidx);
}

bool AttachedBodiesCollisionState::updateVoxelsStates()
{
    return m_impl->updateVoxelsStates();
}

bool AttachedBodiesCollisionState::updateVoxelsState(int vsidx)
{
    return m_impl->updateVoxelsState(vsidx);
}

const CollisionSpheresState& AttachedBodiesCollisionState::spheresState(
    int ssidx) const
{
    return m_impl->spheresState(ssidx);
}

const CollisionSphereState& AttachedBodiesCollisionState::sphereState(
    const SphereIndex& sidx) const
{
    return m_impl->sphereState(sidx);
}

bool AttachedBodiesCollisionState::sphereStateDirty(
    const SphereIndex& sidx) const
{
    return m_impl->sphereStateDirty(sidx);
}

bool AttachedBodiesCollisionState::updateSphereStates()
{
    return m_impl->updateSphereStates();
}

bool AttachedBodiesCollisionState::updateSphereStates(int ssidx)
{
    return m_impl->updateSphereStates(ssidx);
}

bool AttachedBodiesCollisionState::updateSphereState(const SphereIndex& sidx)
{
    return m_impl->updateSphereState(sidx);
}

const std::vector<int>& AttachedBodiesCollisionState::groupSpheresStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupSpheresStateIndices(group_name);
}

const std::vector<int>& AttachedBodiesCollisionState::groupSpheresStateIndices(
    int gidx) const
{
    return m_impl->groupSpheresStateIndices(gidx);
}

const std::vector<int>&
AttachedBodiesCollisionState::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupOutsideVoxelsStateIndices(group_name);
}

const std::vector<int>&
AttachedBodiesCollisionState::groupOutsideVoxelsStateIndices(int gidx) const
{
    return m_impl->groupOutsideVoxelsStateIndices(gidx);
}

visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization() const
{
    return m_impl->getVisualization();
}

visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization(
    const std::string& group_name) const
{
    return m_impl->getVisualization(group_name);
}

visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization(int gidx) const
{
    return m_impl->getVisualization(gidx);
}

} // namespace collision
} // namespace sbpl
