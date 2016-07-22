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
    auto attachedBodyTransform(int lidx) const -> const Eigen::Affine3d&;

    bool attachedBodyTransformDirty(const std::string& id) const;
    bool attachedBodyTransformDirty(int abidx) const;

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

    const AttachedBodiesCollisionModel* m_model;
    RobotCollisionState* m_state;

    std::vector<bool>                   m_dirty_sphere_states;
    std::vector<CollisionSphereState>   m_sphere_states;
    std::vector<CollisionSpheresState>  m_spheres_states;
    std::vector<bool>                   m_dirty_voxels_states;
    std::vector<CollisionVoxelsState>   m_voxels_states;
    std::vector<CollisionGroupState>    m_group_states;

    hash_map<int, CollisionSpheresState*>       m_attached_body_spheres_states;
    hash_map<int, CollisionVoxelsState*>        m_attached_body_voxels_states;
};

AttachedBodiesCollisionState::AttachedBodiesCollisionState(
    const AttachedBodiesCollisionModel* model,
    RobotCollisionState* state)
:
    m_model(model),
    m_state(state)
{
    // initialize sphere(s) states and update all references between sphere(s)
    // states and sphere(s) models
    if (spheres) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "Adding sphere(s) states and updating references");
        // append new sphere states
        int unique_sphere_count = spheres->spheres.size();
        m_sphere_states.resize(m_sphere_states.size() + unique_sphere_count);
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Sphere State Count %zu -> %zu", m_sphere_states.size() - unique_sphere_count, m_sphere_states.size());

        // dirty new sphere states
        ROS_DEBUG_NAMED(RCM_LOGGER, "Dirtying new sphere states");
        m_dirty_sphere_states.resize(m_sphere_states.size(), true);

        // append a new spheres state
        m_spheres_states.resize(m_spheres_states.size() + 1);
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres State Count %zu -> %zu", m_spheres_states.size() - 1, m_spheres_states.size());

        updateSpheresStateToSphereStatesReferences();
        updateLinkBodyToSpheresStateReferences();
    }

    // initialize voxels state and update references to voxels models
    if (voxels) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "Adding voxels state and updating references");
        m_voxels_states.resize(m_voxels_states.size() + 1);
        CollisionVoxelsState& voxels_state = m_voxels_states.back();
        voxels_state.voxels = voxels->voxels;
        m_dirty_voxels_states.resize(m_dirty_voxels_states.size() + 1, true);

        updateVoxelsStateToModelReferences();
        updateLinkBodyToVoxelsStateReferences();
    }

    // update all references from group states to sphere and voxels states
    assert(m_model->groupCount() == m_group_states.size());
    updateGroupStateReferences();
}

AttachedBodiesCollisionState::~AttachedBodiesCollisionState()
{

}

inline
const AttachedBodiesCollisionModel* AttachedBodiesCollisionState::model()
{
    return m_model;
}

inline
RobotCollisionState* AttachedBodiesCollisionState::state()
{
    return m_state;
}

inline
const Eigen::Affine3d& AttachedBodiesCollisionState::attachedBodyTransform(
    const std::string& id) const
{
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_link_transforms[lidx];
}

inline
const Eigen::Affine3d& AttachedBodiesCollisionState::attachedBodyTransform(
    int lidx) const
{
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_link_transforms[lidx];
}

inline
bool AttachedBodiesCollisionState::attachedBodyTransformDirty(
    const std::string& id) const
{
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_dirty_link_transforms[lidx];
}

inline
bool AttachedBodiesCollisionState::attachedBodyTransformDirty(int abidx) const
{
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_dirty_link_transforms[lidx];
}

inline
bool AttachedBodiesCollisionState::updateAttachedBodyTransform(
    const std::string& id)
{
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return updateLinkTransform(lidx);
}

inline
bool AttachedBodiesCollisionState::updateAttachedBodyTransform(int abidx)
{
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return updateLinkTransform(lidx);
}

const CollisionVoxelsState& AttachedBodiesCollisionState::voxelsState(
    int vsidx) const
{

}

bool AttachedBodiesCollisionState::voxelsStateDirty(int vsidx) const
{

}

bool AttachedBodiesCollisionState::updateVoxelsStates()
{

}

bool AttachedBodiesCollisionState::updateVoxelsState(int vsidx)
{
    const int bidx = state.model->body_index;
    updateAttachedBodyTransform(bidx);

    const Eigen::Affine3d& T_model_body = attachedBodyTransform(bidx);

    // transform voxels into the model frame
    new_voxels.resize(state.model->voxels.size());
    for (size_t i = 0; i < state.model->voxels.size(); ++i) {
        new_voxels[i] = T_model_body * state.model->voxels[i];
    }
    return false;
}

const CollisionSpheresState& AttachedBodiesCollisionState::spheresState(
    int ssidx) const
{

}

const CollisionSpheresState& AttachedBodiesCollisionState::sphereState(
    const SphereIndex& sidx) const
{

}

bool AttachedBodiesCollisionState::sphereStateDirty(
    const SphereIndex& sidx) const
{

}

bool AttachedBodiesCollisionState::updateSphereStates()
{

}

bool AttachedBodiesCollisionState::updateSphereStates(int ssidx)
{
    const int bidx = m_sphere_states[sidx].parent_state->model->body_index;
    updateAttachedBodyTransform(bidx);

    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating position of sphere '%s'", m_sphere_states[sidx].model->name.c_str());
    const Eigen::Affine3d& T_model_body = attachedBodyTransform(bidx);
    m_sphere_states[sidx].pos = T_model_body * m_sphere_states[sidx].model->center;
    return false;
}

bool AttachedBodiesCollisionState::updateSphereState(const SphereIndex& sidx)
{

}

const std::vector<int>& AttachedBodiesCollisionState::groupSpheresStateIndices(const std::string& group_name) const
{

}

const std::vector<int>& AttachedBodiesCollisionState::groupSpheresStateIndices(int gidx) const
{

}

const std::vector<int>& AttachedBodiesCollisionState::groupOutsideVoxelsStateIndices(const std::string& group_name) const
{

}

const std::vector<int>& AttachedBodiesCollisionState::groupOutsideVoxelsStateIndices(int gidx) const
{

}

visualization_msgs::MarkerArray AttachedBodiesCollisionState::getVisualization() const
{

}

visualization_msgs::MarkerArray AttachedBodiesCollisionState::getVisualization(const std::string& group_name) const
{

}

visualization_msgs::MarkerArray AttachedBodiesCollisionState::getVisualization(int gidx) const
{

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
    int lidx) const
{
    return m_impl->attachedBodyTransform(lidx);
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
    return m_impl->voxelsStateDirty();
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

const CollisionSpheresState& AttachedBodiesCollisionState::sphereState(
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
    const std::string& group_name) cons
{
    return m_impl->groupOutsideVoxelsStateIndices(group_name);
}

const std::vector<int>&
AttachedBodiesCollisionState::groupOutsideVoxelsStateIndices(int gidx) const
{
    return m_impl->groupOutsideVoxelsStateIndices();
}

visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization() const
{
    return m_impl->getVisualization();
}

visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization(const std::string& group_name) const
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

////////////////////
// FOR PROCESSING //
////////////////////

    bool updateSpheresStateToSphereStatesReferences()
    {
        // assumes you're finished changing the set of sphere models, spheres
        // models, sphere states, and spheres states and updates the following
        // references:
        // * sphere state -> sphere model
        // * sphere state -> spheres state
        // * spheres state -> sphere state[]
        // * spheres state -> spheres model

        for (auto& spheres_state : m_spheres_states) {
            spheres_state.spheres.clear();
        }

        int sphere_state_idx = 0;
        for (size_t i = 0; i < m_model->spheresModelCount(); ++i) {
            // NOTE: careful with the variable shadowing here
            const CollisionSpheresModel& spheres_model = m_model->spheresModel(i);
                CollisionSpheresState& spheres_state = m_spheres_states[i];
            // spheres state -> spheres model
            spheres_state.model = &spheres_model;

            for (const CollisionSphereModel* sphere_model : spheres_model.spheres) {
                // sphere state -> sphere model
                m_sphere_states[sphere_state_idx].model = sphere_model;
                // sphere state -> spheres state
                m_sphere_states[sphere_state_idx].parent_state = &spheres_state;
                // spheres state -> sphere state[]
                spheres_state.spheres.push_back(&m_sphere_states[sphere_state_idx]);
                ++sphere_state_idx;
            }
        }

        return true;
    }

    bool updateVoxelsStateToModelReferences()
    {
        assert(m_model->voxelsModelCount() == m_voxels_states.size());
        for (size_t i = 0; i < m_model->voxelsModelCount(); ++i) {
            const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
                CollisionVoxelsState& voxels_state = m_voxels_states[i];
            voxels_state.model = &voxels_model;
        }
        return true;
    }

    bool updateLinkBodyToSpheresStateReferences()
    {
        // assumes that the links/attached bodies do have a corresponding sphere
        // state but that the pointer reference has been invalidated by an increase
        // in the size of the spheres state array
        assert(m_model->spheresModelCount() == m_spheres_states.size());
        for (size_t i = 0; i < m_spheres_states.size(); ++i) {
            const CollisionSpheresModel& spheres_model = m_model->spheresModel(i);
            CollisionSpheresState* spheres_state = &m_spheres_states[i];
            assert(spheres_model.link_index < m_link_spheres_states.size());
            m_link_spheres_states[spheres_model.link_index] = spheres_state;
        }

        ROS_DEBUG_NAMED(RCM_LOGGER, "Updated per-link and per-body spheres state references");
        return true;
    }

    bool updateLinkBodyToVoxelsStateReferences()
    {
        // assumes that the links/attached bodies do have a corresponding voxels
        // state but that the pointer reference has been invalidated by an increase
        // in the size of the voxels state array
        assert(m_voxels_states.size() == m_model->voxelsModelCount());
        for (size_t i = 0; i < m_voxels_states.size(); ++i) {
            const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
            CollisionVoxelsState* voxels_state = &m_voxels_states[i];
            m_link_voxels_states[voxels_model.link_index] = voxels_state;
        }
        return true;
    }

    void updateLinkBodyToSpheresStateReferences()
    {
        m_attached_body_spheres_states[spheres_model.body_index] = spheres_state;
    }
    void updateLinkBodyToVoxelsStateReferences()
    {
        m_attached_body_voxels_states[voxels_model.body_index] = voxels_state;
    }
    void updateGroupStateReferences()
    {
        for (int bidx : model.body_indices) {
            CollisionSpheresState* spheres_state = m_attached_body_spheres_states[bidx];
            if (spheres_state) {
                for (CollisionSphereState* sphere_state : spheres_state->spheres) {
                    int ssidx = std::distance(m_sphere_states.data(), sphere_state);
                    state.sphere_indices.push_back(ssidx);
                }
            }
        }

        // gather the indices of all attached body voxels states that do NOT
        // belong to this group
        for (const auto& entry : m_attached_body_voxels_states) {
            if (std::find(
                    model.body_indices.begin(),
                    model.body_indices.end(),
                    entry.first) ==
                model.body_indices.end())
            {
                CollisionVoxelsState* voxels_state = entry.second;
                if (voxels_state) {
                    int vsidx = std::distance(m_voxels_states.data(), voxels_state);
                    state.voxels_indices.push_back(vsidx);
                }
            }
        }
    }
