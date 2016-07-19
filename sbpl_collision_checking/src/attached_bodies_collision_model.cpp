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

class AttachedBodiesCollisionModelImpl
{
public:

    AttachedBodiesCollisionModelImpl(RobotCollisionState* state);
    ~AttachedBodiesCollisionModelImpl();

    /// \name Attached Bodies Model
    ///@{

    bool attachBody(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        const std::string& link_name,
        bool create_voxels_model,
        bool create_spheres_model);
    bool detachBody(const std::string& id);

    size_t attachedBodyCount() const;
    bool   hasAttachedBody(const std::string& id) const;
    int    attachedBodyIndex(const std::string& id) const;
    auto   attachedBodyName(int abidx) const -> const std::string&;
    int    attachedBodyLinkIndex(int abidx) const;

    auto attachedBodyIndices(const std::string& link_name) const ->
            const std::vector<int>&;
    auto attachedBodyIndices(int lidx) const -> const std::vector<int>&;

    ///@}

    /// \name Attached Bodies State
    ///@{

    auto attachedBodyTransform(const std::string& id) const ->
            const Eigen::Affine3d&;
    auto attachedBodyTransform(int abidx) const -> const Eigen::Affine3d&;

    bool attachedBodyTransformDirty(const std::string& id) const;
    bool attachedBodyTransformDirty(int abidx) const;

    bool updateAttachedBodyTransform(const std::string& id);
    bool updateAttachedBodyTransform(int abidx);

    auto attachedBodyTransform(const std::string& id) const ->
            const Eigen::Affine3d&;
    auto attachedBodyTransform(int abidx) const -> const Eigen::Affine3d&;

    bool attachedBodyTransformDirty(const std::string& id) const;
    bool attachedBodyTransformDirty(int abidx) const;

    bool updateAttachedBodyTransform(const std::string& id);
    bool updateAttachedBodyTransform(int abidx);

    ///@}

    /// \name Attached Bodies Collision Model
    ///@{
    ///@}

    /// \name Attached Bodies Collision State
    ///@{

    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto sphereState(int ssidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(int ssidx) const;
    bool updateSphereStates();
    bool updateSphereState(int ssidx);

    auto groupSphereStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto groupSphereStateIndices(int gidx) const ->
            const std::vector<int>&;
    auto groupOutsideVoxelsStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;

    ///@}

private:

    RobotCollisionState* m_state;

    struct AttachedBodyModel
    {
        std::string id;
        int link_index;
    };

    std::vector<CollisionSphereModel>   m_sphere_models;
    std::vector<CollisionSpheresModel>  m_spheres_models;
    std::vector<CollisionVoxelsModel>   m_voxels_models;
    std::vector<CollisionGroupModels>   m_group_models;
    hash_map<std::string, int>          m_group_name_to_index;

    std::vector<bool>                   m_dirty_sphere_states;
    std::vector<CollisionSphereState>   m_sphere_states;
    std::vector<CollisionSpheresState>  m_spheres_states;
    std::vector<bool>                   m_dirty_voxels_states;
    std::vector<CollisionVoxelsState>   m_voxels_states;
    std::vector<CollisionGroupState>    m_group_states;

    hash_map<int, CollisionSpheresModelConfig>  m_attached_body_spheres_config;
    hash_map<int, CollisionVoxelModelConfig>    m_attached_body_voxels_config;

    std::vector<std::vector<int>>               m_link_attached_bodies;

    hash_map<int, AttachedBodyModel>            m_attached_bodies;
    hash_map<std::string, int>                  m_attached_body_name_to_index;
    int                                         m_attached_bodies_added;

    // per-attached-body references to corresponding spheres and voxels models
    hash_map<int, const CollisionSpheresModel*> m_attached_body_spheres_models;
    hash_map<int, const CollisionVoxelsModel*>  m_attached_body_voxels_models;

    hash_map<int, CollisionSpheresState*>       m_attached_body_spheres_states;
    hash_map<int, CollisionVoxelsState*>        m_attached_body_voxels_states;

    void bodyAttached(
        int abidx,
        const CollisionSpheresModel* spheres,
        const CollisionVoxelsModel* voxels);
    void bodyDetached(int abidx);

    int generateAttachedBodyIndex();

    void generateSpheresModel(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        const std::string& link_name,
        std::vector<CollisionSphereConfig>& spheres,
        CollisionSpheresModelConfig& spheres_model);

    void generateVoxelsModel(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        const std::string& link_name,
        CollisionVoxelModelConfig& voxels_models);
};

AttachedBodesCollisionModel::AttachedBodiesCollisionModel(
    RobotCollisionState* state)
:
    m_state(state)
{
    m_link_attached_bodies.resize(m_state->model()->linkCount());
}

AttachedBodiesCollisionModel::~AttachedBodiesCollisionModel()
{
}

bool AttachedBodiesCollisionModel::attachBody(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name,
    bool create_voxels_model,
    bool create_spheres_model)
{
    if (hasAttachedBody(id)) {
        ROS_WARN_NAMED(RCM_LOGGER, "Already have attached body '%s'", id.c_str());
        return false;
    }

    if (!m_state->model()->hasLink(link_name)) {
        ROS_WARN_NAMED(RCM_LOGGER, "Link '%s' does not exist in the Robot Collision Model", link_name.c_str());
        return false;
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, "Attaching body '%s'", id.c_str());

    AttachedBodyModel ab;
    ab.id = id;
    ab.link_index = m_state->model()->linkIndex(link_name);
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Link Index: %d", ab.link_index);
    const int abidx = generateAttachedBodyIndex();
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Body Index: %d", abidx);
    m_attached_bodies[abidx] = ab;

    // map the id to the generated index
    m_attached_body_name_to_index[id] = abidx;

    // attach the body to its parent link
    m_link_attached_bodies[ab.link_index].push_back(abidx);

    // generate spheres model
    CollisionSpheresModel* spheres_model = nullptr;
    if (create_spheres_model) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Generating spheres model");
        // create configuration spheres and a spheres model for this body
        std::vector<CollisionSphereConfig> sphere_configs;
        CollisionSpheresModelConfig spheres_model_config;
        generateSpheresModel(
                id, shapes, transforms,
                link_name,
                sphere_configs, spheres_model_config);

        // stash the config for regenerating references
        m_attached_body_spheres_config[abidx] = spheres_model_config;

        // append sphere models
        size_t prev_sphere_count = m_sphere_models.size();
        size_t prev_sphere_capacity = m_sphere_models.capacity();
        m_sphere_models.resize(prev_sphere_count + sphere_configs.size());
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Sphere Count %zu -> %zu", prev_sphere_count, m_sphere_models.size());
        for (size_t i = 0; i < sphere_configs.size(); ++i) {
            const CollisionSphereConfig& sphere_config = sphere_configs[i];
            CollisionSphereModel& sphere_model = m_sphere_models[prev_sphere_count + i];
            sphere_model.name = sphere_config.name;
            sphere_model.center = Eigen::Vector3d(sphere_config.x, sphere_config.y, sphere_config.z);
            sphere_model.radius = sphere_config.radius;
            sphere_model.priority = sphere_config.priority;
        }

        // append spheres models
        m_spheres_models.resize(m_spheres_models.size() + 1);
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres Model Count %zu -> %zu", m_spheres_models.size() - 1, m_spheres_models.size());
        spheres_model = &m_spheres_models.back();
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres Model: %p", spheres_model);

        // attach to the link
        spheres_model->link_index = -1;
        spheres_model->body_index = abidx;

        if (m_sphere_models.size() > prev_sphere_capacity) {
            // TODO: can get away without regenerating all references if current
            // references are still valid
        }

        updateSpheresModelToSphereModelsReferences();
        updateLinkBodyToSpheresModelReferences();
    }
    m_attached_body_spheres_models[abidx] = spheres_model;

    // generate voxels model
    CollisionVoxelsModel* voxels_model = nullptr;
    if (create_voxels_model) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Generating voxels model");
        // create configuration voxels model for this body
        CollisionVoxelModelConfig voxels_model_config;
        generateVoxelsModel(
                id, shapes, transforms,
                link_name,
                voxels_model_config);

        // stash the config perhaps for future use
        m_attached_body_voxels_config[abidx] = voxels_model_config;

        size_t prev_voxels_model_count = m_voxels_models.size();
        m_voxels_models.resize(prev_voxels_model_count + 1);
        ROS_DEBUG_NAMED(RCM_LOGGER, "  Voxels Model Count %zu -> %zu", prev_voxels_model_count, m_voxels_models.size());
        voxels_model = &m_voxels_models.back();

        voxels_model->link_index = -1;
        voxels_model->body_index = abidx;

        const double AB_VOXEL_RES = 0.01;
        voxels_model->voxel_res = AB_VOXEL_RES;
        if (!voxelizeAttachedBody(shapes, transforms, *voxels_model)) {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize attached body '%s'", id.c_str());
            // TODO: anything to do in this case
        }

        updateLinkBodyToVoxelsModelReferences();
    }
    m_attached_body_voxels_models[abidx] = voxels_model;

    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating group model");

    // add attached body to all groups containing its parent link
    for (CollisionGroupModel& group_model : m_group_models) {
        for (int lidx : group_model.link_indices) {
            if (lidx == ab.link_index) {
                group_model.body_indices.push_back(abidx);
            }
        }
    }

    bodyAttached(abidx, spheres_model, voxels_model);
    return true;
}

bool AttachedBodiesCollisionModel::detachBody(const std::string& id)
{
    if (!hasAttachedBody(id)) {
        ROS_WARN_NAMED(RCM_LOGGER, "No attached body '%s' found in Robot Collision Model", id.c_str());
        return false;
    }

    int ret;

    int abidx = attachedBodyIndex(id);

    // defer regeneration of group state references until group models are
    // updated and attached body voxel states are removed

    // get the corresponding voxels model and state, if one exists
    auto vsit = m_attached_body_voxels_models.find(abidx);
    assert(vsit != m_attached_body_voxels_models.end());
    const CollisionVoxelsModel* voxels_model = vsit->second;

    // remove the per-body voxels model and state
    m_attached_body_voxels_models.erase(abidx);

    if (voxels_model) {
        // remove the voxels model
        const auto vmidx = std::distance(
                const_cast<const CollisionVoxelsModel*>(m_voxels_models.data()),
                voxels_model);
        m_voxels_models.erase(m_voxels_models.begin() + vmidx);
    }

    updateLinkBodyToVoxelsModelReferences();

    // remove the stashed configuration
    m_attached_body_voxels_config.erase(abidx);

    // get the corresponding spheres model and state, if one exists
    auto ssit = m_attached_body_spheres_models.find(abidx);
    assert(ssit != m_attached_body_spheres_models.end());
    const CollisionSpheresModel* spheres_model = ssit->second;

    // remove the per-body spheres model and state
    m_attached_body_spheres_models.erase(abidx);

    if (spheres_model) {
        if (!spheres_model->spheres.empty()) {
            // remove the sphere models; NOTE; this makes the same assumption as
            // for removing sphere states with the additional assumption that
            // these sphere models are not shared between multiple spheres
            // models as is the case with attached bodies
            const auto smfirst_idx = std::distance(
                    const_cast<const CollisionSphereModel*>(m_sphere_models.data()),
                    spheres_model->spheres.front());
            const auto smlast_idx = std::distance(
                    const_cast<const CollisionSphereModel*>(m_sphere_models.data()),
                    spheres_model->spheres.back());
            m_sphere_models.erase(
                    m_sphere_models.begin() + smfirst_idx,
                    m_sphere_models.begin() + smlast_idx + 1);
        }

        // remove the spheres model
        const auto smidx = std::distance(
                const_cast<const CollisionSpheresModel*>(m_spheres_models.data()),
                spheres_model);
        m_spheres_models.erase(m_spheres_models.begin() + smidx);
    }

    updateLinkBodyToSpheresModelReferences();
    updateSpheresModelToSphereModelsReferences();

    m_attached_body_spheres_config.erase(abidx);

    // remove from group models
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        CollisionGroupModel& group_model = m_group_models[i];
        std::remove_if(
                group_model.body_indices.begin(),
                group_model.body_indices.end(),
                [&](int bidx) { return bidx == abidx; });
    }

    // remove the attached body from its attached link
    std::vector<int>& attached_bodies =
            m_link_attached_bodies[m_attached_bodies[abidx].link_index];
    auto it = std::remove_if(attached_bodies.begin(), attached_bodies.end(),
            [abidx](int idx) { return idx == abidx; });
    attached_bodies.erase(it, attached_bodies.end());

    // remove the id -> index mapping
    ret = m_attached_body_name_to_index.erase(id);
    assert(ret > 0);

    // remove the attached body
    ret = m_attached_bodies.erase(abidx);

    assert(ret > 0);

    bodyDetached(abidx);
    return true;
}

inline
size_t AttachedBodiesCollisionModel::attachedBodyCount() const
{
    return m_attached_bodies.size();
}

inline
bool AttachedBodiesCollisionModel::hasAttachedBody(const std::string& id) const
{
    return m_attached_body_name_to_index.find(id) !=
            m_attached_body_name_to_index.end();
}

inline
int AttachedBodiesCollisionModel::attachedBodyIndex(const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    assert(m_attached_bodies.find(it->second) != m_attached_bodies.end());
    return it->second;
}

inline
const std::string& AttachedBodiesCollisionModel::attachedBodyName(
    int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.id;
}

inline
int AttachedBodiesCollisionModel::attachedBodyLinkIndex(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.link_index;
}

inline
const std::vector<int>& AttachedBodiesCollisionModel::attachedBodyIndices(
    const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_attached_bodies[lidx];
}

inline
const std::vector<int>& AttachedBodiesCollisionModel::attachedBodyIndices(
    int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_attached_bodies, lidx);
    return m_link_attached_bodies[lidx];
}

inline
const Eigen::Affine3d& AttachedBodiesCollisionModel::attachedBodyTransform(
    const std::string& id) const
{
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_link_transforms[lidx];
}

inline
const Eigen::Affine3d& AttachedBodiesCollisionModel::attachedBodyTransform(
    int abidx) const
{
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_link_transforms[lidx];
}

inline
bool AttachedBodiesCollisionModel::attachedBodyTransformDirty(
    const std::string& id) const
{
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_dirty_link_transforms[lidx];
}

inline
bool AttachedBodiesCollisionModel::attachedBodyTransformDirty(int abidx) const
{
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_dirty_link_transforms[lidx];
}

inline
bool AttachedBodiesCollisionModel::updateAttachedBodyTransform(
    const std::string& id)
{
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return updateLinkTransform(lidx);
}

inline
bool AttachedBodiesCollisionModel::updateAttachedBodyTransform(int abidx)
{
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return updateLinkTransform(lidx);
}

const CollisionVoxelsState& AttachedBodiesCollisionModel::voxelsState(
    int vsidx) const
{
}

bool AttachedBodiesCollisionModel::voxelsStateDirty(int vsidx) const
{
    return false;
}

bool AttachedBodiesCollisionModel::updateVoxelsStates()
{
    return false;
}

bool AttachedBodiesCollisionModel::updateVoxelsState(int vsidx)
{
    return false;
}

const CollisionSphereState& AttachedBodiesCollisionModel::sphereState(
    int ssidx) const
{
}

bool AttachedBodiesCollisionModel::sphereStateDirty(int ssidx) const
{
    return false;
}

bool AttachedBodiesCollisionModel::updateSphereStates()
{
    return false;
}

bool AttachedBodiesCollisionModel::updateSphereState(int ssidx)
{
    return false;
}

const std::vector<int>& AttachedBodiesCollisionModel::groupSphereStateIndices(
    const std::string& group_name) const
{
}

const std::vector<int>& AttachedBodiesCollisionModel::groupSphereStateIndices(
    int gidx) const
{
}

const std::vector<int>& AttachedBodiesCollisionModel::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
}

const std::vector<int>& AttachedBodiesCollisionModel::groupOutsideVoxelsStateIndices(
    int gidx) const
{
}

void AttachedBodiesCollisionModelImpl::bodyAttached(
    int abidx,
    const CollisionSpheresModel* spheres,
    const CollisionVoxelsModel* voxels)
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

void AttachedBodiesCollisionModelImpl::bodyDetached(int abidx)
{
    int ret;

    // defer regeneration of group state references until group models are
    // updated and attached body voxel states are removed

    // get the corresponding voxels model and state, if one exists
    auto vsit = m_attached_body_voxels_states.find(abidx);
    assert(vsit != m_attached_body_voxels_states.end());
    CollisionVoxelsState* voxels_state = vsit->second;

    // remove the per-body voxels model and state
    m_attached_body_voxels_states.erase(abidx);

    if (voxels_state) {
        // remove the voxels state
        const auto vsidx = std::distance(m_voxels_states.data(), voxels_state);
        m_voxels_states.erase(m_voxels_states.begin() + vsidx);
        m_dirty_voxels_states.erase(m_dirty_voxels_states.begin() + vsidx);
    }

    updateLinkBodyToVoxelsStateReferences();
    updateVoxelsStateToModelReferences();

    // get the corresponding spheres model and state, if one exists
    auto ssit = m_attached_body_spheres_states.find(abidx);
    assert(ssit != m_attached_body_spheres_states.end());
    CollisionSpheresState* spheres_state = ssit->second;

    // remove the per-body spheres model and state
    m_attached_body_spheres_states.erase(abidx);

    if (spheres_state) {
        if (!spheres_state->spheres.empty()) {
            // remove the sphere states; NOTE: this takes advantage of the fact
            // that the sphere states for an attached body are added
            // contiguously to the sphere state array; this way we can remove
            // them all in one iteration
            const auto ssfirst_idx = std::distance(
                    m_sphere_states.data(), spheres_state->spheres.front());
            const auto sslast_idx = std::distance(
                    m_sphere_states.data(), spheres_state->spheres.back());
            m_sphere_states.erase(
                    m_sphere_states.begin() + ssfirst_idx,
                    m_sphere_states.begin() + sslast_idx + 1);
            m_dirty_sphere_states.erase(
                    m_dirty_sphere_states.begin() + ssfirst_idx,
                    m_dirty_sphere_states.begin() + sslast_idx + 1);
        }

        // remove the spheres state
        const auto ssidx = std::distance(m_spheres_states.data(), spheres_state);
        m_spheres_states.erase(m_spheres_states.begin() + ssidx);
    }

    updateLinkBodyToSpheresStateReferences();
    updateSpheresStateToSphereStatesReferences();

    updateGroupStateReferences();
}

int AttachedBodiesCollisionModelImpl::generateAttachedBodyIndex()
{
    // TODO: please don't add more than INT_MAX attached bodies over the
    // lifetime of this object
    if (m_attached_bodies_added ==
            std::numeric_limits<decltype(m_attached_bodies_added)>::max())
    {
        ROS_WARN_NAMED(RCM_LOGGER, "Oh god, it's happening");
    }
    return m_attached_bodies_added++;
}

void AttachedBodiesCollisionModelImpl::generateSpheresModel(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name,
    std::vector<CollisionSphereConfig>& spheres,
    CollisionSpheresModelConfig& spheres_model)
{
    assert(std::all_of(shapes.begin(), shapes.end(),
            [](const shapes::ShapeConstPtr& shape) { return shape; }));
    assert(hasLink(link_name));
    assert(shapes.size() == transforms.size());

    // TODO: reserve a special character so as to guarantee sphere uniqueness
    // here and disallow use of the special character on config-generated
    // spheres

    // TODO: yeah...
    const double object_enclosing_sphere_radius = 0.025;

    std::vector<Eigen::Vector3d> voxels;
    for (size_t i = 0; i < shapes.size(); ++i) {
        if (!VoxelizeShape(
                *shapes[i], transforms[i],
                object_enclosing_sphere_radius / std::sqrt(2),
                Eigen::Vector3d::Zero(),
                voxels))
        {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize attached body shape for sphere generation");
            return;
        }
    }

    spheres.clear();
    spheres_model.link_name = link_name;
    spheres_model.spheres.clear();

    for (size_t i = 0; i < voxels.size(); ++i) {
        CollisionSphereConfig sphere_config;
        sphere_config.name = id + "/" + std::to_string(i);
        sphere_config.x = voxels[i].x();
        sphere_config.y = voxels[i].y();
        sphere_config.z = voxels[i].z();
        sphere_config.radius = object_enclosing_sphere_radius;

        spheres_model.spheres.push_back(sphere_config.name);

        spheres.push_back(std::move(sphere_config));
    }
}

void AttachedBodiesCollisionModel::generateVoxelsModel(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name,
    CollisionVoxelModelConfig& voxels_model)
{
    // TODO: see generateSpheresModel
    voxels_model.link_name = link_name;
}

/////////////////////////////////////////////////
// AttachedBodiesCollisionModel Implementation //
/////////////////////////////////////////////////

AttachedBodiesCollisionModel::AttachedBodiesCollisionModel(
    RobotCollisionState* state)
:
    m_impl(new AttachedBodiesCollisionModel(state));
{
}

AttachedBodiesCollisionModel::~AttachedBodiesCollisionModel()
{
}

bool AttachedBodiesCollisionModel::attachBody(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name,
    bool create_voxels_model,
    bool create_spheres_model)
{
    return m_impl->attachBody(
            id, shapes, transforms,
            link_name,
            create_voxels_model, create_spheres_model);
}

bool AttachedBodiesCollisionModel::detachBody(const std::string& id)
{
    return m_impl->detachBody(id);
}

size_t AttachedBodiesCollisionModel::attachedBodyCount() const
{
    return m_impl->attachedBodyCount();
}

bool AttachedBodiesCollisionModel::hasAttachedBody(const std::string& id) const
{
    return m_impl->hasAttachedBody(id);
}

int AttachedBodiesCollisionModel::attachedBodyIndex(const std::string& id) const
{
    return m_impl->attachedBodyIndex(id);
}

const std::string& AttachedBodiesCollisionModel::attachedBodyName(
    int abidx) const
{
    return m_impl->attachedBodyName(abidx);
}

int AttachedBodiesCollisionModel::attachedBodyLinkIndex(int abidx) const
{
    return m_impl->attachedBodyLinkIndex(abidx);
}

const std::vector<int>& AttachedBodiesCollisionModel::attachedBodyIndices(
    const std::string& link_name) const
{
    return m_impl->attachedBodyIndices(link_name);
}

const std::vector<int>& AttachedBodiesCollisionModel::attachedBodyIndices(
    int lidx) const
{
    return m_impl->attachedBodyIndices(lidx);
}

const Eigen::Affine3d& AttachedBodiesCollisionModel::attachedBodyTransform(
    const std::string& id) const
{
    return m_impl->attachedBodyTransform(id);
}

const Eigen::Affine3d& AttachedBodiesCollisionModel::attachedBodyTransform(
    int abidx) const
{
    return m_impl->attachedBodyTransform(abidx);
}

bool AttachedBodiesCollisionModel::attachedBodyTransformDirty(
    const std::string& id) const
{
    return m_impl->attachedBodyTransformDirty(id);
}

bool AttachedBodiesCollisionModel::attachedBodyTransformDirty(int abidx) const
{
    return m_impl->attachedBodyTransformDirty(abidx);
}

bool AttachedBodiesCollisionModel::updateAttachedBodyTransform(
    const std::string& id)
{
    return m_impl->updateAttachedBodyTransform(id);
}

bool AttachedBodiesCollisionModel::updateAttachedBodyTransform(int abidx)
{
    return m_impl->updateAttachedBodyTransform(abidx);
}

const CollisionVoxelsState& AttachedBodiesCollisionModel::voxelsState(
    int vsidx) const
{
    return m_impl->voxelsState(vsidx);
}

bool AttachedBodiesCollisionModel::voxelsStateDirty(int vsidx) const
{
    return m_impl->voxelsStateDirty();
}

bool AttachedBodiesCollisionModel::updateVoxelsStates()
{
    return m_impl->updateVoxelsStates();
}

bool AttachedBodiesCollisionModel::updateVoxelsState(int vsidx)
{
    return m_impl->updateVoxelsState(vsidx);
}

const CollisionSphereState& AttachedBodiesCollisionModel::sphereState(
    int ssidx) const
{
    return m_impl->sphereState(ssidx);
}

bool AttachedBodiesCollisionModel::sphereStateDirty(int ssidx) const
{
    return m_impl->sphereStateDirty(ssidx);
}

bool AttachedBodiesCollisionModel::updateSphereStates()
{
    return m_impl->updateSphereStates();
}

bool AttachedBodiesCollisionModel::updateSphereState(int ssidx)
{
    return m_impl->updateSphereState(ssidx);
}

const std::vector<int>& AttachedBodiesCollisionModel::groupSphereStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupSphereStateIndices(group_name);
}

const std::vector<int>& AttachedBodiesCollisionModel::groupSphereStateIndices(
    int gidx) const
{
    return m_impl->groupSphereStateIndices(gidx);
}

const std::vector<int>& AttachedBodiesCollisionModel::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupOutsideVoxelsStateIndices(group_name);
}

const std::vector<int>& AttachedBodiesCollisionModel::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    return m_impl->groupOutsideVoxelsStateIndices(gidx);
}

} // namespace collision
} // namespace sbpl
