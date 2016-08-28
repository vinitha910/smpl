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

// project includes
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/debug.h>
#include "voxel_operations.h"

namespace sbpl {
namespace collision {

static const char* RCM_LOGGER = "robot";

class AttachedBodiesCollisionModelImpl
{
public:

    AttachedBodiesCollisionModelImpl(const RobotCollisionModel* model);
    ~AttachedBodiesCollisionModelImpl();

    bool attachBody(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3Vector& transforms,
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

    int version() const;

    size_t sphereModelCount() const;

    bool   hasSpheresModel(const std::string& id) const;
    bool   hasSpheresModel(int abidx) const;
    size_t spheresModelCount() const;
    auto   spheresModel(int smidx) const -> const CollisionSpheresModel&;

    bool   hasVoxelsModel(const std::string& id) const;
    bool   hasVoxelsModel(int abidx) const;
    size_t voxelsModelCount() const;
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    size_t groupCount() const;
    auto   group(int gidx) const -> const CollisionGroupModel&;
    bool   hasGroup(const std::string& group_name) const;
    int    groupIndex(const std::string& group_name) const;
    auto   groupName(int gidx) const -> const std::string&;

    auto groupLinkIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto groupLinkIndices(int gidx) const -> const std::vector<int>&;

private:

    struct AttachedBodyModel
    {
        std::string id;
        int link_index;
    };

    const RobotCollisionModel*                  m_model;

    // Attached Body Information
    hash_map<int, AttachedBodyModel>            m_attached_bodies;
    hash_map<std::string, int>                  m_attached_body_name_to_index;
    int                                         m_attached_bodies_added;

    // cached configuration for regenerating references (necessary still?)
    hash_map<int, CollisionSpheresModelConfig>  m_attached_body_spheres_config;
    hash_map<int, CollisionVoxelModelConfig>    m_attached_body_voxels_config;

    // References to Robot Model
    std::vector<std::vector<int>>               m_link_attached_bodies;

    // Collision Model
    std::vector<CollisionSpheresModel>          m_spheres_models;
    std::vector<CollisionVoxelsModel>           m_voxels_models;
    std::vector<CollisionGroupModel>            m_group_models;
    hash_map<std::string, int>                  m_group_name_to_index;

    // per-attached-body references to corresponding spheres and voxels models
    hash_map<int, const CollisionSpheresModel*> m_attached_body_spheres_models;
    hash_map<int, const CollisionVoxelsModel*>  m_attached_body_voxels_models;

    int m_version;

    int generateAttachedBodyIndex();

    void createSpheresModel(
        int abidx,
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3Vector& transforms);

    void createVoxelsModel(
        int abidx,
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3Vector& transforms);

    void generateSpheresModel(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3Vector& transforms,
        CollisionSpheresModelConfig& spheres_model);

    void generateVoxelsModel(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3Vector& transforms,
        CollisionVoxelModelConfig& voxels_models);

    bool voxelizeAttachedBody(
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3Vector& transforms,
        CollisionVoxelsModel& model) const;
};

AttachedBodiesCollisionModelImpl::AttachedBodiesCollisionModelImpl(
    const RobotCollisionModel* model)
:
    m_model(model),
    m_attached_bodies(),
    m_attached_body_name_to_index(),
    m_attached_bodies_added(),
    m_attached_body_spheres_config(),
    m_attached_body_voxels_config(),
    m_link_attached_bodies(),
    m_spheres_models(),
    m_voxels_models(),
    m_group_models(),
    m_group_name_to_index(),
    m_attached_body_spheres_models(),
    m_attached_body_voxels_models(),
    m_version(0)
{
    m_link_attached_bodies.resize(m_model->linkCount());
}

AttachedBodiesCollisionModelImpl::~AttachedBodiesCollisionModelImpl()
{
}

inline
bool AttachedBodiesCollisionModelImpl::attachBody(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3Vector& transforms,
    const std::string& link_name,
    bool create_voxels_model,
    bool create_spheres_model)
{
    if (hasAttachedBody(id)) {
        ROS_WARN_NAMED(RCM_LOGGER, "Already have attached body '%s'", id.c_str());
        return false;
    }

    if (!m_model->hasLink(link_name)) {
        ROS_WARN_NAMED(RCM_LOGGER, "Link '%s' does not exist in the Robot Collision Model", link_name.c_str());
        return false;
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, "Attaching body '%s'", id.c_str());

    // create the attached body descriptor
    AttachedBodyModel ab;
    ab.id = id;
    ab.link_index = m_model->linkIndex(link_name);
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Link Index: %d", ab.link_index);
    const int abidx = generateAttachedBodyIndex();
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Body Index: %d", abidx);
    m_attached_bodies[abidx] = ab;

    // map the id to the generated index
    m_attached_body_name_to_index[id] = abidx;

    // attach the body to its parent link
    m_link_attached_bodies[ab.link_index].push_back(abidx);

    // create requested models
    if (create_spheres_model) {
        createSpheresModel(abidx, id, shapes, transforms);
    }
    if (create_voxels_model) {
        createVoxelsModel(abidx, id, shapes, transforms);
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating group model");

    // add attached body to all groups whose corresponding robot model groups
    // contain its parent link
    for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
        const CollisionGroupModel& parent_group_model = m_model->group(gidx);
        if (std::find(
                parent_group_model.link_indices.begin(),
                parent_group_model.link_indices.end(),
                ab.link_index) !=
            parent_group_model.link_indices.end())
        {
            m_group_models[gidx].link_indices.push_back(abidx);
        }
    }

    ++m_version;
    return true;
}

inline
bool AttachedBodiesCollisionModelImpl::detachBody(const std::string& id)
{
    if (!hasAttachedBody(id)) {
        ROS_WARN_NAMED(RCM_LOGGER, "No attached body '%s' found in Robot Collision Model", id.c_str());
        return false;
    }

    int abidx = attachedBodyIndex(id);

    // remove from group models
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        CollisionGroupModel& group_model = m_group_models[i];
        auto it = std::remove_if(
                group_model.link_indices.begin(),
                group_model.link_indices.end(),
                [&](int bidx) { return bidx == abidx; });
        group_model.link_indices.erase(it, group_model.link_indices.end());
    }

    // remove the voxels model
    auto vsit = m_attached_body_voxels_models.find(abidx);
    if (vsit != m_attached_body_voxels_models.end()) {
        const CollisionVoxelsModel* voxels_model = vsit->second;

        // remove the per-body voxels model
        m_attached_body_voxels_models.erase(abidx);

        // remove the voxels model itself
        const auto vmidx = std::distance(
                const_cast<const CollisionVoxelsModel*>(m_voxels_models.data()),
                voxels_model);
        m_voxels_models.erase(m_voxels_models.begin() + vmidx);

        // remove the stashed configuration
        m_attached_body_voxels_config.erase(abidx);
    }


    // remove the spheres model
    auto ssit = m_attached_body_spheres_models.find(abidx);
    if (ssit != m_attached_body_spheres_models.end()) {
        const CollisionSpheresModel* spheres_model = ssit->second;

        // remove the per-body spheres model
        m_attached_body_spheres_models.erase(abidx);

        // remove the spheres model itself
        const auto smidx = std::distance(
                const_cast<const CollisionSpheresModel*>(m_spheres_models.data()),
                spheres_model);
        m_spheres_models.erase(m_spheres_models.begin() + smidx);

        // remove the stashed configuration
        m_attached_body_spheres_config.erase(abidx);
    }

    // remove the attached body from its attached link
    std::vector<int>& link_bodies =
            m_link_attached_bodies[m_attached_bodies[abidx].link_index];
    auto it = std::remove_if(link_bodies.begin(), link_bodies.end(),
            [abidx](int idx) { return idx == abidx; });
    link_bodies.erase(it, link_bodies.end());

    int ret;

    // remove the id -> index mapping
    ret = m_attached_body_name_to_index.erase(id);
    assert(ret > 0);

    // remove the attached body
    ret = m_attached_bodies.erase(abidx);
    assert(ret > 0);

    ++m_version;
    return true;
}

inline
size_t AttachedBodiesCollisionModelImpl::attachedBodyCount() const
{
    return m_attached_bodies.size();
}

inline
bool AttachedBodiesCollisionModelImpl::hasAttachedBody(
    const std::string& id) const
{
    return m_attached_body_name_to_index.find(id) !=
            m_attached_body_name_to_index.end();
}

inline
int AttachedBodiesCollisionModelImpl::attachedBodyIndex(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    assert(m_attached_bodies.find(it->second) != m_attached_bodies.end());
    return it->second;
}

inline
const std::string& AttachedBodiesCollisionModelImpl::attachedBodyName(
    int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.id;
}

inline
int AttachedBodiesCollisionModelImpl::attachedBodyLinkIndex(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.link_index;
}

inline
const std::vector<int>& AttachedBodiesCollisionModelImpl::attachedBodyIndices(
    const std::string& link_name) const
{
    const int lidx = m_model->linkIndex(link_name);
    return m_link_attached_bodies[lidx];
}

inline
const std::vector<int>& AttachedBodiesCollisionModelImpl::attachedBodyIndices(
    int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_attached_bodies, lidx);
    return m_link_attached_bodies[lidx];
}

inline
int AttachedBodiesCollisionModelImpl::version() const
{
    return m_version;
}

inline
size_t AttachedBodiesCollisionModelImpl::sphereModelCount() const
{
    size_t total_sphere_count = 0;
    for (const auto& spheres_model : m_spheres_models) {
        total_sphere_count += spheres_model.spheres.size();
    }
    return total_sphere_count;
}

inline
bool AttachedBodiesCollisionModelImpl::hasSpheresModel(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    assert(m_attached_bodies.find(it->second) != m_attached_bodies.end());
    return m_attached_body_spheres_models.find(it->second) !=
            m_attached_body_spheres_models.end();
}

inline
bool AttachedBodiesCollisionModelImpl::hasSpheresModel(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return m_attached_body_spheres_models.find(abidx) !=
            m_attached_body_spheres_models.end();
}

inline
size_t AttachedBodiesCollisionModelImpl::spheresModelCount() const
{
    return m_attached_body_spheres_models.size();
}

inline
const CollisionSpheresModel& AttachedBodiesCollisionModelImpl::spheresModel(
    int smidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_models, smidx);
    return m_spheres_models[smidx];
}

inline
bool AttachedBodiesCollisionModelImpl::hasVoxelsModel(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    auto vmit = m_attached_body_voxels_models.find(it->second);
    return vmit != m_attached_body_voxels_models.end();
}

inline
bool AttachedBodiesCollisionModelImpl::hasVoxelsModel(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    auto vmit = m_attached_body_voxels_models.find(abidx);
    return vmit != m_attached_body_voxels_models.end();
}

inline
size_t AttachedBodiesCollisionModelImpl::voxelsModelCount() const
{
    return m_voxels_models.size();
}

inline
const CollisionVoxelsModel& AttachedBodiesCollisionModelImpl::voxelsModel(
    int vmidx) const
{
    ASSERT_VECTOR_RANGE(m_voxels_models, vmidx);
    return m_voxels_models[vmidx];
}

inline
size_t AttachedBodiesCollisionModelImpl::groupCount() const
{
    return m_group_models.size();
}

inline
const CollisionGroupModel& AttachedBodiesCollisionModelImpl::group(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx];
}

inline
bool AttachedBodiesCollisionModelImpl::hasGroup(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    return it != m_group_name_to_index.end();
}

inline
int AttachedBodiesCollisionModelImpl::groupIndex(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    return it->second;
}

inline
const std::string& AttachedBodiesCollisionModelImpl::groupName(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].name;
}

inline
const std::vector<int>& AttachedBodiesCollisionModelImpl::groupLinkIndices(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    m_group_models[it->second].link_indices;
}

inline
const std::vector<int>& AttachedBodiesCollisionModelImpl::groupLinkIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].link_indices;
}

inline
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

inline
void AttachedBodiesCollisionModelImpl::createSpheresModel(
    int abidx,
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3Vector& transforms)
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Generating spheres model");

    // create configuration spheres and a spheres model for this body
    CollisionSpheresModelConfig spheres_config;
    generateSpheresModel(id, shapes, transforms, spheres_config);

    // stash the config for regenerating references
    m_attached_body_spheres_config[abidx] = spheres_config;

    // append spheres models
    m_spheres_models.resize(m_spheres_models.size() + 1);
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres Model Count %zu -> %zu", m_spheres_models.size() - 1, m_spheres_models.size());
    CollisionSpheresModel* spheres_model = &m_spheres_models.back();

    // attach to the attached body
    spheres_model->link_index = abidx;

    spheres_model->spheres.buildFrom(spheres_config.spheres);
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres Model: %p", spheres_model);

    // map attached body -> collision spheres model
    m_attached_body_spheres_models.clear();
    for (const CollisionSpheresModel& sm : m_spheres_models) {
        m_attached_body_spheres_models[sm.link_index] = &sm;
    }
}

inline
void AttachedBodiesCollisionModelImpl::createVoxelsModel(
    int abidx,
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3Vector& transforms)
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Generating voxels model");
    // create configuration voxels model for this body
    CollisionVoxelModelConfig voxels_config;
    generateVoxelsModel(id, shapes, transforms, voxels_config);

    // stash the config perhaps for future use
    m_attached_body_voxels_config[abidx] = voxels_config;

    // append voxels models
    m_voxels_models.resize(m_voxels_models.size() + 1);
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Voxels Model Count %zu -> %zu", m_voxels_models.size() - 1, m_voxels_models.size());
    CollisionVoxelsModel* voxels_model = &m_voxels_models.back();

    // attach to the attached body
    voxels_model->link_index = abidx;

    const double AB_VOXEL_RES = 0.01;
    voxels_model->voxel_res = AB_VOXEL_RES;

    if (!voxelizeAttachedBody(shapes, transforms, *voxels_model)) {
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize attached body '%s'", id.c_str());
        // TODO: anything to do in this case
    }

    // map attached body -> collision voxels model
    m_attached_body_voxels_models.clear();
    for (const CollisionVoxelsModel& vm : m_voxels_models) {
        m_attached_body_voxels_models[vm.link_index] = &vm;
    }
}

inline
void AttachedBodiesCollisionModelImpl::generateSpheresModel(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3Vector& transforms,
    CollisionSpheresModelConfig& spheres_model)
{
    assert(std::all_of(shapes.begin(), shapes.end(),
            [](const shapes::ShapeConstPtr& shape) { return shape; }));
    assert(shapes.size() == transforms.size());

    // TODO: reserve a special character so as to guarantee sphere uniqueness
    // here and disallow use of the special character on config-generated
    // spheres

    // TODO: yeah...
    const double object_enclosing_sphere_radius = 0.025;

    // voxelize the object
    std::vector<Vector3> voxels;
    for (size_t i = 0; i < shapes.size(); ++i) {
        if (!VoxelizeShape(
                *shapes[i], transforms[i],
                object_enclosing_sphere_radius / std::sqrt(2),
                Eigen::Vector3d::Zero(),
                std::back_inserter(voxels)))
        {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize attached body shape for sphere generation");
            return;
        }
    }

    spheres_model.link_name = id;
    spheres_model.spheres.clear();

    for (size_t i = 0; i < voxels.size(); ++i) {
        CollisionSphereConfig sphere_config;
        sphere_config.name = id + "/" + std::to_string(i);
        sphere_config.x = voxels[i].x();
        sphere_config.y = voxels[i].y();
        sphere_config.z = voxels[i].z();
        sphere_config.radius = object_enclosing_sphere_radius;

        spheres_model.spheres.push_back(std::move(sphere_config));
    }
}

inline
void AttachedBodiesCollisionModelImpl::generateVoxelsModel(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3Vector& transforms,
    CollisionVoxelModelConfig& voxels_model)
{
    voxels_model.link_name = id;
}

inline
bool AttachedBodiesCollisionModelImpl::voxelizeAttachedBody(
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3Vector& transforms,
    CollisionVoxelsModel& model) const
{
    if (shapes.size() != transforms.size()) {
        ROS_ERROR_NAMED(RCM_LOGGER, "shapes array and transforms array must have equal length");
        return false;
    }

    std::vector<Eigen::Vector3d> voxels;
    for (size_t i = 0; i < shapes.size(); ++i) {
        const shapes::Shape& shape = *shapes[i];
        const Eigen::Affine3d& transform = transforms[i];
        VoxelizeShape(
                shape, transform,
                model.voxel_res, Eigen::Vector3d::Zero(), model.voxels);
    }

    return true;
}

/////////////////////////////////////////////////
// AttachedBodiesCollisionModel Implementation //
/////////////////////////////////////////////////

AttachedBodiesCollisionModel::AttachedBodiesCollisionModel(
    const RobotCollisionModel* model)
:
    m_impl(new AttachedBodiesCollisionModelImpl(model))
{
}

AttachedBodiesCollisionModel::~AttachedBodiesCollisionModel()
{
}

/// \brief Attach a body to the collision model
/// \param shapes The shapes making up the body
/// \param transforms The offsets from the attached link for each shape
/// \param link_name The link to attach to
bool AttachedBodiesCollisionModel::attachBody(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3Vector& transforms,
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

int AttachedBodiesCollisionModel::version() const
{
    return m_impl->version();
}

size_t AttachedBodiesCollisionModel::sphereModelCount() const
{
    return m_impl->sphereModelCount();
}

bool AttachedBodiesCollisionModel::hasSpheresModel(const std::string& id) const
{
    return m_impl->hasSpheresModel(id);
}

bool AttachedBodiesCollisionModel::hasSpheresModel(int abidx) const
{
    return m_impl->hasSpheresModel(abidx);
}

size_t AttachedBodiesCollisionModel::spheresModelCount() const
{
    return m_impl->spheresModelCount();
}

const CollisionSpheresModel& AttachedBodiesCollisionModel::spheresModel(
    int smidx) const
{
    return m_impl->spheresModel(smidx);
}

bool AttachedBodiesCollisionModel::hasVoxelsModel(const std::string& id) const
{
    return m_impl->hasVoxelsModel(id);
}

bool AttachedBodiesCollisionModel::hasVoxelsModel(int abidx) const
{
    return m_impl->hasVoxelsModel(abidx);
}

size_t AttachedBodiesCollisionModel::voxelsModelCount() const
{
    return m_impl->voxelsModelCount();
}

const CollisionVoxelsModel& AttachedBodiesCollisionModel::voxelsModel(
    int vmidx) const
{
    return m_impl->voxelsModel(vmidx);
}

size_t AttachedBodiesCollisionModel::groupCount() const
{
    return m_impl->groupCount();
}

const CollisionGroupModel& AttachedBodiesCollisionModel::group(int gidx) const
{
    return m_impl->group(gidx);
}

bool AttachedBodiesCollisionModel::hasGroup(
    const std::string& group_name) const
{
    return m_impl->hasGroup(group_name);
}

int AttachedBodiesCollisionModel::groupIndex(
    const std::string& group_name) const
{
    return m_impl->groupIndex(group_name);
}

const std::string& AttachedBodiesCollisionModel::groupName(int gidx) const
{
    return m_impl->groupName(gidx);
}

const std::vector<int>& AttachedBodiesCollisionModel::groupLinkIndices(
    const std::string& group_name) const
{
    return m_impl->groupLinkIndices(group_name);
}

const std::vector<int>& AttachedBodiesCollisionModel::groupLinkIndices(
    int gidx) const
{
    return m_impl->groupLinkIndices(gidx);
}

} // namespace collision
} // namespace sbpl
