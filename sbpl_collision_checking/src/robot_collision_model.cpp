////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <sbpl_collision_checking/robot_collision_model.h>

// standard includes
#include <assert.h>
#include <stdlib.h>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <leatherman/viz.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <urdf/model.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include "debug.h"
#include "transform_functions.h"
#include "voxel_operations.h"

namespace sbpl {
namespace collision {

static const char* RCM_LOGGER = "robot";

/////////////////////////////////////////
// RobotCollisionModelImpl Declaration //
/////////////////////////////////////////

class RobotCollisionModelImpl
{
public:

    RobotCollisionModelImpl();
    ~RobotCollisionModelImpl();

    bool init(
        const urdf::ModelInterface& urdf,
        const CollisionModelConfig& config);

    auto   name() const -> const std::string&;
    auto   modelFrame() const -> const std::string&;

    size_t jointVarCount() const;
    auto   jointVarNames() const -> const std::vector<std::string>&;

    bool   hasJointVar(const std::string& joint_name) const;
    int    jointVarIndex(const std::string& joint_name) const;
    auto   jointVarName(int jidx) const -> const std::string&;

    bool   jointVarIsContinuous(const std::string& joint_name) const;
    bool   jointVarHasPositionBounds(const std::string& joint_name) const;
    double jointVarMaxPosition(const std::string& joint_name) const;
    double jointVarMinPosition(const std::string& joint_name) const;

    bool   jointVarIsContinuous(int jidx) const;
    bool   jointVarHasPositionBounds(int jidx) const;
    double jointVarMinPosition(int jidx) const;
    double jointVarMaxPosition(int jidx) const;

    size_t jointCount() const;

    int    jointParentLinkIndex(int jidx) const;
    int    jointChildLinkIndex(int jidx) const;

    auto   jointOrigin(int jidx) const -> const Eigen::Affine3d&;
    auto   jointAxis(int jidx) const -> const Eigen::Vector3d&;
    auto   jointTransformFn(int jidx) const -> JointTransformFunction;

    size_t linkCount() const;
    auto   linkNames() const -> const std::vector<std::string>&;
    bool   hasLink(const std::string& link_name) const;
    int    linkIndex(const std::string& link_name) const;
    auto   linkName(int lidx) const -> const std::string&;

    int    linkParentJointIndex(int lidx) const;
    auto   linkChildJointIndices(int lidx) const ->
            const std::vector<int>&;

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

    size_t sphereModelCount() const;
    auto   sphereModel(int smidx) const -> const CollisionSphereModel&;

    bool   hasSpheresModel(const std::string& link_name) const;
    bool   hasSpheresModel(int lidx) const;
    size_t spheresModelCount() const;
    auto   spheresModel(int smidx) const -> const CollisionSpheresModel&;

    bool   hasVoxelsModel(const std::string& link_name) const;
    bool   hasVoxelsModel(int lidx) const;
    size_t voxelsModelCount() const;
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    size_t groupCount() const;
    auto   group(int gidx) const -> const CollisionGroupModel&;
    bool   hasGroup(const std::string& group_name) const;
    int    groupIndex(const std::string& group_name) const;
    auto   groupName(int gidx) const -> const std::string&;
    auto   groupLinkIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupLinkIndices(int gidx) const ->
            const std::vector<int>&;

    bool registerRobotCollisionState(RobotCollisionState* state);
    bool unregisterRobotCollisionState(RobotCollisionState* state);

private:

    struct AttachedBodyModel
    {
        std::string id;
        int link_index;
    };

    // cached after initialization to allow regeneration of references
    CollisionModelConfig m_config;
    hash_map<int, CollisionSpheresModelConfig> m_attached_body_spheres_config;
    hash_map<int, CollisionVoxelModelConfig>   m_attached_body_voxels_config;

    /// \name Robot Model
    ///@{
    std::string                             m_name;
    std::string                             m_model_frame;
    std::vector<std::string>                m_jvar_names;
    std::vector<bool>                       m_jvar_continuous;
    std::vector<bool>                       m_jvar_has_position_bounds;
    std::vector<double>                     m_jvar_min_positions;
    std::vector<double>                     m_jvar_max_positions;
    hash_map<std::string, int>              m_jvar_name_to_index;

    Affine3dVector                          m_joint_origins;
    std::vector<Eigen::Vector3d>            m_joint_axes;
    std::vector<JointTransformFunction>     m_joint_transforms;
    std::vector<int>                        m_joint_parent_links;
    std::vector<int>                        m_joint_child_links;

    std::vector<std::string>                m_link_names;
    std::vector<int>                        m_link_parent_joints;
    std::vector<std::vector<int>>           m_link_children_joints;
    std::vector<std::vector<int>>           m_link_attached_bodies;
    hash_map<std::string, int>              m_link_name_to_index;

    hash_map<int, AttachedBodyModel>        m_attached_bodies;
    hash_map<std::string, int>              m_attached_body_name_to_index;
    int                                     m_attached_bodies_added;
    ///@}

    /// \name Collision Model
    ///@{

    // one entry per element specific through config that may be shared between
    // multiple collision sphere states
    std::vector<CollisionSphereModel>   m_sphere_models;

    // one entry for each link that has a spheres model
    std::vector<CollisionSpheresModel>  m_spheres_models;

    // one entry for each link that has a voxels model
    std::vector<CollisionVoxelsModel>   m_voxels_models;

    // one entry for each group specified through config
    std::vector<CollisionGroupModel>    m_group_models;
    hash_map<std::string, int>          m_group_name_to_index;

    // per-link references to corresponding spheres and voxels models
    std::vector<const CollisionSpheresModel*> m_link_spheres_models;
    std::vector<const CollisionVoxelsModel*>  m_link_voxels_models;

    // per-attached-body references to corresponding spheres and voxels models
    hash_map<int, const CollisionSpheresModel*> m_attached_body_spheres_models;
    hash_map<int, const CollisionVoxelsModel*>  m_attached_body_voxels_models;

    ///@}

    std::vector<RobotCollisionState*> m_collision_states;

    bool initRobotModel(const urdf::ModelInterface& urdf);
    bool initCollisionModel(
        const urdf::ModelInterface& urdf,
        const CollisionModelConfig& config);

    bool checkCollisionModelReferences() const;

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

    bool updateSpheresModelToSphereModelsReferences();
    bool updateLinkBodyToSpheresModelReferences();
    bool updateLinkBodyToVoxelsModelReferences();

    Eigen::Affine3d poseUrdfToEigen(const urdf::Pose& p) const;

    bool voxelizeLink(
        const urdf::ModelInterface& urdf,
        const std::string& link_name,
        CollisionVoxelsModel& model) const;

    bool voxelizeCollisionElement(
        const urdf::Collision& collision,
        double res,
        std::vector<Eigen::Vector3d>& voxels) const;

    bool voxelizeGeometry(
        const urdf::Geometry& geom,
        const Eigen::Affine3d& pose,
        double res,
        std::vector<Eigen::Vector3d>& voxels) const;

    bool voxelizeAttachedBody(
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        CollisionVoxelsModel& model) const;
};

// TODO:
//    bool hasSpheresModelOverride(const std::string& link_name, const std::string& group_name) const;
//    bool hasSpheresModelOverride(const std::string& link_name, int gidx) const;
//    bool hasSpheresModelOverride(int lidx, const std::string& group_name) const;
//    bool hasSpheresModelOverride(int lidx, int gidx) const;

////////////////////////////////////////
// RobotCollisionModelImpl Definition //
////////////////////////////////////////

RobotCollisionModelImpl::RobotCollisionModelImpl() :
    m_name(),
    m_model_frame(),
    m_jvar_names(),
    m_jvar_continuous(),
    m_jvar_has_position_bounds(),
    m_jvar_min_positions(),
    m_jvar_max_positions(),
    m_jvar_name_to_index(),
    m_joint_origins(),
    m_joint_axes(),
    m_joint_transforms(),
    m_link_names(),
    m_link_name_to_index(),
    m_attached_bodies(),
    m_attached_body_name_to_index(),
    m_attached_bodies_added(0),
    m_sphere_models(),
    m_spheres_models(),
    m_voxels_models(),
    m_group_models(),
    m_group_name_to_index(),
    m_link_spheres_models(),
    m_link_voxels_models()
{
}

RobotCollisionModelImpl::~RobotCollisionModelImpl()
{
}

bool RobotCollisionModelImpl::init(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    bool success = true;
    success = success && initRobotModel(urdf);
    success = success && initCollisionModel(urdf, config);

    if (success) {
        m_config = config;
    }

    return success;
}

inline
const std::string& RobotCollisionModelImpl::name() const
{
    return m_name;
}

inline
const std::string& RobotCollisionModelImpl::modelFrame() const
{
    return m_model_frame;
}

inline
size_t RobotCollisionModelImpl::jointVarCount() const
{
    return m_jvar_names.size();
}

inline
const std::vector<std::string>& RobotCollisionModelImpl::jointVarNames() const
{
    return m_jvar_names;
}

inline
bool RobotCollisionModelImpl::hasJointVar(const std::string& joint_name) const
{
    return m_jvar_name_to_index.find(joint_name) != m_jvar_name_to_index.end();
}

inline
int RobotCollisionModelImpl::jointVarIndex(const std::string& joint_name) const
{
    auto it = m_jvar_name_to_index.find(joint_name);
    ASSERT_RANGE(it != m_jvar_name_to_index.end());
    assert(it->second >= 0 && it->second < m_jvar_names.size());
    return it->second;
}

inline
const std::string& RobotCollisionModelImpl::jointVarName(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_names, jidx);
    return m_jvar_names[jidx];
}

inline
bool RobotCollisionModelImpl::jointVarIsContinuous(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_continuous[jidx];
}

inline
bool RobotCollisionModelImpl::jointVarHasPositionBounds(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_has_position_bounds[jidx];
}

inline
double RobotCollisionModelImpl::jointVarMinPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_min_positions[jidx];
}

inline
double RobotCollisionModelImpl::jointVarMaxPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_max_positions[jidx];
}

inline
bool RobotCollisionModelImpl::jointVarIsContinuous(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_continuous, jidx);
    return m_jvar_continuous[jidx];
}

inline
bool RobotCollisionModelImpl::jointVarHasPositionBounds(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_has_position_bounds, jidx);
    return m_jvar_has_position_bounds[jidx];
}

inline
double RobotCollisionModelImpl::jointVarMinPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_min_positions, jidx);
    return m_jvar_min_positions[jidx];
}

inline
double RobotCollisionModelImpl::jointVarMaxPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_max_positions, jidx);
    return m_jvar_max_positions[jidx];
}

size_t RobotCollisionModelImpl::jointCount() const
{
    return m_joint_transforms.size();
}

int RobotCollisionModelImpl::jointParentLinkIndex(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_parent_links, jidx);
    return m_joint_parent_links[jidx];
}

int RobotCollisionModelImpl::jointChildLinkIndex(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_child_links, jidx);
    return m_joint_child_links[jidx];
}

inline
const Eigen::Affine3d& RobotCollisionModelImpl::jointOrigin(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_origins, jidx);
    return m_joint_origins[jidx];
}

inline
const Eigen::Vector3d& RobotCollisionModelImpl::jointAxis(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_axes, jidx);
    return m_joint_axes[jidx];
}

inline
JointTransformFunction RobotCollisionModelImpl::jointTransformFn(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_transforms, jidx);
    return m_joint_transforms[jidx];
}

inline
size_t RobotCollisionModelImpl::linkCount() const
{
    return m_link_names.size();
}

inline
const std::vector<std::string>& RobotCollisionModelImpl::linkNames() const
{
    return m_link_names;
}

inline
bool RobotCollisionModelImpl::hasLink(const std::string& link_name) const
{
    return m_link_name_to_index.find(link_name) != m_link_name_to_index.end();
}

inline
int RobotCollisionModelImpl::linkIndex(const std::string& link_name) const
{
    auto it = m_link_name_to_index.find(link_name);
    ASSERT_RANGE(it != m_link_name_to_index.end());
    assert(it->second >= 0 && it->second < m_link_names.size());
    return it->second;
}

inline
const std::string& RobotCollisionModelImpl::linkName(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_names, lidx);
    return m_link_names[lidx];
}

int RobotCollisionModelImpl::linkParentJointIndex(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_parent_joints, lidx);
    return m_link_parent_joints[lidx];
}

const std::vector<int>& RobotCollisionModelImpl::linkChildJointIndices(
    int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_children_joints, lidx);
    return m_link_children_joints[lidx];
}

bool RobotCollisionModelImpl::attachBody(
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

    if (!hasLink(link_name)) {
        ROS_WARN_NAMED(RCM_LOGGER, "Link '%s' does not exist in the Robot Collision Model", link_name.c_str());
        return false;
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, "Attaching body '%s'", id.c_str());

    AttachedBodyModel ab;
    ab.id = id;
    ab.link_index = linkIndex(link_name);
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

    for (RobotCollisionState* state : m_collision_states) {
        state->bodyAttached(abidx, spheres_model, voxels_model);
    }

    return true;
}

bool RobotCollisionModelImpl::detachBody(const std::string& id)
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

    for (RobotCollisionState* state : m_collision_states) {
        state->bodyDetached(abidx);
    }

    return true;
}

size_t RobotCollisionModelImpl::attachedBodyCount() const
{
    return m_attached_bodies.size();
}

bool RobotCollisionModelImpl::hasAttachedBody(const std::string& id) const
{
    return m_attached_body_name_to_index.find(id) !=
            m_attached_body_name_to_index.end();
}

int RobotCollisionModelImpl::attachedBodyIndex(const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    assert(m_attached_bodies.find(it->second) != m_attached_bodies.end());
    return it->second;
}

const std::string& RobotCollisionModelImpl::attachedBodyName(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.id;
}

int RobotCollisionModelImpl::attachedBodyLinkIndex(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    return it->second.link_index;
}

const std::vector<int>& RobotCollisionModelImpl::attachedBodyIndices(
    const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_attached_bodies[lidx];
}

const std::vector<int>& RobotCollisionModelImpl::attachedBodyIndices(
    int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_attached_bodies, lidx);
    return m_link_attached_bodies[lidx];
}

inline
size_t RobotCollisionModelImpl::sphereModelCount() const
{
    return m_sphere_models.size();
}

inline
const CollisionSphereModel& RobotCollisionModelImpl::sphereModel(int smidx) const
{
    ASSERT_VECTOR_RANGE(m_sphere_models, smidx);
    return m_sphere_models[smidx];
}

inline
bool RobotCollisionModelImpl::hasSpheresModel(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_spheres_models[lidx] != nullptr;
}

inline
bool RobotCollisionModelImpl::hasSpheresModel(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_spheres_models, lidx);
    return m_link_spheres_models[lidx] != nullptr;
}

inline
size_t RobotCollisionModelImpl::spheresModelCount() const
{
    return m_spheres_models.size();
}

const CollisionSpheresModel& RobotCollisionModelImpl::spheresModel(
    int smidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_models, smidx);
    return m_spheres_models[smidx];
}

inline
bool RobotCollisionModelImpl::hasVoxelsModel(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_voxels_models[lidx] != nullptr;
}

inline
bool RobotCollisionModelImpl::hasVoxelsModel(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_voxels_models, lidx);
    return m_link_voxels_models[lidx];
}

inline
size_t RobotCollisionModelImpl::voxelsModelCount() const
{
    return m_voxels_models.size();
}

inline
const CollisionVoxelsModel& RobotCollisionModelImpl::voxelsModel(int vmidx) const
{
    ASSERT_RANGE(vmidx >= 0 && vmidx < m_voxels_models.size());
    return m_voxels_models[vmidx];
}

inline
size_t RobotCollisionModelImpl::groupCount() const
{
    return m_group_models.size();
}

inline
const CollisionGroupModel& RobotCollisionModelImpl::group(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx];
}

inline
bool RobotCollisionModelImpl::hasGroup(const std::string& group_name) const
{
    return m_group_name_to_index.find(group_name) !=
            m_group_name_to_index.end();
}

inline
int RobotCollisionModelImpl::groupIndex(const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    assert(it->second >= 0 && it->second < m_group_models.size());
    return it->second;
}

inline
const std::string& RobotCollisionModelImpl::groupName(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].name;
}

inline
const std::vector<int>& RobotCollisionModelImpl::groupLinkIndices(
    const std::string& group_name) const
{
    const int gidx = groupIndex(group_name);
    return m_group_models[gidx].link_indices;
}

inline
const std::vector<int>& RobotCollisionModelImpl::groupLinkIndices(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].link_indices;
}

inline
bool RobotCollisionModelImpl::registerRobotCollisionState(
    RobotCollisionState* state)
{
    auto it = std::find(
            m_collision_states.begin(), m_collision_states.end(), state);
    if (it != m_collision_states.end()) {
        return false;
    }

    m_collision_states.push_back(state);
    return true;
}

inline
bool RobotCollisionModelImpl::unregisterRobotCollisionState(
    RobotCollisionState* state)
{
    auto it = std::find(
            m_collision_states.begin(), m_collision_states.end(), state);
    if (it == m_collision_states.end()) {
        return false;
    }
    m_collision_states.erase(it);
    return true;
}

bool RobotCollisionModelImpl::initRobotModel(const urdf::ModelInterface& urdf)
{
    m_name = urdf.getName();

    auto root_link = urdf.getRoot();
    m_model_frame = root_link->name;

    // TODO: depth-first or post-traversal reordering to keep dependent
    // joints/links next to one another

    m_joint_transforms.push_back(ComputeFixedJointTransform);
    m_joint_origins.push_back(Eigen::Affine3d::Identity());
    m_joint_axes.push_back(Eigen::Vector3d::Zero());
    m_joint_parent_links.push_back(-1);

    // breadth-first traversal of all links in the robot
    typedef std::pair<boost::shared_ptr<const urdf::Link>, int>
    link_parent_joint_idx_pair;

    std::queue<link_parent_joint_idx_pair> links;
    links.push(std::make_pair(root_link, 0));

    std::vector<std::string> joint_names;
    while (!links.empty()) {
        boost::shared_ptr<const urdf::Link> link;
        int parent_joint_idx;
        std::tie(link, parent_joint_idx) = links.front();

        links.pop();

        m_link_names.push_back(link->name);
        m_link_parent_joints.push_back(parent_joint_idx);

        const size_t lidx = m_link_names.size() - 1;
        m_link_name_to_index[m_link_names.back()] = lidx;

        m_link_children_joints.push_back(std::vector<int>());
        m_link_attached_bodies.push_back(std::vector<int>());

        // for each joint
        for (const auto& joint : link->child_joints) {
            const std::string& joint_name = joint->name;

            joint_names.push_back(joint_name);

            auto limits = joint->limits;

            double min_position_limit = std::numeric_limits<double>::lowest();
            double max_position_limit = std::numeric_limits<double>::max();
            bool has_position_limit = false;
            bool continuous = (joint->type == urdf::Joint::CONTINUOUS);

            urdf::Pose origin = joint->parent_to_joint_origin_transform;
            urdf::Vector3 axis = joint->axis;

            if (limits) {
                has_position_limit = true;
                min_position_limit = limits->lower;
                max_position_limit = limits->upper;
            }

            switch (joint->type) {
            case urdf::Joint::FIXED:
            {
                m_joint_transforms.push_back(ComputeFixedJointTransform);
            }   break;
            case urdf::Joint::REVOLUTE:
            {
                m_jvar_names.push_back(joint_name);
                m_jvar_continuous.push_back(continuous);
                m_jvar_has_position_bounds.push_back(has_position_limit);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);

                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeRevoluteJointTransform);
            }   break;
            case urdf::Joint::PRISMATIC:
            {
                m_jvar_names.push_back(joint_name);
                m_jvar_continuous.push_back(continuous);
                m_jvar_has_position_bounds.push_back(has_position_limit);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);

                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputePrismaticJointTransform);
            }   break;
            case urdf::Joint::CONTINUOUS:
            {
                m_jvar_names.push_back(joint_name);
                m_jvar_continuous.push_back(continuous);
                m_jvar_has_position_bounds.push_back(has_position_limit);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);

                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeContinuousJointTransform);
            }   break;
            case urdf::Joint::PLANAR:
            {
                // NOTE: local joint variable names follow moveit conventions
                std::string var_name;

                var_name = joint_name + "/x";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
                m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/y";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
                m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/theta";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
                m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputePlanarJointTransform);
            }   break;
            case urdf::Joint::FLOATING:
            {
                // NOTE: local joint variable names follow moveit conventions
                std::string var_name;

                var_name = joint_name + "/trans_x";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
                m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/trans_y";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
                m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/trans_z";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
                m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/rot_x";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(true);
                m_jvar_min_positions.push_back(-1.0);
                m_jvar_max_positions.push_back(1.0);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/rot_y";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(true);
                m_jvar_min_positions.push_back(-1.0);
                m_jvar_max_positions.push_back(1.0);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/rot_z";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(true);
                m_jvar_min_positions.push_back(-1.0);
                m_jvar_max_positions.push_back(1.0);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/rot_w";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(true);
                m_jvar_min_positions.push_back(-1.0);
                m_jvar_max_positions.push_back(1.0);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeFloatingJointTransform);
            }   break;
            default:
            {
                ROS_ERROR_NAMED(RCM_LOGGER, "Unknown joint type encountered");
                return false;
            }   break;
            }

            m_link_children_joints[lidx].push_back(m_joint_origins.size());

            m_joint_origins.push_back(poseUrdfToEigen(origin));
            m_joint_axes.push_back(Eigen::Vector3d(axis.x, axis.y, axis.z));

            m_joint_parent_links.push_back(lidx);

            // NOTE: can't push child link indices here since we don't yet know
            // what those indices will be, at least not without some effort

            // push the child link onto the queue
            auto child_link = urdf.getLink(joint->child_link_name);
            links.push(std::make_pair(child_link, m_joint_axes.size() - 1));
        }
    }

    // map joint -> child link
    m_joint_child_links.resize(m_joint_transforms.size());
    for (size_t lidx = 0; lidx < m_link_parent_joints.size(); ++lidx) {
        int pjidx = m_link_parent_joints[lidx];
        m_joint_child_links[pjidx] = lidx;
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeFixedJointTransform: %p", ComputeFixedJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeRevoluteJointTransform: %p", ComputeRevoluteJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeContinuousJointTransform: %p", ComputeContinuousJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputePrismaticJointTransform: %p", ComputePrismaticJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputePlanarJointTransform: %p", ComputePlanarJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeFloatingJointTransform: %p", ComputeFloatingJointTransform);

    ROS_DEBUG_NAMED(RCM_LOGGER, "Robot Model:");
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Name: %s", m_name.c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Model Frame: %s", m_model_frame.c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Variable Names: %s", to_string(m_jvar_names).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Variable Continuous: %s", to_string(m_jvar_continuous).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Variable Has Position Bounds: %s", to_string(m_jvar_has_position_bounds).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Variable Min Positions: %s", to_string(m_jvar_min_positions).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Variable Max Positions: %s", to_string(m_jvar_max_positions).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Transform Functions:");
    for (auto p : m_joint_transforms) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    %p", p);
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Origins:");
    for (const auto& o : m_joint_origins) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    %s", AffineToString(o).c_str());
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Parent Links: %s", to_string(m_joint_parent_links).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Child Links: %s", to_string(m_joint_child_links).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Link Names: %s", to_string(m_link_names).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Link Parent Joints: %s", to_string(m_link_parent_joints).c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Link Child Joints: %s", to_string(m_link_children_joints).c_str());

    return true;
}

bool RobotCollisionModelImpl::initCollisionModel(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    // initialize sphere models
    m_sphere_models.resize(config.spheres.size());
    for (size_t i = 0; i < m_sphere_models.size(); ++i) {
        const CollisionSphereConfig& sphere_config = config.spheres[i];
        CollisionSphereModel& sphere_model = m_sphere_models[i];
        sphere_model.name = sphere_config.name;
        sphere_model.center = Eigen::Vector3d(sphere_config.x, sphere_config.y, sphere_config.z);
        sphere_model.radius = sphere_config.radius;
        sphere_model.priority = sphere_config.priority;
    }

    // initialize spheres models
    m_spheres_models.resize(config.spheres_models.size());
    for (size_t i = 0; i < m_spheres_models.size(); ++i) {
        const CollisionSpheresModelConfig& spheres_config = config.spheres_models[i];
        CollisionSpheresModel& spheres_model = m_spheres_models[i];

        // attach to the link
        spheres_model.link_index = linkIndex(spheres_config.link_name);
        spheres_model.body_index = -1;

        // add references to all spheres; NOTE: cannot use
        // updateSpheresModelToSphereModelsReferences(at this point
        for (const std::string& sphere_name : spheres_config.spheres) {
            // find the sphere model with this name
            auto sit = std::find_if(m_sphere_models.begin(), m_sphere_models.end(),
                    [&sphere_name](const CollisionSphereModel& sphere)
                    {
                        return sphere.name == sphere_name;
                    });
            if (sit == m_sphere_models.end()) {
                ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find sphere '%s' specified in spheres model for link '%s'", sphere_name.c_str(), spheres_config.link_name.c_str());
                return false;
            }
            const CollisionSphereModel* sphere_model =
                    m_sphere_models.data() +
                    std::distance(m_sphere_models.begin(), sit);
            spheres_model.spheres.push_back(sphere_model);
        }
    }

    // initialize voxels models
    m_voxels_models.resize(config.voxel_models.size());
    for (size_t i = 0; i < m_voxels_models.size(); ++i) {
        CollisionVoxelsModel& voxels_model = m_voxels_models[i];
        const std::string& link_name = config.voxel_models[i].link_name;
        voxels_model.link_index = linkIndex(link_name);
        voxels_model.body_index = -1;
        const double LINK_VOXEL_RESOLUTION = 0.01;
        voxels_model.voxel_res = LINK_VOXEL_RESOLUTION;
        if (!voxelizeLink(urdf, link_name, voxels_model)) {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize link '%s'", link_name.c_str());
        }
    }

    // initialize groups
    m_group_models.resize(config.groups.size());
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        CollisionGroupModel& group_model = m_group_models[i];
        const CollisionGroupConfig& group_config = config.groups[i];
        const std::string& group_name = group_config.name;
        group_model.name = group_name;

        for (size_t j = 0; j < group_config.links.size(); ++j) {
            const std::string& link_name = group_config.links[j];
            group_model.link_indices.push_back(linkIndex(link_name));
        }

        m_group_name_to_index[group_name] = i;
    }

    // initialize link spheres models
    m_link_spheres_models.assign(m_link_names.size(), nullptr);
    for (const CollisionSpheresModel& spheres_model : m_spheres_models) {
        m_link_spheres_models[spheres_model.link_index] = &spheres_model;
    }

    // initialize link voxels models
    m_link_voxels_models.assign(m_link_names.size(), nullptr);
    for (const CollisionVoxelsModel& voxels_model : m_voxels_models) {
        m_link_voxels_models[voxels_model.link_index] = &voxels_model;
    }

    assert(checkCollisionModelReferences());

    ROS_DEBUG_NAMED(RCM_LOGGER, "Collision Model:");
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Sphere Models: [%p, %p]", m_sphere_models.data(), m_sphere_models.data() + m_sphere_models.size());
    for (const auto& sphere_model : m_sphere_models) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    name: %s, center: (%0.3f, %0.3f, %0.3f), radius: %0.3f, priority: %d", sphere_model.name.c_str(), sphere_model.center.x(), sphere_model.center.y(), sphere_model.center.z(), sphere_model.radius, sphere_model.priority);
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres Models: [%p, %p]", m_spheres_models.data(), m_spheres_models.data() + m_spheres_models.size());
    for (const auto& spheres_model : m_spheres_models) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    link_index: %d, spheres: %s", spheres_model.link_index, to_string(spheres_model.spheres).c_str());
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Voxels Models: [%p, %p]", m_voxels_models.data(), m_voxels_models.data() + m_voxels_models.size());
    for (const auto& voxels_model : m_voxels_models) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    link_index: %d, voxel_res: %0.3f, voxel count: %zu", voxels_model.link_index, voxels_model.voxel_res, voxels_model.voxels.size());
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Group Models:");
    for (const auto& group_model : m_group_models) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    name: %s, link_indices: %s", group_model.name.c_str(), to_string(group_model.link_indices).c_str());
    }

    return true;
}

bool RobotCollisionModelImpl::checkCollisionModelReferences() const
{
    for (const auto& spheres_model : m_spheres_models) {
        if (spheres_model.link_index != -1 &&
            (
                spheres_model.link_index < 0 ||
                spheres_model.link_index >= m_link_names.size()
            ))
        {
            return false;
        }

        for (auto sphere : spheres_model.spheres) {
            if (!(sphere >= m_sphere_models.data() &&
                    sphere < m_sphere_models.data() + m_sphere_models.size()))
            {
                return false;
            }
        }
    }

    for (const auto& voxels_model : m_voxels_models) {
        if (voxels_model.link_index != -1 &&
            (
                voxels_model.link_index < 0 ||
                voxels_model.link_index >= m_link_names.size()
            ))
        {
            return false;
        }
    }

    for (const auto& group_model : m_group_models) {
        for (int lidx : group_model.link_indices) {
            if (lidx < 0 || lidx >= m_link_names.size()) {
                return false;
            }
        }
    }

    return true;
}

int RobotCollisionModelImpl::generateAttachedBodyIndex()
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

void RobotCollisionModelImpl::generateSpheresModel(
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

void RobotCollisionModelImpl::generateVoxelsModel(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name,
    CollisionVoxelModelConfig& voxels_model)
{
    // TODO: see generateSpheresModel
    voxels_model.link_name = link_name;
}

bool RobotCollisionModelImpl::updateSpheresModelToSphereModelsReferences()
{
    // assumes you're finished changing the set of sphere models and spheres
    // models and updates the following references:
    // * spheres model -> sphere model[]
    for (CollisionSpheresModel& spheres_model : m_spheres_models) {
        spheres_model.spheres.clear();
    }

    for (size_t i = 0; i < m_spheres_models.size(); ++i) {
        CollisionSpheresModel& model = m_spheres_models[i];
        const CollisionSpheresModelConfig& config =
                i < m_config.spheres_models.size() ?
                        m_config.spheres_models[i] :
                        m_attached_body_spheres_config.at(model.body_index);

        for (const std::string& sphere_name : config.spheres) {
            auto sit = std::find_if(m_sphere_models.begin(), m_sphere_models.end(),
                    [&sphere_name](const CollisionSphereModel& sphere)
                    {
                        return sphere.name == sphere_name;
                    });
            if (sit == m_sphere_models.end()) {
                ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find sphere '%s' specified in spheres model for link '%s'", sphere_name.c_str(), config.link_name.c_str());
                // TODO: what to do if this fails
                return false;
            }

            const CollisionSphereModel* sphere_model =
                    m_sphere_models.data() +
                    std::distance(m_sphere_models.begin(), sit);
            model.spheres.push_back(sphere_model);
        }
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, " Regenerated sphere references");
    return true;
}

bool RobotCollisionModelImpl::updateLinkBodyToSpheresModelReferences()
{
    return false;
}

bool RobotCollisionModelImpl::updateLinkBodyToVoxelsModelReferences()
{
    return false;
}

Eigen::Affine3d RobotCollisionModelImpl::poseUrdfToEigen(const urdf::Pose& p) const
{
    return Eigen::Translation3d(p.position.x, p.position.y, p.position.z) *
            Eigen::Quaterniond(
                    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
}

bool RobotCollisionModelImpl::voxelizeLink(
    const urdf::ModelInterface& urdf,
    const std::string& link_name,
    CollisionVoxelsModel& model) const
{
    auto link = urdf.getLink(link_name);

    if (!link) {
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find link '%s' in the URDF", link_name.c_str());
        return false;
    }

    if (!link->collision && link->collision_array.empty()) {
        ROS_WARN_NAMED(RCM_LOGGER, "Failed to find collision elements of link '%s'", link->name.c_str());
        return true;
    }

    if (link->collision) {
        if (!voxelizeCollisionElement(
            *link->collision, model.voxel_res, model.voxels))
        {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize collision element for link '%s'", link_name.c_str());
            return false;
        }
    }
    else if (!link->collision_array.empty()) {
        for (auto collision : link->collision_array) {
            if (!voxelizeCollisionElement(
                    *collision, model.voxel_res, model.voxels))
            {
                ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize collision element for link '%s'", link_name.c_str());
                return false;
            }
        }
    }
    else {
        ROS_ERROR_NAMED(RCM_LOGGER, "Hmm");
        return false;
    }

    if (model.voxels.empty()) {
        ROS_WARN_NAMED(RCM_LOGGER, "Voxelizing collision elements for link '%s' produced 0 voxels", link_name.c_str());
    }

    return true;
}

bool RobotCollisionModelImpl::voxelizeAttachedBody(
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
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
                shape, transform, model.voxel_res, Eigen::Vector3d::Zero(), model.voxels);
    }

    return true;
}

bool RobotCollisionModelImpl::voxelizeCollisionElement(
    const urdf::Collision& collision,
    double res,
    std::vector<Eigen::Vector3d>& voxels) const
{
    auto geom = collision.geometry;

    if (!geom) {
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find geometry for collision element");
        return false;
    }

    Eigen::Affine3d pose;

    geometry_msgs::Pose p;
    p.position.x = collision.origin.position.x;
    p.position.y = collision.origin.position.y;
    p.position.z = collision.origin.position.z;
    collision.origin.rotation.getQuaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    tf::poseMsgToEigen(p, pose);

    return voxelizeGeometry(*geom, pose, res, voxels);
}

bool RobotCollisionModelImpl::voxelizeGeometry(
    const urdf::Geometry& geom,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels) const
{
    if (geom.type == urdf::Geometry::MESH) {
        geometry_msgs::Vector3 scale;
        scale.x = 1.0f; scale.y = 1.0f; scale.z = 1.0f;
        std::vector<geometry_msgs::Point> mesh_vertices;
        std::vector<int> triangles;
        urdf::Mesh* mesh = (urdf::Mesh*)&geom;
        if (!leatherman::getMeshComponentsFromResource(
                mesh->filename, scale, triangles, mesh_vertices))
        {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to get mesh from file. (%s)", mesh->filename.c_str());
            return false;
        }

        ROS_DEBUG_NAMED(RCM_LOGGER, "mesh: %s  triangles: %zu  vertices: %zu", mesh->filename.c_str(), triangles.size(), mesh_vertices.size());

        std::vector<Eigen::Vector3d> vertices(mesh_vertices.size());
        for (size_t vidx = 0; vidx < mesh_vertices.size(); ++vidx) {
            const geometry_msgs::Point& vertex = mesh_vertices[vidx];
            vertices[vidx] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
        }

        sbpl::VoxelizeMesh(vertices, triangles, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::BOX) {
        urdf::Box* box = (urdf::Box*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "box: { dims: %0.3f, %0.3f, %0.3f }", box->dim.x, box->dim.y, box->dim.z);
        sbpl::VoxelizeBox(box->dim.x, box->dim.y, box->dim.z, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::CYLINDER) {
        urdf::Cylinder* cyl = (urdf::Cylinder*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "cylinder: { radius: %0.3f, length: %0.3f }", cyl->radius, cyl->length);
        sbpl::VoxelizeCylinder(cyl->radius, cyl->length, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::SPHERE) {
        urdf::Sphere* sph = (urdf::Sphere*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "sphere: { radius: %0.3f }", sph->radius);
        sbpl::VoxelizeSphere(sph->radius, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else {
        ROS_ERROR_NAMED(RCM_LOGGER, "Unrecognized geometry type for voxelization");
        return false;
    }

    return true;
}

////////////////////////////////////////
// RobotCollisionModel Implementation //
////////////////////////////////////////

RobotCollisionModel::RobotCollisionModel() :
    m_impl(new RobotCollisionModelImpl)
{
}

RobotCollisionModel::~RobotCollisionModel()
{
}

bool RobotCollisionModel::init(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    return m_impl->init(urdf, config);
}

const std::string& RobotCollisionModel::name() const
{
    return m_impl->name();
}

const std::string& RobotCollisionModel::modelFrame() const
{
    return m_impl->modelFrame();
}

size_t RobotCollisionModel::jointVarCount() const
{
    return m_impl->jointVarCount();
}

const std::vector<std::string>& RobotCollisionModel::jointVarNames() const
{
    return m_impl->jointVarNames();
}

bool RobotCollisionModel::hasJointVar(const std::string& joint_name) const
{
    return m_impl->hasJointVar(joint_name);
}

int RobotCollisionModel::jointVarIndex(const std::string& joint_name) const
{
    return m_impl->jointVarIndex(joint_name);
}

const std::string& RobotCollisionModel::jointVarName(int jidx) const
{
    return m_impl->jointVarName(jidx);
}

bool RobotCollisionModel::jointVarIsContinuous(
    const std::string& joint_name) const
{
    return m_impl->jointVarIsContinuous(joint_name);
}

bool RobotCollisionModel::jointVarHasPositionBounds(
    const std::string& joint_name) const
{
    return m_impl->jointVarHasPositionBounds(joint_name);
}

double RobotCollisionModel::jointVarMaxPosition(
    const std::string& joint_name) const
{
    return m_impl->jointVarMaxPosition(joint_name);
}

double RobotCollisionModel::jointVarMinPosition(
    const std::string& joint_name) const
{
    return m_impl->jointVarMinPosition(joint_name);
}

bool RobotCollisionModel::jointVarIsContinuous(int jidx) const
{
    return m_impl->jointVarIsContinuous(jidx);
}

bool RobotCollisionModel::jointVarHasPositionBounds(int jidx) const
{
    return m_impl->jointVarHasPositionBounds(jidx);
}

double RobotCollisionModel::jointVarMinPosition(int jidx) const
{
    return m_impl->jointVarMinPosition(jidx);
}

double RobotCollisionModel::jointVarMaxPosition(int jidx) const
{
    return m_impl->jointVarMaxPosition(jidx);
}

size_t RobotCollisionModel::jointCount() const
{
    return m_impl->jointCount();
}

int RobotCollisionModel::jointParentLinkIndex(int jidx) const
{
    return m_impl->jointParentLinkIndex(jidx);
}

int RobotCollisionModel::jointChildLinkIndex(int jidx) const
{
    return m_impl->jointChildLinkIndex(jidx);
}

const Eigen::Affine3d& RobotCollisionModel::jointOrigin(int jidx) const
{
    return m_impl->jointOrigin(jidx);
}

const Eigen::Vector3d& RobotCollisionModel::jointAxis(int jidx) const
{
    return m_impl->jointAxis(jidx);
}

JointTransformFunction RobotCollisionModel::jointTransformFn(int jidx) const
{
    return m_impl->jointTransformFn(jidx);
}

size_t RobotCollisionModel::linkCount() const
{
    return m_impl->linkCount();
}

const std::vector<std::string>& RobotCollisionModel::linkNames() const
{
    return m_impl->linkNames();
}

bool RobotCollisionModel::hasLink(const std::string& link_name) const
{
    return m_impl->hasLink(link_name);
}

int RobotCollisionModel::linkIndex(const std::string& link_name) const
{
    return m_impl->linkIndex(link_name);
}

const std::string& RobotCollisionModel::linkName(int lidx) const
{
    return m_impl->linkName(lidx);
}

int RobotCollisionModel::linkParentJointIndex(int lidx) const
{
    return m_impl->linkParentJointIndex(lidx);
}

const std::vector<int>& RobotCollisionModel::linkChildJointIndices(
    int lidx) const
{
    return m_impl->linkChildJointIndices(lidx);
}

bool RobotCollisionModel::attachBody(
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

bool RobotCollisionModel::detachBody(const std::string& id)
{
    return m_impl->detachBody(id);
}

size_t RobotCollisionModel::attachedBodyCount() const
{
    return m_impl->attachedBodyCount();
}

bool RobotCollisionModel::hasAttachedBody(const std::string& id) const
{
    return m_impl->hasAttachedBody(id);
}

int RobotCollisionModel::attachedBodyIndex(const std::string& id) const
{
    return m_impl->attachedBodyIndex(id);
}

const std::string& RobotCollisionModel::attachedBodyName(int abidx) const
{
    return m_impl->attachedBodyName(abidx);
}

int RobotCollisionModel::attachedBodyLinkIndex(int abidx) const
{
    return m_impl->attachedBodyLinkIndex(abidx);
}

const std::vector<int>& RobotCollisionModel::attachedBodyIndices(
    const std::string& link_name) const
{
    return m_impl->attachedBodyIndices(link_name);
}

const std::vector<int>& RobotCollisionModel::attachedBodyIndices(int lidx) const
{
    return m_impl->attachedBodyIndices(lidx);
}

size_t RobotCollisionModel::sphereModelCount() const
{
    return m_impl->sphereModelCount();
}

const CollisionSphereModel& RobotCollisionModel::sphereModel(int smidx) const
{
    return m_impl->sphereModel(smidx);
}

bool RobotCollisionModel::hasSpheresModel(const std::string& link_name) const
{
    return m_impl->hasSpheresModel(link_name);
}

bool RobotCollisionModel::hasSpheresModel(int lidx) const
{
    return m_impl->hasSpheresModel(lidx);
}

size_t RobotCollisionModel::spheresModelCount() const
{
    return m_impl->spheresModelCount();
}

const CollisionSpheresModel& RobotCollisionModel::spheresModel(int smidx) const
{
    return m_impl->spheresModel(smidx);
}

bool RobotCollisionModel::hasVoxelsModel(const std::string& link_name) const
{
    return m_impl->hasVoxelsModel(link_name);
}

bool RobotCollisionModel::hasVoxelsModel(int lidx) const
{
    return m_impl->hasVoxelsModel(lidx);
}

size_t RobotCollisionModel::voxelsModelCount() const
{
    return m_impl->voxelsModelCount();
}

const CollisionVoxelsModel& RobotCollisionModel::voxelsModel(int vmidx) const
{
    return m_impl->voxelsModel(vmidx);
}

size_t RobotCollisionModel::groupCount() const
{
    return m_impl->groupCount();
}

const CollisionGroupModel& RobotCollisionModel::group(int gidx) const
{
    return m_impl->group(gidx);
}

bool RobotCollisionModel::hasGroup(const std::string& group_name) const
{
    return m_impl->hasGroup(group_name);
}

int RobotCollisionModel::groupIndex(const std::string& group_name) const
{
    return m_impl->groupIndex(group_name);
}

const std::string& RobotCollisionModel::groupName(int gidx) const
{
    return m_impl->groupName(gidx);
}

const std::vector<int>& RobotCollisionModel::groupLinkIndices(
    const std::string& group_name) const
{
    return m_impl->groupLinkIndices(group_name);
}

const std::vector<int>& RobotCollisionModel::groupLinkIndices(int gidx) const
{
    return m_impl->groupLinkIndices(gidx);
}

bool RobotCollisionModel::registerRobotCollisionState(RobotCollisionState* state)
{
    return m_impl->registerRobotCollisionState(state);
}

bool RobotCollisionModel::unregisterRobotCollisionState(RobotCollisionState* state)
{
    return m_impl->unregisterRobotCollisionState(state);
}

} // namespace collision
} // namespace sbpl
