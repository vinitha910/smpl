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
#include <angles/angles.h>
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
#include <sbpl_collision_checking/robot_collision_model.h>
#include "voxel_operations.h"

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

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

typedef Eigen::Affine3d (*JointTransformFunction)(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

static const char* RCM_LOGGER = "robot";

std::string AffineToString(const Eigen::Affine3d& t)
{
    const Eigen::Vector3d pos(t.translation());
    const Eigen::Quaterniond rot(t.rotation());
    const int ENOUGH = 1024;
    char buff[ENOUGH] = { 0 };
    snprintf(buff, ENOUGH, "{ pos = (%0.3f, %0.3f, %0.3f), rot = (%0.3f, %0.3f, %0.3f, %0.3f) }", pos.x(), pos.y(), pos.z(), rot.w(), rot.x(), rot.y(), rot.z());
    return std::string(buff);
}

////////////////////////////////////////
// Compute*JointTransform Definitions //
////////////////////////////////////////

Eigen::Affine3d ComputeRevoluteJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Affine3d ComputeContinuousJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Affine3d ComputePrismaticJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Affine3d ComputeFloatingJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Affine3d ComputePlanarJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Affine3d ComputeFixedJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

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

    size_t linkCount() const;
    auto   linkNames() const -> const std::vector<std::string>&;
    bool   hasLink(const std::string& link_name) const;
    int    linkIndex(const std::string& link_name) const;
    auto   linkName(int lidx) const -> const std::string&;

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

    auto attachedBodyIndices(const std::string& link_name) const ->
            const std::vector<int>&;
    auto attachedBodyIndices(int lidx) const -> const std::vector<int>&;

    size_t sphereModelCount() const;
    auto   sphereModel(int smidx) const -> const CollisionSphereModel&;
    bool   hasSpheresModel(const std::string& link_name) const;
    bool   hasSpheresModel(int lidx) const;

    bool   hasVoxelsModel(const std::string& link_name) const;
    bool   hasVoxelsModel(int lidx) const;
    size_t voxelsModelCount() const;
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    size_t groupCount() const;
    auto   groups() const -> const std::vector<CollisionGroupModel>&;
    bool   hasGroup(const std::string& group_name) const;
    int    groupIndex(const std::string& group_name) const;
    auto   groupName(int gidx) const -> const std::string&;
    auto   groupLinkIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupLinkIndices(int gidx) const ->
            const std::vector<int>&;
    auto   groupSphereStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupSphereStateIndices(int gidx) const ->
            const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;

    auto   worldToModelTransform() const -> const Eigen::Affine3d&;
    bool   setWorldToModelTransform(const Eigen::Affine3d& transform);

    auto   jointPositions() const -> const std::vector<double>&;
    auto   linkTransforms() const -> const Affine3dVector&;

    double jointPosition(const std::string& joint_name) const;
    double jointPosition(int jidx) const;

    bool   setJointPosition(const std::string& joint_name, double position);
    bool   setJointPosition(int jidx, double position);

    auto   linkTransform(const std::string& link_name) const ->
            const Eigen::Affine3d&;
    auto   linkTransform(int lidx) const -> const Eigen::Affine3d&;

    bool   linkTransformDirty(const std::string& link_name) const;
    bool   linkTransformDirty(int lidx) const;

    bool   updateLinkTransforms();
    bool   updateLinkTransform(int lidx);
    bool   updateLinkTransform(const std::string& link_name);

    auto attachedBodyTransform(const std::string& id) const ->
            const Eigen::Affine3d&;
    auto attachedBodyTransform(int abidx) const -> const Eigen::Affine3d&;

    bool attachedBodyTransformDirty(const std::string& id) const;
    bool attachedBodyTransformDirty(int abidx) const;

    bool updateAttachedBodyTransform(const std::string& id);
    bool updateAttachedBodyTransform(int abidx);

    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto sphereState(int ssidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(int ssidx) const;
    bool updateSphereStates();
    bool updateSphereState(int ssidx);

    auto getVisualization() const ->
        visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const ->
        visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const ->
        visualization_msgs::MarkerArray;
    auto getStaticModelVisualization() const ->
            visualization_msgs::MarkerArray;
    auto getStaticModelVisualization(const std::string& group_name) const ->
            visualization_msgs::MarkerArray;
    auto getStaticModelVisualization(int gidx) const ->
            visualization_msgs::MarkerArray;
    auto getDynamicModelVisualization() const ->
            visualization_msgs::MarkerArray;
    auto getDynamicModelVisualization(const std::string& group_name) const ->
            visualization_msgs::MarkerArray;
    auto getDynamicModelVisualization(int gidx) const ->
            visualization_msgs::MarkerArray;

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

    std::vector<double*>                    m_joint_var_offset;
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

    /// \name Robot State
    ///@{
    std::vector<double>             m_jvar_positions;           // per joint

    std::vector<bool>               m_dirty_link_transforms;    // per link
    Affine3dVector                  m_link_transforms;          // per link
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

    /// \name Collision State
    ///@{

    // one entry for each unique sphere, i.e. for each sphere model referenced
    // for each spheres model
    std::vector<bool>                       m_dirty_sphere_states;
    std::vector<CollisionSphereState>       m_sphere_states;

    // one entry for each spheres model
    std::vector<CollisionSpheresState>      m_spheres_states;

    // one entry for each voxels model
    std::vector<bool>                       m_dirty_voxels_states;
    std::vector<CollisionVoxelsState>       m_voxels_states;

    // one entry for each corresponding group model
    std::vector<CollisionGroupState>        m_group_states;

    // per-link references to corresponding spheres and voxels states
    std::vector<CollisionVoxelsState*>      m_link_voxels_states;
    std::vector<CollisionSpheresState*>     m_link_spheres_states;

    hash_map<int, CollisionSpheresState*>   m_attached_body_spheres_states;
    hash_map<int, CollisionVoxelsState*>    m_attached_body_voxels_states;

    ///@}

    bool initRobotModel(const urdf::ModelInterface& urdf);
    bool initRobotState();
    bool initCollisionModel(
        const urdf::ModelInterface& urdf,
        const CollisionModelConfig& config);
    bool initCollisionState();

    bool checkCollisionModelReferences() const;
    bool checkCollisionStateReferences() const;

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
    bool updateSpheresStateToSphereStatesReferences();
    bool updateVoxelsStateToModelReferences();
    bool updateLinkBodyToSpheresModelReferences();
    bool updateLinkBodyToVoxelsModelReferences();
    bool updateLinkBodyToSpheresStateReferences();
    bool updateLinkBodyToVoxelsStateReferences();

    bool updateGroupStateToModelReferences();

    bool updateGroupStateReferences();

    // update the set of indices into sphere states and set of indices into
    // voxels states for a collision group; assumes the group model's
    // link_indices and body_indices are valid and the
    // m_[link|attached_body]_[voxels|spheres]_states mappings are current
    bool updateGroupStateReferences(
        const CollisionGroupModel& model,
        CollisionGroupState& state);

    void clear();

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
// Compute*JointTransform Definitions //
////////////////////////////////////////

Eigen::Affine3d ComputeRevoluteJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin * Eigen::AngleAxisd(jvals[0], axis);
}

Eigen::Affine3d ComputeContinuousJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin * Eigen::AngleAxisd(jvals[0], axis);
}

Eigen::Affine3d ComputePrismaticJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin * Eigen::Translation3d(Eigen::Vector3d(
            0.0, 0.0, jvals[0]));
}

Eigen::Affine3d ComputeFloatingJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin *
            Eigen::Translation3d(Eigen::Vector3d(jvals[0], jvals[1], jvals[2])) *
            Eigen::Quaterniond(jvals[6], jvals[3], jvals[4], jvals[5]);
}

Eigen::Affine3d ComputePlanarJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin *
            Eigen::Translation3d(Eigen::Vector3d(jvals[0], jvals[1], 0.0)) *
            Eigen::AngleAxisd(jvals[2], Eigen::Vector3d::UnitZ());
}

Eigen::Affine3d ComputeFixedJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin;
}

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
    m_jvar_positions(),
    m_dirty_link_transforms(),
    m_link_transforms(),
    m_sphere_models(),
    m_spheres_models(),
    m_voxels_models(),
    m_group_models(),
    m_group_name_to_index(),
    m_link_spheres_models(),
    m_link_voxels_models(),
    m_dirty_sphere_states(),
    m_sphere_states(),
    m_dirty_voxels_states(),
    m_voxels_states(),
    m_group_states(),
    m_link_voxels_states(),
    m_link_spheres_states()
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
    success = success && initRobotState();
    success = success && initCollisionModel(urdf, config);
    success = success && initCollisionState();

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
double RobotCollisionModelImpl::jointVarMaxPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_max_positions[jidx];
}

inline
double RobotCollisionModelImpl::jointVarMinPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_min_positions[jidx];
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

    // initialize sphere(s) states and update all references between sphere(s)
    // states and sphere(s) models
    if (spheres_model) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "Adding sphere(s) states and updating references");
        // append new sphere states
        int unique_sphere_count = spheres_model->spheres.size();
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
    if (voxels_model) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "Adding voxels state and updating references");
        m_voxels_states.resize(m_voxels_states.size() + 1);
        CollisionVoxelsState& voxels_state = m_voxels_states.back();
        voxels_state.voxels = voxels_model->voxels;
        m_dirty_voxels_states.resize(m_dirty_voxels_states.size() + 1, true);

        updateVoxelsStateToModelReferences();
        updateLinkBodyToVoxelsStateReferences();
    }

    // update all references from group states to sphere and voxels states
    assert(m_group_models.size() == m_group_states.size());
    updateGroupStateReferences();

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
    auto vsit = m_attached_body_voxels_states.find(abidx);
    assert(vsit != m_attached_body_voxels_states.end());
    CollisionVoxelsState* voxels_state = vsit->second;
    const CollisionVoxelsModel* voxels_model = voxels_state ?
            voxels_state->model : nullptr;

    // remove the per-body voxels model and state
    m_attached_body_voxels_states.erase(abidx);
    m_attached_body_voxels_models.erase(abidx);

    if (voxels_state) {
        // remove the voxels state
        const auto vsidx = std::distance(m_voxels_states.data(), voxels_state);
        m_voxels_states.erase(m_voxels_states.begin() + vsidx);
        m_dirty_voxels_states.erase(m_dirty_voxels_states.begin() + vsidx);
    }

    if (voxels_model) {
        // remove the voxels model
        const auto vmidx = std::distance(
                // really, c++?
                const_cast<const CollisionVoxelsModel*>(m_voxels_models.data()),
                voxels_model);
        m_voxels_models.erase(m_voxels_models.begin() + vmidx);
    }

    updateLinkBodyToVoxelsStateReferences();
    updateLinkBodyToVoxelsModelReferences();
    updateVoxelsStateToModelReferences();

    // remove the stashed configuration
    m_attached_body_voxels_config.erase(abidx);

    // get the corresponding spheres model and state, if one exists
    auto ssit = m_attached_body_spheres_states.find(abidx);
    assert(ssit != m_attached_body_spheres_states.end());
    CollisionSpheresState* spheres_state = ssit->second;
    const CollisionSpheresModel* spheres_model = spheres_state ?
            spheres_state->model : nullptr;

    // remove the per-body spheres model and state
    m_attached_body_spheres_states.erase(abidx);
    m_attached_body_spheres_models.erase(abidx);

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

    updateLinkBodyToSpheresStateReferences();
    updateLinkBodyToSpheresModelReferences();
    updateSpheresModelToSphereModelsReferences();
    updateSpheresStateToSphereStatesReferences();

    m_attached_body_spheres_config.erase(abidx);

    // remove from group models
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        CollisionGroupModel& group_model = m_group_models[i];
        std::remove_if(
                group_model.body_indices.begin(),
                group_model.body_indices.end(),
                [&](int bidx) { return bidx == abidx; });
    }

    updateGroupStateReferences();

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
const std::vector<CollisionGroupModel>& RobotCollisionModelImpl::groups() const
{
    return m_group_models;
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
const std::vector<int>& RobotCollisionModelImpl::groupSphereStateIndices(
    const std::string& group_name) const
{
    const int gidx = groupIndex(group_name);
    return m_group_states[gidx].sphere_indices;
}

inline
const std::vector<int>& RobotCollisionModelImpl::groupSphereStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].sphere_indices;
}

inline
const std::vector<int>& RobotCollisionModelImpl::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    const int gidx = groupIndex(group_name);
    return m_group_states[gidx].voxels_indices;
}

inline
const std::vector<int>& RobotCollisionModelImpl::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].voxels_indices;
}

inline
const Eigen::Affine3d& RobotCollisionModelImpl::worldToModelTransform() const
{
    return m_joint_origins[0];
}

bool RobotCollisionModelImpl::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    if (!transform.isApprox(m_joint_origins[0], 0.0)) {
        m_joint_origins[0] = transform;
        std::fill(m_dirty_link_transforms.begin(), m_dirty_link_transforms.end(), true);
        std::fill(m_dirty_voxels_states.begin(), m_dirty_voxels_states.end(), true);
        std::fill(m_dirty_sphere_states.begin(), m_dirty_sphere_states.end(), true);
        return true;
    }
    else {
        return false;
    }
}

inline
const std::vector<double>& RobotCollisionModelImpl::jointPositions() const
{
    return m_jvar_positions;
}

inline
const Affine3dVector& RobotCollisionModelImpl::linkTransforms() const
{
    return m_link_transforms;
}

inline
double RobotCollisionModelImpl::jointPosition(const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_positions[jidx];
}

inline
double RobotCollisionModelImpl::jointPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, jidx);
    return m_jvar_positions[jidx];
}

inline
bool RobotCollisionModelImpl::setJointPosition(
    const std::string& joint_name,
    double position)
{
    const int jidx = jointVarIndex(joint_name);
    return setJointPosition(jidx, position);
}

bool RobotCollisionModelImpl::setJointPosition(int jidx, double position)
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, jidx);

    if (m_jvar_positions[jidx] != position) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "Setting joint position of joint %d to %0.3f", jidx, position);

        m_jvar_positions[jidx] = position;

        // TODO: cache affected link transforms in a per-joint array?

        std::queue<int> q;
        q.push(m_joint_child_links[jidx]);
        while (!q.empty()) {
            int lidx = q.front();
            q.pop();

            ROS_DEBUG_NAMED(RCM_LOGGER, "Dirtying transform to link '%s'", m_link_names[lidx].c_str());

            // dirty the transform of the affected link
            m_dirty_link_transforms[lidx] = true;

            // dirty the voxels states of any attached voxels model
            if (m_link_voxels_states[lidx]) {
                int dvsidx = std::distance(m_voxels_states.data(), m_link_voxels_states[lidx]);
                m_dirty_voxels_states[dvsidx] = true;
            }

            // dirty the sphere states of any attached sphere models
            if (m_link_spheres_states[lidx]) {
                for (CollisionSphereState* sphere_state :
                    m_link_spheres_states[lidx]->spheres)
                {
                    int dssidx = std::distance(m_sphere_states.data(), sphere_state);
                    m_dirty_sphere_states[dssidx] = true;
                }
            }

            // add child links to the queue
            for (int cjidx : m_link_children_joints[lidx]) {
                q.push(m_joint_child_links[cjidx]);
            }
        }

        return true;
    }
    else {
        return false;
    }
}

inline
const Eigen::Affine3d& RobotCollisionModelImpl::linkTransform(
    const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_transforms[lidx];
}

inline
const Eigen::Affine3d& RobotCollisionModelImpl::linkTransform(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_transforms, lidx);
    return m_link_transforms[lidx];
}

inline
bool RobotCollisionModelImpl::linkTransformDirty(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_dirty_link_transforms[lidx];
}

inline
bool RobotCollisionModelImpl::linkTransformDirty(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    return m_dirty_link_transforms[lidx];
}

bool RobotCollisionModelImpl::updateLinkTransforms()
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating all link transforms");
    bool updated = false;
    for (size_t lidx = 0; lidx < m_link_names.size(); ++lidx) {
        updated |= updateLinkTransform(lidx);
    }
    return updated;
}

bool RobotCollisionModelImpl::updateLinkTransform(int lidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    if (!m_dirty_link_transforms[lidx]) {
        return false;
    }

    int pjidx = m_link_parent_joints[lidx];
    int plidx = m_joint_parent_links[pjidx];

    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating transform for link '%s'. parent joint = %d, parent link = %d", m_link_names[lidx].c_str(), pjidx, plidx);

    if (plidx >= 0) {
        // recursively update the kinematic tree
        // TODO: optimize out recursion
        updateLinkTransform(plidx);
    }

    const Eigen::Affine3d& T_model_parent = (plidx >= 0) ?
            (m_link_transforms[plidx]) : (Eigen::Affine3d::Identity());

    const Eigen::Affine3d& T_parent_link =
            m_joint_transforms[pjidx](
                    m_joint_origins[pjidx],
                    m_joint_axes[pjidx],
                    m_joint_var_offset[pjidx]);

    m_link_transforms[lidx] = T_model_parent * T_parent_link;

    ROS_DEBUG_NAMED(RCM_LOGGER, " -> %s", AffineToString(m_link_transforms[lidx]).c_str());

    m_dirty_link_transforms[lidx] = false;
    return true;
}

inline
bool RobotCollisionModelImpl::updateLinkTransform(const std::string& link_name)
{
    const int lidx = linkIndex(link_name);
    return updateLinkTransform(lidx);
}

inline
const Eigen::Affine3d& RobotCollisionModelImpl::attachedBodyTransform(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    auto bit = m_attached_bodies.find(it->second);
    assert(bit != m_attached_bodies.end());
    const int lidx = bit->second.link_index;
    assert(lidx >= 0 && lidx < m_link_transforms.size());
    return m_link_transforms[lidx];
}

inline
const Eigen::Affine3d& RobotCollisionModelImpl::attachedBodyTransform(
    int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    const int lidx = it->second.link_index;
    assert(lidx >= 0 && lidx < m_link_transforms.size());
    return m_link_transforms[it->second.link_index];
}

inline
bool RobotCollisionModelImpl::attachedBodyTransformDirty(
    const std::string& id) const
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    auto bit = m_attached_bodies.find(it->second);
    assert(bit != m_attached_bodies.end());
    const int lidx = bit->second.link_index;
    assert(lidx >= 0 && lidx < m_dirty_link_transforms.size());
    return m_dirty_link_transforms[lidx];
}

inline
bool RobotCollisionModelImpl::attachedBodyTransformDirty(int abidx) const
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    const int lidx = it->second.link_index;
    assert(lidx >= 0 && lidx < m_dirty_link_transforms.size());
    return m_dirty_link_transforms[lidx];
}

inline
bool RobotCollisionModelImpl::updateAttachedBodyTransform(const std::string& id)
{
    auto it = m_attached_body_name_to_index.find(id);
    ASSERT_RANGE(it != m_attached_body_name_to_index.end());
    auto bit = m_attached_bodies.find(it->second);
    assert(bit != m_attached_bodies.end());
    const int lidx = bit->second.link_index;
    assert(lidx >= 0 && lidx < m_link_transforms.size());
    return updateLinkTransform(lidx);
}

inline
bool RobotCollisionModelImpl::updateAttachedBodyTransform(int abidx)
{
    auto it = m_attached_bodies.find(abidx);
    ASSERT_RANGE(it != m_attached_bodies.end());
    const int lidx = it->second.link_index;
    assert(lidx >= 0 && lidx < m_link_transforms.size());
    return updateLinkTransform(lidx);
}

inline
const CollisionVoxelsState& RobotCollisionModelImpl::voxelsState(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    return m_voxels_states[vsidx];
}

inline
bool RobotCollisionModelImpl::voxelsStateDirty(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);
    return m_dirty_voxels_states[vsidx];
}

bool RobotCollisionModelImpl::updateVoxelsStates()
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating all voxels states");
    bool updated = false;
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        updated |= updateVoxelsState(vsidx);
    }
    return updated;
}

bool RobotCollisionModelImpl::updateVoxelsState(int vsidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);

    if (!m_dirty_voxels_states[vsidx]) {
        return false;
    }

    CollisionVoxelsState& state = m_voxels_states[vsidx];

    std::vector<Eigen::Vector3d> new_voxels;
    if (state.model->body_index < 0) {
        const int lidx = state.model->link_index;
        updateLinkTransform(lidx);

        const Eigen::Affine3d& T_model_link = m_link_transforms[lidx];

        // transform voxels into the model frame
        new_voxels.resize(state.model->voxels.size());
        for (size_t i = 0; i < state.model->voxels.size(); ++i) {
            new_voxels[i] = T_model_link * state.model->voxels[i];
        }
    }
    else {
        const int bidx = state.model->body_index;
        updateAttachedBodyTransform(bidx);

        const Eigen::Affine3d& T_model_body = attachedBodyTransform(bidx);

        // transform voxels into the model frame
        new_voxels.resize(state.model->voxels.size());
        for (size_t i = 0; i < state.model->voxels.size(); ++i) {
            new_voxels[i] = T_model_body * state.model->voxels[i];
        }
    }

    state.voxels = std::move(new_voxels);
    m_dirty_voxels_states[vsidx] = false;
    return true;
}

inline
const CollisionSphereState& RobotCollisionModelImpl::sphereState(int ssidx) const
{
    ASSERT_VECTOR_RANGE(m_sphere_states, ssidx);
    return m_sphere_states[ssidx];
}

inline
bool RobotCollisionModelImpl::sphereStateDirty(int ssidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, ssidx);
    return m_dirty_sphere_states[ssidx];
}

bool RobotCollisionModelImpl::updateSphereStates()
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating all sphere positions");
    bool updated = false;
    for (size_t ssidx = 0; ssidx < m_sphere_states.size(); ++ssidx) {
        updated |= updateSphereState(ssidx);
    }
    return updated;
}

bool RobotCollisionModelImpl::updateSphereState(int sidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, sidx);

    if (!m_dirty_sphere_states[sidx]) {
        return false;
    }

    if (m_sphere_states[sidx].parent_state->model->body_index < 0) {
        const int lidx = m_sphere_states[sidx].parent_state->model->link_index;
        updateLinkTransform(lidx);

        ROS_DEBUG_NAMED(RCM_LOGGER, "Updating position of sphere '%s'", m_sphere_states[sidx].model->name.c_str());
        const Eigen::Affine3d& T_model_link = m_link_transforms[lidx];
        m_sphere_states[sidx].pos = T_model_link * m_sphere_states[sidx].model->center;
    }
    else {
        const int bidx = m_sphere_states[sidx].parent_state->model->body_index;
        updateAttachedBodyTransform(bidx);

        ROS_DEBUG_NAMED(RCM_LOGGER, "Updating position of sphere '%s'", m_sphere_states[sidx].model->name.c_str());
        const Eigen::Affine3d& T_model_body = attachedBodyTransform(bidx);
        m_sphere_states[sidx].pos = T_model_body * m_sphere_states[sidx].model->center;
    }

    m_dirty_sphere_states[sidx] = false;
    return true;
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getVisualization() const
{
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < groupCount(); ++i) {
        auto marr = getVisualization(i);
        ma.markers.insert(ma.markers.end(), marr.markers.begin(), marr.markers.end());
    }
    return ma;
}

inline
visualization_msgs::MarkerArray RobotCollisionModelImpl::getVisualization(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    return getVisualization(it->second);
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getVisualization(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    const CollisionGroupState& group_state = m_group_states[gidx];

    std::vector<std::vector<double>> spheres;
    std::vector<double> rad;
    spheres.reserve(group_state.sphere_indices.size());
    rad.reserve(group_state.sphere_indices.size());

    for (int ssidx : group_state.sphere_indices) {
        const CollisionSphereState& sphere_state = m_sphere_states[ssidx];
        std::vector<double> sphere(4, 0.0);
        sphere[0] = sphere_state.pos.x();
        sphere[1] = sphere_state.pos.y();
        sphere[2] = sphere_state.pos.z();
        sphere[3] = sphere_state.model->radius;
        spheres.push_back(std::move(sphere));
        rad.push_back(sphere_state.model->radius);
    }

    const int hue = 90;
    return viz::getSpheresMarkerArray(
            spheres, rad, hue, "", "collision_model", 0);
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getStaticModelVisualization() const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getStaticModelVisualization(
    const std::string& group_name) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getStaticModelVisualization(int gidx) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getDynamicModelVisualization() const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getDynamicModelVisualization(
    const std::string& group_name) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionModelImpl::getDynamicModelVisualization(int gidx) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
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

    // initialize joint variable offsets
    // NOTE: need to initialize this before initRobotState to set up these
    // references
    m_jvar_positions.assign(m_jvar_names.size(), 0.0);

    m_joint_var_offset.resize(m_joint_transforms.size());
    double* d = m_jvar_positions.data();
    for (size_t i = 0; i < m_joint_transforms.size(); ++i) {
        m_joint_var_offset[i] = d;
        JointTransformFunction f = m_joint_transforms[i];
        if (f == ComputeFixedJointTransform) {
            d += 0;
        }
        else if (f == ComputeRevoluteJointTransform ||
                f == ComputeContinuousJointTransform ||
                f == ComputePrismaticJointTransform)
        {
            d += 1;
        }
        else if (f == ComputePlanarJointTransform) {
            d += 3;
        }
        else if (f == ComputeFloatingJointTransform) {
            d += 7;
        }
        else {
            ROS_ERROR_NAMED(RCM_LOGGER, "Unrecognized JointTransformFunction");
            return false;
        }
    }

    ROS_INFO_NAMED(RCM_LOGGER, "ComputeFixedJointTransform: %p", ComputeFixedJointTransform);
    ROS_INFO_NAMED(RCM_LOGGER, "ComputeRevoluteJointTransform: %p", ComputeRevoluteJointTransform);
    ROS_INFO_NAMED(RCM_LOGGER, "ComputeContinuousJointTransform: %p", ComputeContinuousJointTransform);
    ROS_INFO_NAMED(RCM_LOGGER, "ComputePrismaticJointTransform: %p", ComputePrismaticJointTransform);
    ROS_INFO_NAMED(RCM_LOGGER, "ComputePlanarJointTransform: %p", ComputePlanarJointTransform);
    ROS_INFO_NAMED(RCM_LOGGER, "ComputeFloatingJointTransform: %p", ComputeFloatingJointTransform);

    ROS_INFO_NAMED(RCM_LOGGER, "Robot Model:");
    ROS_INFO_NAMED(RCM_LOGGER, "  Name: %s", m_name.c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Model Frame: %s", m_model_frame.c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Variable Names: %s", to_string(m_jvar_names).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Variable Continuous: %s", to_string(m_jvar_continuous).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Variable Has Position Bounds: %s", to_string(m_jvar_has_position_bounds).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Variable Min Positions: %s", to_string(m_jvar_min_positions).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Variable Max Positions: %s", to_string(m_jvar_max_positions).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Variable Offsets: %s", to_string(m_joint_var_offset).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Transform Functions:");
    for (auto p : m_joint_transforms) {
        ROS_INFO_NAMED(RCM_LOGGER, "    %p", p);
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Origins:");
    for (const auto& o : m_joint_origins) {
        ROS_INFO_NAMED(RCM_LOGGER, "    %s", AffineToString(o).c_str());
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Parent Links: %s", to_string(m_joint_parent_links).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Joint Child Links: %s", to_string(m_joint_child_links).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Link Names: %s", to_string(m_link_names).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Link Parent Joints: %s", to_string(m_link_parent_joints).c_str());
    ROS_INFO_NAMED(RCM_LOGGER, "  Link Child Joints: %s", to_string(m_link_children_joints).c_str());

    return true;
}

bool RobotCollisionModelImpl::initRobotState()
{
    assert(!m_joint_origins.empty());
    m_joint_origins[0] = Eigen::Affine3d::Identity();

//    m_jvar_positions.assign(m_jvar_names.size(), 0.0);
    m_dirty_link_transforms.assign(m_link_names.size(), true);
    m_link_transforms.assign(m_link_names.size(), Eigen::Affine3d::Identity());

    ROS_INFO_NAMED(RCM_LOGGER, "Robot State:");
    ROS_INFO_NAMED(RCM_LOGGER, "  %zu Joint Positions", m_jvar_positions.size());
    ROS_INFO_NAMED(RCM_LOGGER, "  %zu Dirty Link Transforms", m_dirty_link_transforms.size());
    ROS_INFO_NAMED(RCM_LOGGER, "  %zu Link Transforms", m_link_transforms.size());
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

    ROS_INFO_NAMED(RCM_LOGGER, "Collision Model:");
    ROS_INFO_NAMED(RCM_LOGGER, "  Sphere Models: [%p, %p]", m_sphere_models.data(), m_sphere_models.data() + m_sphere_models.size());
    for (const auto& sphere_model : m_sphere_models) {
        ROS_INFO_NAMED(RCM_LOGGER, "    name: %s, center: (%0.3f, %0.3f, %0.3f), radius: %0.3f, priority: %d", sphere_model.name.c_str(), sphere_model.center.x(), sphere_model.center.y(), sphere_model.center.z(), sphere_model.radius, sphere_model.priority);
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Spheres Models: [%p, %p]", m_spheres_models.data(), m_spheres_models.data() + m_spheres_models.size());
    for (const auto& spheres_model : m_spheres_models) {
        ROS_INFO_NAMED(RCM_LOGGER, "    link_index: %d, spheres: %s", spheres_model.link_index, to_string(spheres_model.spheres).c_str());
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Voxels Models: [%p, %p]", m_voxels_models.data(), m_voxels_models.data() + m_voxels_models.size());
    for (const auto& voxels_model : m_voxels_models) {
        ROS_INFO_NAMED(RCM_LOGGER, "    link_index: %d, voxel_res: %0.3f, voxel count: %zu", voxels_model.link_index, voxels_model.voxel_res, voxels_model.voxels.size());
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Group Models:");
    for (const auto& group_model : m_group_models) {
        ROS_INFO_NAMED(RCM_LOGGER, "    name: %s, link_indices: %s", group_model.name.c_str(), to_string(group_model.link_indices).c_str());
    }

    return true;
}

bool RobotCollisionModelImpl::initCollisionState()
{
    // preallocate the spheres states array so that we can use valid pointers
    // as references

    int unique_sphere_count = 0;
    for (const auto& spheres_model : m_spheres_models) {
        for (auto sphere_model : spheres_model.spheres) {
            ++unique_sphere_count;
        }
    }

    m_sphere_states.assign(unique_sphere_count, CollisionSphereState());

    // initialize sphere and spheres states
    int sphere_state_idx = 0;
    m_spheres_states.assign(m_spheres_models.size(), CollisionSpheresState());
    for (size_t i = 0; i < m_spheres_models.size(); ++i) {
        const CollisionSpheresModel& spheres_model = m_spheres_models[i];
              CollisionSpheresState& spheres_state = m_spheres_states[i];
        spheres_state.model = &spheres_model;

        // initialize sphere and map sphere state -> spheres state and spheres
        // state -> sphere states
        for (const CollisionSphereModel* sphere_model : spheres_model.spheres) {
            m_sphere_states[sphere_state_idx].model = sphere_model;
            m_sphere_states[sphere_state_idx].parent_state = &spheres_state;
            spheres_state.spheres.push_back(&m_sphere_states[sphere_state_idx]);
            ++sphere_state_idx;
        }
    }

    m_dirty_sphere_states.assign(m_sphere_states.size(), true);

    // initialize voxels states

    m_dirty_voxels_states.assign(m_voxels_models.size(), true);
    m_voxels_states.assign(m_voxels_models.size(), CollisionVoxelsState());

    for (size_t i = 0; i < m_voxels_models.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_voxels_models[i];
              CollisionVoxelsState& voxels_state = m_voxels_states[i];
        // duplicate voxels from voxels model
        voxels_state.model = &voxels_model;
        voxels_state.voxels = voxels_model.voxels;
    }

    // initialize link voxels states
    m_link_voxels_states.assign(m_link_names.size(), nullptr);
    for (int i = 0; i < m_voxels_states.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_voxels_models[i];
        CollisionVoxelsState* voxels_state = &m_voxels_states[i];
        m_link_voxels_states[voxels_model.link_index] = voxels_state;
    }

    // initialize link spheres states
    m_link_spheres_states.assign(m_link_names.size(), nullptr);
    for (int i = 0; i < m_spheres_states.size(); ++i) {
        const CollisionSpheresModel& spheres_model = m_spheres_models[i];
        CollisionSpheresState* spheres_state = &m_spheres_states[i];
        m_link_spheres_states[spheres_model.link_index] = spheres_state;
    }

    // initialize group states

    m_group_states.assign(m_group_models.size(), CollisionGroupState());

    updateGroupStateToModelReferences();
    updateGroupStateReferences();

    assert(checkCollisionStateReferences());

    ROS_INFO_NAMED(RCM_LOGGER, "Collision State:");
    ROS_INFO_NAMED(RCM_LOGGER, "  Dirty Sphere States: %zu", m_dirty_sphere_states.size());
    ROS_INFO_NAMED(RCM_LOGGER, "  Sphere States: [%p, %p]", m_sphere_states.data(), m_sphere_states.data() + m_sphere_states.size());
    for (const auto& sphere_state : m_sphere_states) {
        ROS_INFO_NAMED(RCM_LOGGER, "    model: %p, parent_state: %p, pos: (%0.3f, %0.3f, %0.3f)", sphere_state.model, sphere_state.parent_state, sphere_state.pos.x(), sphere_state.pos.y(), sphere_state.pos.z());
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Spheres States: [%p, %p]", m_spheres_states.data(), m_spheres_states.data() + m_spheres_states.size());
    for (const auto& spheres_state : m_spheres_states) {
        ROS_INFO_NAMED(RCM_LOGGER, "    model: %p, spheres: %s", spheres_state.model, to_string(spheres_state.spheres).c_str());
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Dirty Voxels States: %zu", m_dirty_voxels_states.size());
    ROS_INFO_NAMED(RCM_LOGGER, "  Voxels States: [%p, %p]", m_voxels_states.data(), m_voxels_states.data() + m_voxels_states.size());
    for (const auto& voxels_state : m_voxels_states) {
        ROS_INFO_NAMED(RCM_LOGGER, "    model: %p, voxels: %zu", voxels_state.model, voxels_state.voxels.size());
    }
    ROS_INFO_NAMED(RCM_LOGGER, "  Group States: [%p, %p]", m_group_states.data(), m_group_states.data() + m_group_states.size());
    for (const auto& group_state : m_group_states) {
        ROS_INFO_NAMED(RCM_LOGGER, "    model: %p, sphere_indices: %s, voxels_indices: %s", group_state.model, to_string(group_state.sphere_indices).c_str(), to_string(group_state.voxels_indices).c_str());
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

bool RobotCollisionModelImpl::checkCollisionStateReferences() const
{
    // c++14 would make me happier here...wtb generic lambdas :(
    auto within = [](const void* ptr, const void* start, const void* end) {
        return ptr >= start && ptr < end;
    };

    for (const auto& sphere_state : m_sphere_states) {
        if (!within(
                sphere_state.model,
                m_sphere_models.data(),
                m_sphere_models.data() + m_sphere_models.size()))
        {
            return false;
        }

        if (!within(sphere_state.parent_state,
                m_spheres_states.data(),
                m_spheres_states.data() + m_spheres_states.size()))
        {
            return false;
        }
    }

    for (const auto& spheres_state : m_spheres_states) {
        if (!within(
                spheres_state.model,
                m_spheres_models.data(),
                m_spheres_models.data() + m_spheres_models.size()))
        {
            return false;
        }
    }

    for (const auto& voxels_state : m_voxels_states) {
        if (!within(
                voxels_state.model,
                m_voxels_models.data(),
                m_voxels_models.data() + m_voxels_models.size()))
        {
            return false;
        }
    }

    for (const auto& group_state : m_group_states) {
        if (!within(
                group_state.model,
                m_group_models.data(),
                m_group_models.data() + m_group_models.size()))
        {
            return false;
        }

        if (std::any_of(
                group_state.sphere_indices.begin(),
                group_state.sphere_indices.end(),
                [&](int ssidx)
                {
                    return ssidx < 0 || ssidx >= m_sphere_states.size();
                }))
        {
            return false;
        }

        if (std::any_of(
                group_state.voxels_indices.begin(),
                group_state.voxels_indices.end(),
                [&](int vsidx)
                {
                    return vsidx < 0 || vsidx >= m_voxels_states.size();
                }))
        {
            return false;
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

bool RobotCollisionModelImpl::updateSpheresStateToSphereStatesReferences()
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
    for (size_t i = 0; i < m_spheres_models.size(); ++i) {
        // NOTE: careful with the variable shadowing here
        const CollisionSpheresModel& spheres_model = m_spheres_models[i];
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
}

bool RobotCollisionModelImpl::updateVoxelsStateToModelReferences()
{
    assert(m_voxels_models.size() == m_voxels_states.size());
    for (size_t i = 0; i < m_voxels_models.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_voxels_models[i];
              CollisionVoxelsState& voxels_state = m_voxels_states[i];
        voxels_state.model = &voxels_model;
    }
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

bool RobotCollisionModelImpl::updateLinkBodyToSpheresStateReferences()
{
    // assumes that the links/attached bodies do have a corresponding sphere
    // state but that the pointer reference has been invalidated by an increase
    // in the size of the spheres state array
    assert(m_spheres_models.size() == m_spheres_states.size());
    for (size_t i = 0; i < m_spheres_states.size(); ++i) {
        const CollisionSpheresModel& spheres_model = m_spheres_models[i];
        CollisionSpheresState* spheres_state = &m_spheres_states[i];
        if (spheres_model.link_index >= 0) {
            assert(spheres_model.link_index < m_link_spheres_states.size());
            m_link_spheres_states[spheres_model.link_index] = spheres_state;
        }
        else if (spheres_model.body_index >= 0) {
            m_attached_body_spheres_states[spheres_model.body_index] = spheres_state;
        }
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, "Updated per-link and per-body spheres state references");
    return true;
}

bool RobotCollisionModelImpl::updateLinkBodyToVoxelsStateReferences()
{
    // assumes that the links/attached bodies do have a corresponding voxels
    // state but that the pointer reference has been invalidated by an increase
    // in the size of the voxels state array
    assert(m_voxels_states.size() == m_voxels_models.size());
    for (size_t i = 0; i < m_voxels_states.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_voxels_models[i];
        CollisionVoxelsState* voxels_state = &m_voxels_states[i];
        if (voxels_model.link_index >= 0) {
            m_link_voxels_states[voxels_model.link_index] = voxels_state;
        }
        else if (voxels_model.body_index >= 0) {
            m_attached_body_voxels_states[voxels_model.body_index] = voxels_state;
        }
    }
    return true;
}

bool RobotCollisionModelImpl::updateGroupStateToModelReferences()
{
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        const CollisionGroupModel& group_model = m_group_models[i];
              CollisionGroupState& group_state = m_group_states[i];
        group_state.model = &group_model;
    }
}

bool RobotCollisionModelImpl::updateGroupStateReferences()
{
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        const CollisionGroupModel& group_model = m_group_models[i];
        CollisionGroupState& group_state = m_group_states[i];
        updateGroupStateReferences(group_model, group_state);
    }
}

bool RobotCollisionModelImpl::updateGroupStateReferences(
    const CollisionGroupModel& model,
    CollisionGroupState& state)
{
    state.sphere_indices.clear();
    state.voxels_indices.clear();

    for (int lidx : model.link_indices) {
        CollisionSpheresState* spheres_state = m_link_spheres_states[lidx];
        if (spheres_state) {
            for (CollisionSphereState* sphere_state : spheres_state->spheres) {
                int ssidx = std::distance(m_sphere_states.data(), sphere_state);
                state.sphere_indices.push_back(ssidx);
            }
        }
    }

    for (int bidx : model.body_indices) {
        CollisionSpheresState* spheres_state = m_attached_body_spheres_states[bidx];
        if (spheres_state) {
            for (CollisionSphereState* sphere_state : spheres_state->spheres) {
                int ssidx = std::distance(m_sphere_states.data(), sphere_state);
                state.sphere_indices.push_back(ssidx);
            }
        }
    }

    // gather the indices of all link voxels states that do NOT belong to this group
    for (size_t lidx = 0; lidx < m_link_names.size(); ++lidx) {
        if (std::find(
                model.link_indices.begin(),
                model.link_indices.end(),
                (int)lidx) ==
            model.link_indices.end())
        {
            CollisionVoxelsState* voxels_state = m_link_voxels_states[lidx];
            if (voxels_state) {
                int vsidx = std::distance(m_voxels_states.data(), voxels_state);
                state.voxels_indices.push_back(vsidx);
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

    return true;
}

void RobotCollisionModelImpl::clear()
{
    m_model_frame = "";
    m_jvar_names.clear();
    m_link_names.clear();
    m_jvar_name_to_index.clear();
    m_link_name_to_index.clear();
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
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find collision elements of link '%s'", link->name.c_str());
        return false;
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

const std::vector<CollisionGroupModel>& RobotCollisionModel::groups() const
{
    return m_impl->groups();
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

const std::vector<int>& RobotCollisionModel::groupSphereStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupSphereStateIndices(group_name);
}

const std::vector<int>& RobotCollisionModel::groupSphereStateIndices(
    int gidx) const
{
    return m_impl->groupSphereStateIndices(gidx);
}

const std::vector<int>& RobotCollisionModel::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupOutsideVoxelsStateIndices(group_name);
}

const std::vector<int>& RobotCollisionModel::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    return m_impl->groupOutsideVoxelsStateIndices(gidx);
}

const Eigen::Affine3d& RobotCollisionModel::worldToModelTransform() const
{
    return m_impl->worldToModelTransform();
}

bool RobotCollisionModel::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    return m_impl->setWorldToModelTransform(transform);
}

const std::vector<double>& RobotCollisionModel::jointPositions() const
{
    return m_impl->jointPositions();
}

const Affine3dVector& RobotCollisionModel::linkTransforms() const
{
    return m_impl->linkTransforms();
}

double RobotCollisionModel::jointPosition(const std::string& joint_name) const
{
    return m_impl->jointPosition(joint_name);
}

double RobotCollisionModel::jointPosition(int jidx) const
{
    return m_impl->jointPosition(jidx);
}

bool RobotCollisionModel::setJointPosition(
    const std::string& name,
    double position)
{
    return m_impl->setJointPosition(name, position);
}

bool RobotCollisionModel::setJointPosition(int jidx, double position)
{
    return m_impl->setJointPosition(jidx, position);
}

const Eigen::Affine3d& RobotCollisionModel::linkTransform(
    const std::string& link_name) const
{
    return m_impl->linkTransform(link_name);
}

const Eigen::Affine3d& RobotCollisionModel::linkTransform(int lidx) const
{
    return m_impl->linkTransform(lidx);
}

bool RobotCollisionModel::linkTransformDirty(const std::string& link_name) const
{
    return m_impl->linkTransformDirty(link_name);
}

bool RobotCollisionModel::linkTransformDirty(int lidx) const
{
    return m_impl->linkTransformDirty(lidx);
}

bool RobotCollisionModel::updateLinkTransforms()
{
    return m_impl->updateLinkTransforms();
}

bool RobotCollisionModel::updateLinkTransform(int lidx)
{
    return m_impl->updateLinkTransform(lidx);
}

bool RobotCollisionModel::updateLinkTransform(const std::string& link_name)
{
    return m_impl->updateLinkTransform(link_name);
}

const Eigen::Affine3d& RobotCollisionModel::attachedBodyTransform(const std::string& id) const
{
    return m_impl->attachedBodyTransform(id);
}

const Eigen::Affine3d& RobotCollisionModel::attachedBodyTransform(int abidx) const
{
    return m_impl->attachedBodyTransform(abidx);
}

bool RobotCollisionModel::attachedBodyTransformDirty(const std::string& id) const
{
    return m_impl->attachedBodyTransformDirty(id);
}

bool RobotCollisionModel::attachedBodyTransformDirty(int abidx) const
{
    return m_impl->attachedBodyTransformDirty(abidx);
}

bool RobotCollisionModel::updateAttachedBodyTransform(const std::string& id)
{
    return m_impl->updateAttachedBodyTransform(id);
}

bool RobotCollisionModel::updateAttachedBodyTransform(int abidx)
{
    return m_impl->updateAttachedBodyTransform(abidx);
}

const CollisionVoxelsState& RobotCollisionModel::voxelsState(int vsidx) const
{
    return m_impl->voxelsState(vsidx);
}

bool RobotCollisionModel::voxelsStateDirty(int vsidx) const
{
    return m_impl->voxelsStateDirty(vsidx);
}

bool RobotCollisionModel::updateVoxelsStates()
{
    return m_impl->updateVoxelsStates();
}

bool RobotCollisionModel::updateVoxelsState(int vsidx)
{
    return m_impl->updateVoxelsState(vsidx);
}

const CollisionSphereState& RobotCollisionModel::sphereState(int ssidx) const
{
    return m_impl->sphereState(ssidx);
}

bool RobotCollisionModel::sphereStateDirty(int ssidx) const
{
    return m_impl->sphereStateDirty(ssidx);
}

bool RobotCollisionModel::updateSphereStates()
{
    return m_impl->updateSphereStates();
}

bool RobotCollisionModel::updateSphereState(int ssidx)
{
    return m_impl->updateSphereState(ssidx);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getVisualization() const
{
    return m_impl->getVisualization();
}

visualization_msgs::MarkerArray
RobotCollisionModel::getVisualization(const std::string& group_name) const
{
    return m_impl->getVisualization(group_name);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getVisualization(int gidx) const
{
    return m_impl->getVisualization(gidx);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getStaticModelVisualization() const
{
    return m_impl->getStaticModelVisualization();
}

visualization_msgs::MarkerArray
RobotCollisionModel::getStaticModelVisualization(
    const std::string& group_name) const
{
    return m_impl->getStaticModelVisualization(group_name);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getStaticModelVisualization(int gidx) const
{
    return m_impl->getStaticModelVisualization(gidx);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getDynamicModelVisualization() const
{
    return m_impl->getDynamicModelVisualization();
}

visualization_msgs::MarkerArray
RobotCollisionModel::getDynamicModelVisualization(
    const std::string& group_name) const
{
    return m_impl->getDynamicModelVisualization(group_name);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getDynamicModelVisualization(int gidx) const
{
    return m_impl->getDynamicModelVisualization(gidx);
}

} // namespace collision
} // namespace sbpl
