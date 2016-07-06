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

////////////////////////////////////
// CollisionModelImpl Declaration //
////////////////////////////////////

class CollisionModelImpl
{
public:

    CollisionModelImpl();
    ~CollisionModelImpl();

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

    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto sphereState(int ssidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(int ssidx) const;
    bool updateSphereStates();
    bool updateSphereState(int ssidx);

    auto getVisualization() const -> visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const ->
        visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const -> visualization_msgs::MarkerArray;

private:

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
    hash_map<std::string, int>              m_link_name_to_index;
    ///@}

    /// \name Robot State
    ///@{
    std::vector<double>             m_jvar_positions;          // per joint

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

    ///@}

    bool initRobotModel(const urdf::ModelInterface& urdf);
    bool initRobotState();
    bool initCollisionModel(
        const urdf::ModelInterface& urdf,
        const CollisionModelConfig& config);
    bool initCollisionState();

    bool checkCollisionModelReferences() const;
    bool checkCollisionStateReferences() const;

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
            Eigen::Translation3d(Eigen::Vector3d(
                    jvals[0], jvals[1], jvals[2])) *
            Eigen::AngleAxisd(jvals[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(jvals[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(jvals[3], Eigen::Vector3d::UnitZ());
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

///////////////////////////////////
// CollisionModelImpl Definition //
///////////////////////////////////

CollisionModelImpl::CollisionModelImpl() :
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

CollisionModelImpl::~CollisionModelImpl()
{
}

bool CollisionModelImpl::init(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    return initRobotModel(urdf) &&
            initRobotState() &&
            initCollisionModel(urdf, config) &&
            initCollisionState();
}

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

bool CollisionModelImpl::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    // TODO: equality check?
    m_joint_origins[0] = transform;
    std::fill(m_dirty_link_transforms.begin(), m_dirty_link_transforms.end(), true);
    std::fill(m_dirty_voxels_states.begin(), m_dirty_voxels_states.end(), true);
    std::fill(m_dirty_sphere_states.begin(), m_dirty_sphere_states.end(), true);
    return false;
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

bool CollisionModelImpl::setJointPosition(int jidx, double position)
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, jidx);

    if (m_jvar_positions[jidx] != position) {
        ROS_DEBUG("Setting joint position of joint %d to %0.3f", jidx, position);

        m_jvar_positions[jidx] = position;

        // TODO: cache affected link transforms in a per-joint array?

        std::queue<int> q;
        q.push(m_joint_child_links[jidx]);
        while (!q.empty()) {
            int lidx = q.front();
            q.pop();

            ROS_DEBUG("Dirtying transform to link '%s'", m_link_names[lidx].c_str());

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

bool CollisionModelImpl::updateLinkTransforms()
{
    ROS_DEBUG("Updating all link transforms");
    bool updated = false;
    for (size_t lidx = 0; lidx < m_link_names.size(); ++lidx) {
        updated |= updateLinkTransform(lidx);
    }
    return updated;
}

bool CollisionModelImpl::updateLinkTransform(int lidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    if (!m_dirty_link_transforms[lidx]) {
        return false;
    }

    int pjidx = m_link_parent_joints[lidx];
    int plidx = m_joint_parent_links[pjidx];

    ROS_DEBUG("Updating transform for link '%s'. parent joint = %d, parent link = %d", m_link_names[lidx].c_str(), pjidx, plidx);

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

    ROS_DEBUG(" -> %s", AffineToString(m_link_transforms[lidx]).c_str());

    m_dirty_link_transforms[lidx] = false;
    return true;
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

bool CollisionModelImpl::updateVoxelsStates()
{
    ROS_DEBUG("Updating all voxels states");
    bool updated = false;
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        updated |= updateVoxelsState(vsidx);
    }
    return updated;
}

bool CollisionModelImpl::updateVoxelsState(int vsidx)
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
    std::vector<Eigen::Vector3d> new_voxels;
    new_voxels.resize(state.model->voxels.size());
    for (size_t i = 0; i < state.model->voxels.size(); ++i) {
        new_voxels[i] = T_model_link * state.model->voxels[i];
    }

    state.voxels = std::move(new_voxels);

    m_dirty_voxels_states[vsidx] = false;
    return true;
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

bool CollisionModelImpl::updateSphereStates()
{
    ROS_DEBUG("Updating all sphere positions");
    bool updated = false;
    for (size_t ssidx = 0; ssidx < m_sphere_states.size(); ++ssidx) {
        updated |= updateSphereState(ssidx);
    }
    return updated;
}

bool CollisionModelImpl::updateSphereState(int sidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, sidx);

    if (!m_dirty_sphere_states[sidx]) {
        return false;
    }

    int lidx = m_sphere_states[sidx].parent_state->model->link_index;
    updateLinkTransform(lidx);

    ROS_DEBUG("Updating position of sphere '%s'", m_sphere_states[sidx].model->name.c_str());
    const Eigen::Affine3d& T_model_link = m_link_transforms[lidx];
    m_sphere_states[sidx].pos = T_model_link * m_sphere_states[sidx].model->center;

    m_dirty_sphere_states[sidx] = false;
    return true;
}

visualization_msgs::MarkerArray
CollisionModelImpl::getVisualization() const
{
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < groupCount(); ++i) {
        auto marr = getVisualization(i);
        ma.markers.insert(ma.markers.end(), marr.markers.begin(), marr.markers.end());
    }
    return ma;
}

inline
visualization_msgs::MarkerArray CollisionModelImpl::getVisualization(
    const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    return getVisualization(it->second);
}

visualization_msgs::MarkerArray
CollisionModelImpl::getVisualization(int gidx) const
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

bool CollisionModelImpl::initRobotModel(const urdf::ModelInterface& urdf)
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
//                m_jvar_names.push_back(joint_name);
//                m_jvar_continuous.push_back(false);
//                m_jvar_has_position_bounds.push_back(false);
//                m_jvar_min_positions.push_back(std::numeric_limits<double>::quiet_NaN());
//                m_jvar_max_positions.push_back(std::numeric_limits<double>::quiet_NaN());
//                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

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
                std::string var_name;

                var_name = joint_name + "/x";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/y";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/yaw";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputePlanarJointTransform);
            }   break;
            case urdf::Joint::FLOATING:
            {
                std::string var_name;

                var_name = joint_name + "/x";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/y";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/z";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/roll";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/pitch";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/yaw";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeFloatingJointTransform);
            }   break;
            default:
            {
                ROS_ERROR("Unknown joint type encountered");
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
            d += 6;
        }
        else {
            ROS_ERROR("Unrecognized JointTransformFunction");
            return false;
        }
    }

    ROS_INFO("ComputeFixedJointTransform: %p", ComputeFixedJointTransform);
    ROS_INFO("ComputeRevoluteJointTransform: %p", ComputeRevoluteJointTransform);
    ROS_INFO("ComputeContinuousJointTransform: %p", ComputeContinuousJointTransform);
    ROS_INFO("ComputePrismaticJointTransform: %p", ComputePrismaticJointTransform);
    ROS_INFO("ComputePlanarJointTransform: %p", ComputePlanarJointTransform);
    ROS_INFO("ComputeFloatingJointTransform: %p", ComputeFloatingJointTransform);

    ROS_INFO("Robot Model:");
    ROS_INFO("  Name: %s", m_name.c_str());
    ROS_INFO("  Model Frame: %s", m_model_frame.c_str());
    ROS_INFO("  Joint Variable Names: %s", to_string(m_jvar_names).c_str());
    ROS_INFO("  Joint Variable Continuous: %s", to_string(m_jvar_continuous).c_str());
    ROS_INFO("  Joint Variable Has Position Bounds: %s", to_string(m_jvar_has_position_bounds).c_str());
    ROS_INFO("  Joint Variable Min Positions: %s", to_string(m_jvar_min_positions).c_str());
    ROS_INFO("  Joint Variable Max Positions: %s", to_string(m_jvar_max_positions).c_str());
    ROS_INFO("  Joint Variable Offsets: %s", to_string(m_joint_var_offset).c_str());
    ROS_INFO("  Joint Transform Functions:");
    for (auto p : m_joint_transforms) {
        ROS_INFO("    %p", p);
    }
    ROS_INFO("  Joint Origins:");
    for (const auto& o : m_joint_origins) {
        ROS_INFO("    %s", AffineToString(o).c_str());
    }
    ROS_INFO("  Joint Parent Links: %s", to_string(m_joint_parent_links).c_str());
    ROS_INFO("  Joint Child Links: %s", to_string(m_joint_child_links).c_str());
    ROS_INFO("  Link Names: %s", to_string(m_link_names).c_str());
    ROS_INFO("  Link Parent Joints: %s", to_string(m_link_parent_joints).c_str());
    ROS_INFO("  Link Child Joints: %s", to_string(m_link_children_joints).c_str());

    return true;
}

bool CollisionModelImpl::initRobotState()
{
    assert(!m_joint_origins.empty());
    m_joint_origins[0] = Eigen::Affine3d::Identity();

//    m_jvar_positions.assign(m_jvar_names.size(), 0.0);
    m_dirty_link_transforms.assign(m_link_names.size(), true);
    m_link_transforms.assign(m_link_names.size(), Eigen::Affine3d::Identity());

    ROS_INFO("Robot State:");
    ROS_INFO("  %zu Joint Positions", m_jvar_positions.size());
    ROS_INFO("  %zu Dirty Link Transforms", m_dirty_link_transforms.size());
    ROS_INFO("  %zu Link Transforms", m_link_transforms.size());
    return true;
}

bool CollisionModelImpl::initCollisionModel(
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

        // add references to all spheres
        for (const std::string& sphere_name : spheres_config.spheres) {
            // find the sphere model with this name
            auto sit = std::find_if(m_sphere_models.begin(), m_sphere_models.end(),
                    [&sphere_name](const CollisionSphereModel& sphere)
                    {
                        return sphere.name == sphere_name;
                    });
            if (sit == m_sphere_models.end()) {
                ROS_ERROR("Failed to find sphere '%s' specified in spheres model for link '%s'", sphere_name.c_str(), spheres_config.link_name.c_str());
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
        const double LINK_VOXEL_RESOLUTION = 0.01;
        voxels_model.voxel_res = LINK_VOXEL_RESOLUTION;
        if (!voxelizeLink(urdf, link_name, voxels_model)) {
            ROS_ERROR("Failed to voxelize link '%s'", link_name.c_str());
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

    ROS_INFO("Collision Model:");
    ROS_INFO("  Sphere Models: [%p, %p]", m_sphere_models.data(), m_sphere_models.data() + m_sphere_models.size());
    for (const auto& sphere_model : m_sphere_models) {
        ROS_INFO("    name: %s, center: (%0.3f, %0.3f, %0.3f), radius: %0.3f, priority: %d", sphere_model.name.c_str(), sphere_model.center.x(), sphere_model.center.y(), sphere_model.center.z(), sphere_model.radius, sphere_model.priority);
    }
    ROS_INFO("  Spheres Models: [%p, %p]", m_spheres_models.data(), m_spheres_models.data() + m_spheres_models.size());
    for (const auto& spheres_model : m_spheres_models) {
        ROS_INFO("    link_index: %d, spheres: %s", spheres_model.link_index, to_string(spheres_model.spheres).c_str());
    }
    ROS_INFO("  Voxels Models: [%p, %p]", m_voxels_models.data(), m_voxels_models.data() + m_voxels_models.size());
    for (const auto& voxels_model : m_voxels_models) {
        ROS_INFO("    link_index: %d, voxel_res: %0.3f, voxel count: %zu", voxels_model.link_index, voxels_model.voxel_res, voxels_model.voxels.size());
    }
    ROS_INFO("  Group Models:");
    for (const auto& group_model : m_group_models) {
        ROS_INFO("    name: %s, link_indices: %s", group_model.name.c_str(), to_string(group_model.link_indices).c_str());
    }

    return true;
}

bool CollisionModelImpl::initCollisionState()
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

    for (size_t i = 0; i < m_group_models.size(); ++i) {
        const CollisionGroupModel& group_model = m_group_models[i];
              CollisionGroupState& group_state = m_group_states[i];
        group_state.model = &group_model;

        // gather the indices of all sphere states that belong to this group
        for (int lidx : group_model.link_indices) {
            CollisionSpheresState* spheres_state = m_link_spheres_states[lidx];
            if (spheres_state) {
                for (CollisionSphereState* sphere_state : spheres_state->spheres) {
                    int ssidx = std::distance(m_sphere_states.data(), sphere_state);
                    group_state.sphere_indices.push_back(ssidx);
                }
            }
        }

        // gather the indices of all voxels states that do NOT belong to this group
        for (size_t lidx = 0; lidx < m_link_names.size(); ++lidx) {
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    (int)lidx) ==
                group_model.link_indices.end())
            {
                CollisionVoxelsState* voxels_state = m_link_voxels_states[lidx];
                if (voxels_state) {
                    int vsidx = std::distance(m_voxels_states.data(), voxels_state);
                    group_state.voxels_indices.push_back(vsidx);
                }
            }
        }
    }

    assert(checkCollisionStateReferences());

    ROS_INFO("Collision State:");
    ROS_INFO("  Dirty Sphere States: %zu", m_dirty_sphere_states.size());
    ROS_INFO("  Sphere States: [%p, %p]", m_sphere_states.data(), m_sphere_states.data() + m_sphere_states.size());
    for (const auto& sphere_state : m_sphere_states) {
        ROS_INFO("    model: %p, parent_state: %p, pos: (%0.3f, %0.3f, %0.3f)", sphere_state.model, sphere_state.parent_state, sphere_state.pos.x(), sphere_state.pos.y(), sphere_state.pos.z());
    }
    ROS_INFO("  Spheres States: [%p, %p]", m_spheres_states.data(), m_spheres_states.data() + m_spheres_states.size());
    for (const auto& spheres_state : m_spheres_states) {
        ROS_INFO("    model: %p, spheres: %s", spheres_state.model, to_string(spheres_state.spheres).c_str());
    }
    ROS_INFO("  Dirty Voxels States: %zu", m_dirty_voxels_states.size());
    ROS_INFO("  Voxels States: [%p, %p]", m_voxels_states.data(), m_voxels_states.data() + m_voxels_states.size());
    for (const auto& voxels_state : m_voxels_states) {
        ROS_INFO("    model: %p, voxels: %zu", voxels_state.model, voxels_state.voxels.size());
    }
    ROS_INFO("  Group States: [%p, %p]", m_group_states.data(), m_group_states.data() + m_group_states.size());
    for (const auto& group_state : m_group_states) {
        ROS_INFO("    model: %p, sphere_indices: %s, voxels_indices: %s", group_state.model, to_string(group_state.sphere_indices).c_str(), to_string(group_state.voxels_indices).c_str());
    }

    return true;
}

bool CollisionModelImpl::checkCollisionModelReferences() const
{
    for (const auto& spheres_model : m_spheres_models) {
        if (spheres_model.link_index < 0 ||
            spheres_model.link_index >= m_link_names.size())
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
        if (voxels_model.link_index < 0 ||
            voxels_model.link_index >= m_link_names.size())
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

bool CollisionModelImpl::checkCollisionStateReferences() const
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

void CollisionModelImpl::clear()
{
    m_model_frame = "";
    m_jvar_names.clear();
    m_link_names.clear();
    m_jvar_name_to_index.clear();
    m_link_name_to_index.clear();
}

Eigen::Affine3d CollisionModelImpl::poseUrdfToEigen(const urdf::Pose& p) const
{
    return Eigen::Translation3d(p.position.x, p.position.y, p.position.z) *
            Eigen::Quaterniond(
                    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
}

bool CollisionModelImpl::voxelizeLink(
    const urdf::ModelInterface& urdf,
    const std::string& link_name,
    CollisionVoxelsModel& model) const
{
    auto link = urdf.getLink(link_name);

    if (!link) {
        ROS_ERROR("Failed to find link '%s' in the URDF", link_name.c_str());
        return false;
    }

    if (!link->collision && link->collision_array.empty()) {
        ROS_ERROR("Failed to find collision elements of link '%s'", link->name.c_str());
        return false;
    }

    if (link->collision) {
        if (!voxelizeCollisionElement(
            *link->collision, model.voxel_res, model.voxels))
        {
            ROS_ERROR("Failed to voxelize collision element for link '%s'", link_name.c_str());
            return false;
        }
    }
    else if (!link->collision_array.empty()) {
        for (auto collision : link->collision_array) {
            if (!voxelizeCollisionElement(
                    *collision, model.voxel_res, model.voxels))
            {
                ROS_ERROR("Failed to voxelize collision element for link '%s'", link_name.c_str());
                return false;
            }
        }
    }
    else {
        ROS_ERROR("Hmm");
        return false;
    }

    if (model.voxels.empty()) {
        ROS_WARN("Voxelizing collision elements for link '%s' produced 0 voxels", link_name.c_str());
    }

    return true;
}

bool CollisionModelImpl::voxelizeCollisionElement(
    const urdf::Collision& collision,
    double res,
    std::vector<Eigen::Vector3d>& voxels) const
{
    auto geom = collision.geometry;

    if (!geom) {
        ROS_ERROR("Failed to find geometry for collision element");
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

bool CollisionModelImpl::voxelizeGeometry(
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
            ROS_ERROR("Failed to get mesh from file. (%s)", mesh->filename.c_str());
            return false;
        }

        ROS_DEBUG("mesh: %s  triangles: %zu  vertices: %zu", mesh->filename.c_str(), triangles.size(), mesh_vertices.size());

        std::vector<Eigen::Vector3d> vertices(mesh_vertices.size());
        for (size_t vidx = 0; vidx < mesh_vertices.size(); ++vidx) {
            const geometry_msgs::Point& vertex = mesh_vertices[vidx];
            vertices[vidx] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
        }

        sbpl::VoxelizeMesh(vertices, triangles, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::BOX) {
        urdf::Box* box = (urdf::Box*)&geom;
        ROS_DEBUG("box: { dims: %0.3f, %0.3f, %0.3f }", box->dim.x, box->dim.y, box->dim.z);
        sbpl::VoxelizeBox(box->dim.x, box->dim.y, box->dim.z, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::CYLINDER) {
        urdf::Cylinder* cyl = (urdf::Cylinder*)&geom;
        ROS_DEBUG("cylinder: { radius: %0.3f, length: %0.3f }", cyl->radius, cyl->length);
        sbpl::VoxelizeCylinder(cyl->radius, cyl->length, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::SPHERE) {
        urdf::Sphere* sph = (urdf::Sphere*)&geom;
        ROS_DEBUG("sphere: { radius: %0.3f }", sph->radius);
        sbpl::VoxelizeSphere(sph->radius, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else {
        ROS_ERROR("Unrecognized geometry type for voxelization");
        return false;
    }

    return true;
}

////////////////////////////////////////
// RobotCollisionModel Implementation //
////////////////////////////////////////

RobotCollisionModel::RobotCollisionModel() :
    m_impl(new CollisionModelImpl)
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

} // namespace collision
} // namespace sbpl
