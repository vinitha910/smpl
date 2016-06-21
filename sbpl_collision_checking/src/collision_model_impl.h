////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
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

#ifndef sbpl_collision_collision_model_impl_h
#define sbpl_collision_collision_model_impl_h

// standard includes
#include <stdlib.h>
#include <functional>
#include <string>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <angles/angles.h>
#include <urdf/model.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/robot_collision_model.h>

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

class CollisionModelImpl
{
public:

    CollisionModelImpl();
    ~CollisionModelImpl();

    bool init(
        const std::string& urdf_string,
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
    bool updateSpherePositions();
    bool updateSpherePosition(int ssidx);

    auto getVisualization() const -> visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const ->
        visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const -> visualization_msgs::MarkerArray;

private:

    /// \name Robot Model
    ///@{
    boost::shared_ptr<urdf::Model>          m_urdf;
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

    bool initRobotModel(const std::string& urdf_string);
    bool initRobotState();
    bool initCollisionModel(const CollisionModelConfig& config);
    bool initCollisionState();

    bool checkCollisionModelReferences() const;
    bool checkCollisionStateReferences() const;

    bool isDescendantOf(
        const std::string& link_a_name,
        const std::string& link_b_name) const;

    bool jointInfluencesLink(
        const std::string& joint_name,
        const std::string& link_name) const;

    void clear();

    Eigen::Affine3d poseUrdfToEigen(const urdf::Pose& p) const;

    bool voxelizeLink(
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

} // namespace collision
} // namespace sbpl

#include "detail/collision_model_impl.h"

#endif
