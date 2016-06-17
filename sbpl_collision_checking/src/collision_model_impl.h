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
#include <urdf/model.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

typedef Eigen::aligned_allocator<Eigen::Affine3d> Affine3dAllocator;
typedef std::vector<Eigen::Affine3d, Affine3dAllocator> Affine3dVector;

/// \brief Collision Sphere Model Specification
struct CollisionSphereModel
{
    std::string name;
    Eigen::Vector3d center; ///< offset from link center
    double radius;
    int priority;
};

/// \brief Collision Spheres Model Specification
struct CollisionSpheresModel
{
    std::vector<const CollisionSphereModel*> spheres;
};

/// \brief Collision Voxels Model Specification
struct CollisionVoxelsModel
{
    int link_index;
};

/// \brief Sphere Collision State Specification
struct CollisionSphereState
{
    Eigen::Vector3d pos;
};

struct CollisionSpheresState
{
    std::vector<CollisionSphereState*> states;
};

/// \brief Voxel Collision State Specification
struct CollisionVoxelsState
{
    std::vector<Eigen::Vector3d> voxels; // in the model frame
};

/// \brief Collision Group Model Specification
struct CollisionGroupModel
{
    std::string name;
    std::vector<int> link_indices;
};

/// \brief Collision Group State
struct CollisionGroupState
{
    std::vector<int> sphere_indices; ///< sphere states inside the group
    std::vector<int> voxels_indices; ///< voxels states outside the group
};

class CollisionModelImpl
{
public:

    CollisionModelImpl();
    ~CollisionModelImpl();

    bool init(
        const std::string& urdf_string,
        const CollisionModelConfig& config);

    /// \name Robot Model - General Information
    ///@{
    auto   name() const -> const std::string&;
    auto   modelFrame() const -> const std::string&;
    ///@}

    /// \name Robot Model - Joint Information
    ///@{
    size_t jointCount() const;
    auto   jointNames() const -> const std::vector<std::string>&;

    bool   hasJoint(const std::string& joint_name) const;
    int    jointIndex(const std::string& joint_name) const;
    auto   jointName(int jidx) const -> const std::string&;

    bool   jointIsContinuous(const std::string& joint_name) const;
    bool   jointHasPositionBounds(const std::string& joint_name) const;
    double jointMaxPosition(const std::string& joint_name) const;
    double jointMinPosition(const std::string& joint_name) const;

    bool   jointIsContinuous(int jidx) const;
    bool   jointHasPositionBounds(int jidx) const;
    double jointMinPosition(int jidx) const;
    double jointMaxPosition(int jidx) const;
    ///@}

    /// \name Robot Model - Link Information
    ///@{
    size_t linkCount() const;
    auto   linkNames() const -> const std::vector<std::string>&;

    bool  hasLink(const std::string& link_name) const;
    int   linkIndex(const std::string& link_name) const;
    auto  linkName(int lidx) const -> const std::string&;
    ///@}

    /// \name Collision Model - Collision Spheres Information
    ///@{

    /// \brief Return the number of sphere collision models
    size_t sphereModelCount() const;

    /// \brief Return the sphere collision model for a given sphere collision
    ///        state
    auto   sphereModel(int smidx) const -> const CollisionSphereModel&;

    ///@}

    /// \name Collision Model - Collision Spheres Model Information
    ///@{
    bool hasSpheresModel(const std::string& link_name) const;
    bool hasSpheresModel(int lidx) const;
    ///@}

    /// \name Collision Model - Collision Voxels Model Information
    ///@{

    bool hasVoxelsModel(const std::string& link_name) const;
    bool hasVoxelsModel(int lidx) const;

    /// \brief Return the number of voxel model collision states
    size_t voxelsModelCount() const;

    /// \brief Return the voxel collision model for a given voxel collision state
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    ///@}

    /// \name Collision Model - Group Information
    ///@{

    /// \brief Return the number of collision groups
    size_t groupCount() const;

    /// \brief Return the collision groups
    auto   groups() const -> const std::vector<CollisionGroupModel>&;

    /// \brief Return whether a collision group exists within the model
    bool   hasGroup(const std::string& group_name) const;

    /// \brief Return the index for a collision group
    int    groupIndex(const std::string& group_name) const;

    /// \brief Return the name of a collision group from its index
    auto   groupName(int gidx) const -> const std::string&;

    /// \brief Return the indices of the links belonging to a group
    auto   groupLinkIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupLinkIndices(int gidx) const ->
            const std::vector<int>&;

    /// \brief Return the indices of the collision sphere states belonging to
    ///        this group
    auto   groupSphereStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupSphereStateIndices(int gidx) const ->
            const std::vector<int>&;

    /// \brief Return the indices of the collision voxels states NOT belonging
    ///        to this group.
    auto  groupOutsideVoxelsStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;

    ///@}

    /// \name RobotState
    ///@{
    auto   worldToModelTransform() const -> const Eigen::Affine3d&;
    bool   setWorldToModelTransform(const Eigen::Affine3d& transform);

    auto   jointPositions() const -> const std::vector<double>&;
    auto   linkTransforms() const -> const Affine3dVector&;

    double jointPosition(const std::string& joint_name) const;
    double jointPosition(int jidx) const;

    bool   setJointPosition(const std::string& name, double position);
    bool   setJointPosition(int jidx, double position);

    auto   linkTransform(const std::string& link_name) const ->
            const Eigen::Affine3d&;
    auto   linkTransform(int lidx) const -> const Eigen::Affine3d&;

    bool   linkTransformDirty(const std::string& link_name) const;
    bool   linkTransformDirty(int lidx) const;

    bool   updateLinkTransforms();
    bool   updateLinkTransform(int lidx);
    bool   updateLinkTransform(const std::string& link_name);
    ///@}

    /// \name CollisionState
    ///@{
    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto sphereState(int ssidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(int ssidx) const;
    bool updateSpherePositions();
    bool updateSpherePosition(int ssidx);
    ///@}

private:

    /// \name Robot Model
    ///@{
    boost::shared_ptr<urdf::Model>  m_urdf;
    std::string                     m_name;
    std::string                     m_model_frame;
    std::vector<std::string>        m_joint_names;
    std::vector<double>             m_joint_max_positions;
    std::vector<double>             m_joint_min_positions;
    std::vector<bool>               m_joint_has_position_bounds;
    std::vector<bool>               m_joint_continuous;
    std::vector<std::string>        m_link_names;
    hash_map<std::string, int>      m_joint_name_to_index;
    hash_map<std::string, int>      m_link_name_to_index;
    ///@}

    /// \name Collision Model
    ///@{
    std::vector<CollisionSphereModel>   m_sphere_models;    ///< may be shared between multiple spheres collision models
    std::vector<CollisionSpheresModel>  m_spheres_models;
    std::vector<CollisionVoxelsModel>   m_voxels_models;

    std::vector<CollisionGroupModel>    m_group_models;
    hash_map<std::string, int>          m_group_name_to_index;

    std::vector<CollisionSpheresModel*> m_link_spheres_models;  // per link
    std::vector<CollisionVoxelsModel*>  m_link_voxels_models;   // per link
    ///@}

    /// \name Robot State
    ///@{
    Eigen::Affine3d                 m_T_world_model;

    std::vector<double>             m_joint_positions;          // per joint

    std::vector<bool>               m_dirty_link_transforms;    // per link
    Affine3dVector                  m_link_transforms;          // per link
    ///@}

    /// \name Collision State
    ///@{
    std::vector<bool>                       m_dirty_voxels_states;
    std::vector<CollisionVoxelsState>       m_voxels_states;

    std::vector<bool>                       m_dirty_sphere_states;
    std::vector<CollisionSphereState>       m_sphere_states;

    std::vector<CollisionVoxelsState*>      m_link_voxels_states;    // per link
    std::vector<CollisionSphereState*>      m_link_sphere_states;   // per link

    std::vector<CollisionGroupState>        m_group_states;
    ///@}

    bool initRobotModel(const std::string& urdf_string);
    bool initRobotState();
    bool initCollisionModel(const CollisionModelConfig& config);
    bool initCollisionState();

    bool isDescendantOf(
        const std::string& link_a_name,
        const std::string& link_b_name) const;

    bool jointInfluencesLink(
        const std::string& joint_name,
        const std::string& link_name) const;

    void clear();
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
