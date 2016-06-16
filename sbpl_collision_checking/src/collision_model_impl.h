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

#ifndef sbpl_collision_CollisionModelImpl_h
#define sbpl_collision_CollisionModelImpl_h

// standard includes
#include <map>
#include <string>
#include <vector>

// system includes
#include <kdl/kdl.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/MultiDOFJointState.h>

// project includes
#include <sbpl_collision_checking/group.h>

namespace sbpl {
namespace collision {

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

struct CollisionModelConfig;

/// \brief Collision Group Specification
struct CollisionGroup
{
    std::string name;
    std::vector<int> m_link_indices;
};

/// \brief Voxel Collision Model Specification
struct VoxelCollisionModel
{
    int m_link_index;
};

/// \brief Voxel Collision State Specification
struct VoxelCollisionState
{
    std::vector<Eigen::Vector3d> voxels; // in the model frame
};

/// \brief Sphere Collision Model Specification
struct SphereCollisionModel
{
    std::string name;
    Eigen::Vector3d center; ///< offset from link center
    double radius;
    int priority;
};

/// \brief Sphere Collision State Specification
struct SphereCollisionState
{
    Eigen::Vector3d pos;
};

struct SpheresCollisionModel
{
    std::vector<SphereCollisionModel*> models;
    std::vector<SphereCollisionState*> states;
};

class CollisionModelImpl
{
public:

    typedef Eigen::aligned_allocator<Eigen::Affine3d> Affine3dAllocator;
    typedef std::vector<Eigen::Affine3d, Affine3dAllocator> Affine3dVector;

    CollisionModelImpl();
    ~CollisionModelImpl();

    bool init(
        const std::string& urdf_string,
        const CollisionModelConfig& config);

    /// \name Robot Model
    ///@{
    const std::string& name() const;
    const std::string& modelFrame() const { return m_model_frame; }

    size_t                          jointCount() const;
    const std::vector<std::string>& jointNames() const;

    bool                hasJoint(const std::string& joint_name) const;
    int                 jointIndex(const std::string& joint_name) const;
    const std::string&  jointName(int jidx) const;

    double jointHasPositionBounds(const std::string& joint_name) const;
    double jointMaxPosition(const std::string& joint_name) const;
    double jointMinPosition(const std::string& joint_name) const;
    double jointIsContinuous(const std::string& joint_name) const;

    double jointHasPositionBounds(int jidx) const;
    double jointMinPosition(int jidx) const;
    double jointMaxPosition(int jidx) const;
    double jointIsContinuous(int jidx) const;

    size_t                          linkCount() const;
    const std::vector<std::string>& linkNames() const;

    bool                hasLink(const std::string& link_name) const;
    int                 linkIndex(const std::string& link_name) const;
    const std::string&  linkName(int lidx) const;
    ///@}

    /// \name Collision Model Information
    ///@{
    bool hasVoxelModel(const std::string& link_name) const;
    bool hasVoxelModel(int lidx) const;
    bool hasSpheresModel(const std::string& link_name) const;
    bool hasSpheresModel(int lidx) const;
    ///@}

    /// \name Collision Model Group Information
    ///@{

    /// \brief Return the number of collision groups
    size_t                          groupCount() const;

    /// \brief Return the names of the collision groups
    const std::vector<std::string>& groupNames() const;

    /// \brief Return whether a collision group exists within the model
    bool                            hasGroup(const std::string& group_name) const;

    /// \brief Return the index for a collision group
    int                             groupIndex(const std::string& group_name) const;

    /// \brief Return the name of a collision group from its index
    const std::string&              groupName(int gidx) const;

    /// \brief Return the indices of the links belonging to a group
    const std::vector<int>& groupLinkIndices(const std::string& group_name) const;
    const std::vector<int>& groupLinkIndices(int gidx) const;

    /// \brief Return the indices of the sphere collision states belonging to this group
    const std::vector<int>& groupSphereModelIndices(const std::string& group_name) const;
    const std::vector<int>& groupSphereModelIndices(int gidx) const;

    /// \brief Return the indices of the voxel collision states NOT belonging to this group
    const std::vector<int>& groupOutsideVoxelModelIndices(const std::string& group_name);
    const std::vector<int>& groupOutsideVoxelModelIndices(int gidx) const;

    const std::vector<int>& groupVoxelModelIndices(
        const std::string& group_name) const;
    const std::vector<int>& groupVoxelModelIndices(
        int gidx) const;
    const std::vector<VoxelCollisionModel*> groupVoxelModels(
        const std::string& group_name) const;
    const std::vector<VoxelCollisionModel*> groupVoxelModels(
        int gidx) const;

    const std::vector<const SpheresCollisionModel*>& groupSpheres(
        const std::string& group_name) const;
    const std::vector<const SpheresCollisionModel*>& groupSpheres(
        int gidx) const;

    ///@}

    /// \name Sphere Collision Model Information
    ///@{

    /// \brief Return the number of sphere collision models
    size_t                                   sphereModelCount() const;

    /// \brief Return the sphere collision model for a given sphere collision state
    const SphereCollisionModel&              sphereModel(int sidx) const;

    ///@}

    /// \name Voxel Collision Model Information
    ///@{

    /// \brief Return the number of voxel model collision states
    size_t                      voxelModelCount() const;

    /// \brief Return the voxel collision model for a given voxel collision state
    const VoxelCollisionModel&  voxelModel(int vsidx) const;

    ///@}

    /// \name RobotState
    ///@{
    const Affine3d& worldToModelTransform() const;
    bool setWorldToModelTransform(const Eigen::Affine3d& transform);

    const std::vector<double>&      jointPositions() const;
    const Affine3dVector&           linkTransforms() const;

    double jointPosition(const std::string& joint_name) const;
    double jointPosition(int joint_index) const;

    bool setJointPosition(const std::string& name, double position);
    bool setJointPosition(int jidx, double position);

    const Eigen::Affine3d& linkTransform(const std::string& link_name) const;
    const Eigen::Affine3d& linkTransform(int lidx) const;

    bool linkTransformDirty(const std::string& link_name) const;
    bool linkTransformDirty(int lidx) const;

    bool updateLinkTransforms();
    bool updateLinkTransform(int lidx) const;
    bool updateLinkTransform(const std::string& link_name) const;
    ///@}

    /// \name CollisionState
    ///@{

    bool voxelStateDirty(int vsidx) const;

    const VoxelCollisionState& voxelState(int vsidx) const;

    const Eigen::Vector3d& spherePosition(int sidx) const;

    bool spherePositionDirty(int sidx) const;

    bool updateSpherePositions() const;
    bool updateSpherePositions(const std::string& group_name) const;
    bool updateSpherePositions(int gidx) const;
    bool updateSpherePosition(int sidx) const;
    ///@}

private:

    /// \name Robot Model
    ///@{
    boost::shared_ptr<urdf::Model>  m_urdf;
    std::string                     m_model_frame;
    std::vector<std::string>        m_joint_names;
    std::vector<std::string>        m_link_names;
    hash_map<std::string, int>      m_joint_name_to_index;
    hash_map<std::string, int>      m_link_name_to_index;
    ///@}

    /// \name Collision Model
    ///@{
    std::vector<CollisionGroup>         m_collision_groups;
    hash_map<std::string, int>          m_group_name_to_index;
    std::vector<VoxelCollisionModel>    m_voxel_models;
    std::vector<SphereCollisionModel>   m_sphere_models;
    std::vector<SpheresCollisionModel>  m_spheres_models;

    std::vector<SpheresCollisionModel*> m_link_spheres_models;  // per link
    std::vector<VoxelColisionModel*>    m_link_voxel_models;    // per link
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
    std::vector<bool>                       m_dirty_voxel_states;   // per voxel state
    std::vector<VoxelCollisionState>        m_voxel_states;         // per voxel state
    std::vector<bool>                       m_dirty_sphere_states;  // per sphere state
    std::vector<SphereCollisionState>       m_sphere_states;        // per sphere state
    std::vector<VoxelCollisionState*>       m_link_voxel_states;    // per link
    std::vector<SphereCollisionState*>      m_b
    ///@}

    bool initRobotModel(const std::string& urdf_string);
    bool initRobotState();

    bool initKdlRobotModel();
    bool initAllGroups(const CollisionModelConfig& config);

    bool decomposeRobotModel();

    bool isDescendantOf(
        const std::string& link_a_name,
        const std::string& link_b_name) const;

    bool jointInfluencesLink(
        const std::string& joint_name,
        const std::string& link_name) const;

    void clear();
};

} // namespace collision
} // namespace sbpl

#endif
