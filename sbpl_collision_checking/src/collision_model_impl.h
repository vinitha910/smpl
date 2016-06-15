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

struct CollisionModelConfig;

struct CollisionGroup
{
    std::vector<int> m_link_indices;
};

struct VoxelCollisionModel
{
    int m_link_index;
};

struct VoxelCollisionState
{
    std::vector<Eigen::Vector3d> voxels; // in the model frame
};

struct SphereCollisionModel
{
    int m_link_index;
    std::string name;
    Eigen::Vector3d center; ///< offset from link center
    double radius;
    int priority;
};

struct SphereCollisionState
{
};

struct SpheresCollisionModel
{
    std::vector<SphereCollisionModel> spheres;
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

    /// \name Collision Model
    ///@{

    bool hasVoxelModel(const std::string& link_name) const;
    bool hasVoxelModel(int lidx) const;
    bool hasSpheresModel(const std::string& link_name) const;
    bool hasSpheresModel(int lidx) const;

    // changed = set all requested joint positions and world-to-model transform
    //
    // if not changed, skip transforms and just check the spheres again
    //
    // for each voxel collision state : voxel collision states that are not part of the active group
    //      if the voxel collision state transform is dirty
    //          remove old voxels
    //          update the transform
    //          insert new voxels
    //
    // for each sphere collision state : sphere collision states that are part of the active group
    //      if the sphere collision state transform is dirty
    //          update the transform
    //      if the sphere is in collision with the collision space
    //          return false;

    // get link names outside of a collision group with voxel models
    // get link indices outside of a collision group with voxel models
    // get link indices

    size_t                          groupCount() const;
    const std::vector<std::string>& groupNames() const;
    bool                            hasGroup(const std::string& group_name) const;

    const std::vector<int>& groupSpheresModelIndices(
        const std::string& group_name) const;
    const std::vector<int>& groupSpheresModelIndices(
        int gidx) const;
    const std::vector<const SpheresCollisionModel*>& groupSpheresModels(
        const std::string& group_name) const;
    const std::vector<const SpheresCollisionModel*>& groupSpheresModels(
        int gidx) const;

    std::vector<std::string> linksWithVoxelModels() const;
    std::vector<std::string> linksWithSphereModels() const;
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
    const std::vector<Eigen::Vector3d>& spherePositions() const;

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

    bool spherePositionDirty(int sidx) const;

    bool updateSpherePositions() const;
    bool updateSpherePositions(const std::string& group_name) const;
    bool updateSpherePositions(int gidx) const;
    bool updateSpherePosition(int sidx) const;
    ///@}

    // goal: iterate over all spheres in the active collision group and check
    // their positions

private:

    /// \name Robot Model
    ///@{
    boost::shared_ptr<urdf::Model>  m_urdf;
    std::string                     m_model_frame;
    std::vector<std::string>        m_joint_names;
    std::vector<std::string>        m_link_names;
    std::map<std::string, int>      m_joint_name_to_index;
    std::map<std::string, int>      m_link_name_to_index;
    ///@}

    /// \name Collision Model
    ///@{
    std::vector<CollisionGroup>         m_collision_groups;
    std::vector<SpheresCollisionModel*> m_spheres_models;       // per-link
    ///@}

    /// \name Robot State
    ///@{
    Eigen::Affine3d                 m_T_world_model;
    std::vector<double>             m_joint_positions;
    std::vector<bool>               m_dirty_link_transforms;    // per-link
    Affine3dVector                  m_link_transforms;          // per-link
    ///@}

    /// \name Collision State
    ///@{
    std::vector<bool>                   m_dirty_voxels_transforms; // per-link
    std::vector<VoxelCollisionModel*>   m_voxel_models;            // per-link
    std::vector<bool>                   m_dirty_sphere_transforms; // per-sphere
    std::vector<Eigen::Vector3d>        m_sphere_transforms;       // per-sphere
    ///@}

    void setReferenceFrame(const std::string& frame);

    bool initURDF(const std::string& urdf_string);
    bool initKdlRobotModel();
    bool initAllGroups(const CollisionModelConfig& config);

    bool decomposeRobotModel();

    bool isDescendantOf(
        const std::string& link_a_name,
        const std::string& link_b_name) const;

    bool jointInfluencesLink(
        const std::string& joint_name,
        const std::string& link_name) const;
};

} // namespace collision
} // namespace sbpl

#endif
