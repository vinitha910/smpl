////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
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

#ifndef sbpl_collision_robot_collision_model_h
#define sbpl_collision_robot_collision_model_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometric_shapes/shapes.h>
#include <urdf_model/model.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/types.h>

namespace sbpl {
namespace collision {

/// \ingroup Collision Model
///@{

typedef Eigen::Affine3d (*JointTransformFunction)(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

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
    int link_index; // -1 if not attached to a link
    int body_index; // -1 if not attached to an attached body
    std::vector<const CollisionSphereModel*> spheres;
};

/// \brief Collision Voxels Model Specification
struct CollisionVoxelsModel
{
    int link_index; // -1 if not attached to a link
    int body_index; // -1 if not attached to an attached body
    double voxel_res;
    std::vector<Eigen::Vector3d> voxels; // in the link frame
};

/// \brief Collision Group Model Specification
struct CollisionGroupModel
{
    std::string name;
    std::vector<int> link_indices;
    std::vector<int> body_indices;
};

/// @}

class RobotCollisionModelImpl;

/// \brief Represents the collision model of the robot used for planning.
class RobotCollisionModel
{
public:

    RobotCollisionModel();
    ~RobotCollisionModel();

    bool init(
        const urdf::ModelInterface& urdf,
        const CollisionModelConfig& config);

    /// \name Robot Model - General Information
    ///@{
    auto   name() const -> const std::string&;
    auto   modelFrame() const -> const std::string&;
    ///@}

    /// \name Robot Model - Variable Information
    ///@{
    size_t jointVarCount() const;
    auto   jointVarNames() const -> const std::vector<std::string>&;

    bool   hasJointVar(const std::string& joint_name) const;
    int    jointVarIndex(const std::string& joint_name) const;
    auto   jointVarName(int jidx) const -> const std::string&;

    bool   jointVarIsContinuous(const std::string& joint_name) const;
    bool   jointVarHasPositionBounds(const std::string& joint_name) const;
    double jointVarMinPosition(const std::string& joint_name) const;
    double jointVarMaxPosition(const std::string& joint_name) const;

    bool   jointVarIsContinuous(int jidx) const;
    bool   jointVarHasPositionBounds(int jidx) const;
    double jointVarMinPosition(int jidx) const;
    double jointVarMaxPosition(int jidx) const;
    ///@}

    /// \name Robot Model - Joint Information
    ///@{
    size_t jointCount() const;

    int    jointParentLinkIndex(int jidx) const;
    int    jointChildLinkIndex(int jidx) const;

    auto   jointOrigin(int jidx) const -> const Eigen::Affine3d&;
    auto   jointAxis(int jidx) const -> const Eigen::Vector3d&;
    auto   jointTransformFn(int jidx) const -> JointTransformFunction;
    ///@}

    /// \name Robot Model - Link Information
    ///@{
    size_t linkCount() const;
    auto   linkNames() const -> const std::vector<std::string>&;

    bool  hasLink(const std::string& link_name) const;
    int   linkIndex(const std::string& link_name) const;
    auto  linkName(int lidx) const -> const std::string&;

    int   linkParentJointIndex(int lidx) const;
    auto  linkChildJointIndices(int lidx) const ->
            const std::vector<int>&;
    ///@}

    /// \name Collision Model
    ///@{
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
    ///@}

private:

    std::unique_ptr<RobotCollisionModelImpl> m_impl;
};

typedef std::shared_ptr<RobotCollisionModel> RobotCollisionModelPtr;
typedef std::shared_ptr<const RobotCollisionModel> RobotCollisionModelConstPtr;

} // namespace collision
} // namespace sbpl

#endif
