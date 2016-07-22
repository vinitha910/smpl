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

#ifndef sbpl_collision_robot_collision_state_h
#define sbpl_collision_robot_collision_state_h

// standard includes
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_collision_checking/base_collision_states.h>
#include <sbpl_collision_checking/robot_collision_model.h>

namespace sbpl {
namespace collision {

class RobotCollisionStateImpl;

class RobotCollisionState
{
public:

    RobotCollisionState(const RobotCollisionModel* model);
    ~RobotCollisionState();

    const RobotCollisionModel* model() const;

    /// \name Robot State
    ///@{
    auto   worldToModelTransform() const -> const Eigen::Affine3d&;
    bool   setWorldToModelTransform(const Eigen::Affine3d& transform);

    auto   jointVarPositions() const -> const std::vector<double>&;
    auto   linkTransforms() const -> const Affine3dVector&;

    double jointVarPosition(const std::string& var_name) const;
    double jointVarPosition(int vidx) const;

    bool   setJointVarPosition(const std::string& var_name, double position);
    bool   setJointVarPosition(int vidx, double position);

    auto   linkTransform(const std::string& link_name) const ->
            const Eigen::Affine3d&;
    auto   linkTransform(int lidx) const -> const Eigen::Affine3d&;

    bool   linkTransformDirty(const std::string& link_name) const;
    bool   linkTransformDirty(int lidx) const;

    /// \brief Update the transforms of all links in the kinematic tree
    /// \return Whether the transform required updating; all link transforms
    ///         will be up to date in all cases afterwards
    bool   updateLinkTransforms();

    /// \brief Update the transform of a link in the kinematic  tree
    /// \return Whether the transform required updating; the link transform will
    ///         be up to date in all cases afterwards
    bool   updateLinkTransform(int lidx);
    bool   updateLinkTransform(const std::string& link_name);
    ///@}

    /// \name CollisionState
    ///@{
    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto spheresState(int ssidx) const -> const CollisionSpheresState&;

    auto sphereState(const SphereIndex& sidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(const SphereIndex& sidx) const;
    bool updateSphereStates();
    bool updateSphereStates(int ssidx);
    bool updateSphereState(const SphereIndex& sidx);

    /// \brief Return the indices of the collision sphere states belonging to
    ///        this group
    auto   groupSpheresStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupSpheresStateIndices(int gidx) const ->
            const std::vector<int>&;

    /// \brief Return the indices of the collision voxels states NOT belonging
    ///        to this group.
    auto  groupOutsideVoxelsStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;
    ///@}

    auto getVisualization() const -> visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const ->
            visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const -> visualization_msgs::MarkerArray;

private:

    std::unique_ptr<RobotCollisionStateImpl> m_impl;
};

///@}

} // namespace collision
} // namespace sbpl

#endif
