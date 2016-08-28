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

#ifndef sbpl_collision_voxel_operations_h
#define sbpl_collision_voxel_operations_h

// standard includes
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <sbpl_collision_checking/types.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/Plane.h>
#include <shape_msgs/SolidPrimitive.h>

namespace sbpl {
namespace collision {

/// \name Aggregate Types
///@{

template <typename OutputIt>
bool VoxelizeObject(
    const Object& object,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

///@}

/// \name geometric_shapes::Shape Voxelization
///@{

template <typename OutputIt>
bool VoxelizeShape(
    const shapes::Shape& shape,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

template <typename OutputIt>
bool VoxelizeSphere(
    const shapes::Sphere& sphere,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

template <typename OutputIt>
bool VoxelizeCylinder(
    const shapes::Cylinder& cylinder,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

template <typename OutputIt>
bool VoxelizeCone(
    const shapes::Cone& cone,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

template <typename OutputIt>
bool VoxelizeBox(
    const shapes::Box& box,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

template <typename OutputIt>
bool VoxelizePlane(
    const shapes::Plane& plane,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

template <typename OutputIt>
bool VoxelizeMesh(
    const shapes::Mesh& mesh,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

template <typename OutputIt>
bool VoxelizeOcTree(
    const shapes::OcTree& octree,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst);

///@}

} // namespace collision
} // namespace sbpl

#include "detail/voxel_operations.h"

#endif
