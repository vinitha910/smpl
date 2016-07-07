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

#include <sbpl_collision_checking/types.h>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>

namespace sbpl {
namespace collision {

ObjectConstPtr ConvertCollisionObjectToObject(
    const moveit_msgs::CollisionObject& co)
{
    ObjectPtr o(new Object(co.id));

    for (size_t pidx = 0; pidx < co.primitives.size(); ++pidx) {
        const shape_msgs::SolidPrimitive& prim = co.primitives[pidx];
        const geometry_msgs::Pose& pose = co.primitive_poses[pidx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(prim));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from primitive message");
            return ObjectConstPtr();
        }

        Eigen::Affine3d transform;
        tf::poseMsgToEigen(pose, transform);

        o->shapes_.push_back(sp);
        o->shape_poses_.push_back(transform);
    }

    for (size_t midx = 0; midx < co.meshes.size(); ++midx) {
        const shape_msgs::Mesh& mesh = co.meshes[midx];
        const geometry_msgs::Pose& pose = co.mesh_poses[midx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(mesh));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from mesh message");
            return ObjectConstPtr();
        }

        Eigen::Affine3d transform;
        tf::poseMsgToEigen(pose, transform);

        o->shapes_.push_back(sp);
        o->shape_poses_.push_back(transform);
    }

    for (size_t pidx = 0; pidx < co.planes.size(); ++pidx) {
        const shape_msgs::Plane& plane = co.planes[pidx];
        const geometry_msgs::Pose& pose = co.plane_poses[pidx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(plane));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from plane message");
            return ObjectConstPtr();
        }

        Eigen::Affine3d transform;
        tf::poseMsgToEigen(pose, transform);

        o->shapes_.push_back(sp);
        o->shape_poses_.push_back(transform);
    }

    return ObjectConstPtr(o);
}

} // namespace collision
} // namespace sbpl
