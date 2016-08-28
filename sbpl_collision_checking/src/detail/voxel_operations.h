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

#include "../voxel_operations.h"

// standard includes
#include <utility>

// system includes
#include <octomap/octomap.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/voxelize.h>

namespace sbpl {
namespace collision {

std::vector<int> ConvertToVertexIndices(
    const std::vector<shape_msgs::MeshTriangle>& triangles);

template <typename OutputIt>
bool VoxelizeObject(
    const Object& object,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    for (size_t i = 0; i < object.shapes_.size(); ++i) {
        const shapes::ShapeConstPtr& shape = object.shapes_[i];
        const Eigen::Affine3d& pose = object.shape_poses_[i];
        if (!VoxelizeShape(*shape, pose, res, go, ofirst)) {
            return false;
        }
    }

    return true;
}

template <typename OutputIt>
bool VoxelizeShape(
    const shapes::Shape& shape,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    switch (shape.type) {
    case shapes::SPHERE: {
        const shapes::Sphere* sphere =
                dynamic_cast<const shapes::Sphere*>(&shape);
        if (!sphere) {
            return false;
        }
        return VoxelizeSphere(*sphere, pose, res, go, ofirst);
    }   break;
    case shapes::CYLINDER: {
        const shapes::Cylinder* cylinder =
                dynamic_cast<const shapes::Cylinder*>(&shape);
        if (!cylinder) {
            return false;
        }
        return VoxelizeCylinder(*cylinder, pose, res, go, ofirst);
    }   break;
    case shapes::CONE: {
        const shapes::Cone* cone = dynamic_cast<const shapes::Cone*>(&shape);
        if (!cone) {
            return false;
        }
        return VoxelizeCone(*cone, pose, res, go, ofirst);
    }   break;
    case shapes::BOX: {
        const shapes::Box* box = dynamic_cast<const shapes::Box*>(&shape);
        if (!box) {
            return false;
        }
        return VoxelizeBox(*box, pose, res, go, ofirst);
    }   break;
    case shapes::PLANE: {
        const shapes::Plane* plane = dynamic_cast<const shapes::Plane*>(&shape);
        if (!plane) {
            return false;
        }
        return VoxelizePlane(*plane, pose, res, go, ofirst);
    }   break;
    case shapes::MESH: {
        const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(&shape);
        if (!mesh) {
            return false;
        }
        return VoxelizeMesh(*mesh, pose, res, go, ofirst);
    }   break;
    case shapes::OCTREE: {
        const shapes::OcTree* octree =
                dynamic_cast<const shapes::OcTree*>(&shape);
        if (!octree) {
            return false;
        }
        return VoxelizeOcTree(*octree, pose, res, go, ofirst);
    }   break;
    }

    return false;
}

template <typename OutputIt>
bool VoxelizeSphere(
    const shapes::Sphere& sphere,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    const double radius = sphere.radius;
    sbpl::VoxelizeSphere(radius, pose, res, go, ofirst, false);
    return true;
}

template <typename OutputIt>
bool VoxelizeCylinder(
    const shapes::Cylinder& cylinder,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    const double height = cylinder.length;
    const double radius = cylinder.radius;
    sbpl::VoxelizeCylinder(radius, height, pose, res, go, ofirst, false);
    return true;
}

template <typename OutputIt>
bool VoxelizeCone(
    const shapes::Cone& cone,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    const double height = cone.length;
    const double radius = cone.radius;
    sbpl::VoxelizeCone(radius, height, pose, res, go, ofirst, false);
    return true;
}

template <typename OutputIt>
bool VoxelizeBox(
    const shapes::Box& box,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    const double length = box.size[0];
    const double width = box.size[1];
    const double height = box.size[2];
    sbpl::VoxelizeBox(length, width, height, pose, res, go, ofirst, false);
    return true;
}

template <typename OutputIt>
bool VoxelizePlane(
    const shapes::Plane& plane,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    ROS_ERROR("Voxelization of planes is currently unsupported");
    return false;
}

template <typename OutputIt>
bool VoxelizeMesh(
    const shapes::Mesh& mesh,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    std::vector<Eigen::Vector3d> vertices(mesh.vertex_count);
    for (unsigned int i = 0; i < mesh.vertex_count; ++i) {
        vertices[i] = Eigen::Vector3d(
                mesh.vertices[3 * i + 0],
                mesh.vertices[3 * i + 1],
                mesh.vertices[3 * i + 2]);
    }
    std::vector<int> indices(mesh.triangles, mesh.triangles + 3 * mesh.triangle_count);
    sbpl::VoxelizeMesh(vertices, indices, pose, res, go, ofirst, false);
    return true;
}

template <typename OutputIt>
bool VoxelizeOcTree(
    const shapes::OcTree& octree,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    OutputIt ofirst)
{
    auto tree = octree.octree;
    for (auto lit = tree->begin_leafs(); lit != tree->end_leafs(); ++lit) {
        if (tree->isNodeOccupied(*lit)) {
            if (lit.getSize() <= res) {
                *ofirst++ = Eigen::Vector3d(lit.getX(), lit.getY(), lit.getZ());
            }
            else {
                double ceil_val = ceil(lit.getSize() / res) * res;
                for (double x = lit.getX() - ceil_val; x < lit.getX() + ceil_val; x += res) {
                for (double y = lit.getY() - ceil_val; y < lit.getY() + ceil_val; y += res) {
                for (double z = lit.getZ() - ceil_val; z < lit.getZ() + ceil_val; z += res) {
                    Eigen::Vector3d pt(x, y, z);
                    pt = pose * pt;
                    *ofirst++ = pt;
                }
                }
                }
            }
        }
    }

    return true;
}

} // namespace collision
} // namespace sbpl
