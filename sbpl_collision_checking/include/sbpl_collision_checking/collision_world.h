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

#ifndef sbpl_collision_collision_world_h
#define sbpl_collision_collision_world_h

// standard includes
#include <map>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>
#include <moveit_msgs/CollisionObject.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/MarkerArray.h>

namespace sbpl {
namespace collision {

// TODO: find a home for this
std::vector<int> ConvertToVertexIndices(
    const std::vector<shape_msgs::MeshTriangle>& triangles);

class CollisionWorld
{
public:

    typedef collision_detection::World::Object Object;
    typedef collision_detection::World::ObjectPtr ObjectPtr;
    typedef collision_detection::World::ObjectConstPtr ObjectConstPtr;

    CollisionWorld(OccupancyGrid* grid);

    bool insertObject(const ObjectConstPtr& object);
    bool removeObject(const ObjectConstPtr& object);
    bool removeObject(const std::string& object_name);
    bool moveShapes(const ObjectConstPtr& object);
    bool insertShapes(const ObjectConstPtr& object);
    bool removeShapes(const ObjectConstPtr& object);

    bool processCollisionObject(const moveit_msgs::CollisionObject& object);

    /// \brief Reset the underlying occupancy grid.
    ///
    /// Resets the CollisionWorld by clearing the underlying occupancy grid and
    /// revoxelizing all of the managed objects.
    void reset();

    visualization_msgs::MarkerArray getCollisionObjectsVisualization() const;
    visualization_msgs::MarkerArray getCollisionObjectVoxelsVisualization() const;

private:

    OccupancyGrid* m_grid;

    // set of collision objects
    std::map<std::string, ObjectConstPtr> m_object_map;

    // voxelization of objects in the grid reference frame
    typedef std::vector<Eigen::Vector3d> VoxelList;
    std::map<std::string, std::vector<VoxelList>> m_object_voxel_map;

    ////////////////////
    // Generic Shapes //
    ////////////////////

    bool haveObject(const std::string& name) const;

    bool checkObjectInsert(const Object& object) const;
    bool checkObjectRemove(const Object& object) const;
    bool checkObjectRemove(const std::string& object_name) const;
    bool checkObjectMoveShape(const Object& object) const;
    bool checkObjectInsertShape(const Object& object) const;
    bool checkObjectRemoveShape(const Object& object) const;

    bool voxelizeObject(
        const Object& object,
        std::vector<std::vector<Eigen::Vector3d>>& all_voxels);
    bool voxelizeShape(
        const shapes::Shape& shape,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);

    // voxelize primitive shapes; functions may overwrite the output voxels
    bool voxelizeSphere(
        const shapes::Sphere& sphere,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeCylinder(
        const shapes::Cylinder& cylinder,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeCone(
        const shapes::Cone& cone,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeBox(
        const shapes::Box& box,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizePlane(
        const shapes::Plane& plane,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeMesh(
        const shapes::Mesh& mesh,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeOcTree(
        const shapes::OcTree& octree,
        const Eigen::Affine3d& pose,
        std::vector<Eigen::Vector3d>& voxels);

    ///////////////////////
    // Collision Objects //
    ///////////////////////

    ObjectConstPtr convertCollisionObjectToObject(
        const moveit_msgs::CollisionObject& object) const;

    // return whether or not to accept an incoming collision object
    bool checkCollisionObjectAdd(
        const moveit_msgs::CollisionObject& object) const;
    bool checkCollisionObjectRemove(
        const moveit_msgs::CollisionObject& object) const;
    bool checkCollisionObjectAppend(
        const moveit_msgs::CollisionObject& object) const;
    bool checkCollisionObjectMove(
        const moveit_msgs::CollisionObject& object) const;

    bool addCollisionObject(const moveit_msgs::CollisionObject& object);
    bool removeCollisionObject(const moveit_msgs::CollisionObject& object);
    bool appendCollisionObject(const moveit_msgs::CollisionObject& object);
    bool moveCollisionObject(const moveit_msgs::CollisionObject& object);

    void removeAllCollisionObjects();

    bool voxelizeCollisionObject(
        const moveit_msgs::CollisionObject& object,
        std::vector<std::vector<Eigen::Vector3d>>& all_voxels);

    // voxelize primitive shapes; functions must append to the output voxels
    bool voxelizeSolidPrimitive(
        const shape_msgs::SolidPrimitive& prim,
        const geometry_msgs::Pose& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeBox(
        const shape_msgs::SolidPrimitive& box,
        const geometry_msgs::Pose& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeSphere(
        const shape_msgs::SolidPrimitive& sphere,
        const geometry_msgs::Pose& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeCylinder(
        const shape_msgs::SolidPrimitive& cylinder,
        const geometry_msgs::Pose& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeCone(
        const shape_msgs::SolidPrimitive& cone,
        const geometry_msgs::Pose& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizeMesh(
        const shape_msgs::Mesh& mesh,
        const geometry_msgs::Pose& pose,
        std::vector<Eigen::Vector3d>& voxels);
    bool voxelizePlane(
        const shape_msgs::Plane& plane,
        const geometry_msgs::Pose& pose,
        std::vector<Eigen::Vector3d>& voxels);

    ///////////////////
    // Visualization //
    ///////////////////

    void getAllCollisionObjectVoxels(
        std::vector<geometry_msgs::Point>& points) const;

    visualization_msgs::MarkerArray getWorldObjectMarkerArray(
        const Object& object,
        std::vector<double>& hue,
        const std::string& ns,
        int id) const;
};

} // namespace collision
} // namespace sbpl

#endif
