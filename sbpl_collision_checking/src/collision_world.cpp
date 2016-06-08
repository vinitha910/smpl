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

#include <sbpl_collision_checking/collision_world.h>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <leatherman/utils.h>
#include <sbpl_geometry_utils/Voxelizer.h>

namespace sbpl {
namespace collision {

std::vector<int> ConvertToVertexIndices(
    const std::vector<shape_msgs::MeshTriangle>& triangles)
{
    std::vector<int> triangle_indices(3 * triangles.size());
    for (int j = 0; j < triangles.size(); ++j) {
        triangle_indices[3 * j + 0] = triangles[j].vertex_indices[0];
        triangle_indices[3 * j + 1] = triangles[j].vertex_indices[1];
        triangle_indices[3 * j + 2] = triangles[j].vertex_indices[2];
    }
    return triangle_indices;
}

CollisionWorld::CollisionWorld(OccupancyGrid* grid) :
    m_grid(grid),
    m_object_map(),
    m_object_voxel_map()
{
}

bool CollisionWorld::insertObject(const ObjectConstPtr& object)
{
    if (!checkObjectInsert(*object)) {
        ROS_ERROR("Rejecting addition of collision object '%s'", object->id_.c_str());
        return false;
    }

    assert(m_object_voxel_map.find(object->id_) == m_object_voxel_map.end());

    std::vector<std::vector<Eigen::Vector3d>> all_voxels;
    if (!voxelizeObject(*object, all_voxels)) {
        ROS_ERROR("Failed to voxelize object '%s'", object->id_.c_str());
        return false;
    }

    auto vit = m_object_voxel_map.insert(
            std::make_pair(object->id_, std::vector<VoxelList>()));
    vit.first->second = std::move(all_voxels);
    assert(vit.second);

    m_object_map.insert(std::make_pair(object->id_, object));

    for (const auto& voxel_list : vit.first->second) {
        ROS_DEBUG("Adding %zu voxels from collision object '%s' to the distance transform",
                voxel_list.size(), object->id_.c_str());
        m_grid->addPointsToField(voxel_list);
    }

    return true;
}

bool CollisionWorld::removeObject(const ObjectConstPtr& object)
{
    return removeObject(object->id_);
}

bool CollisionWorld::removeObject(const std::string& object_name)
{
    if (!checkObjectRemove(object_name)) {
        ROS_ERROR("Rejecting removal of collision object '%s'", object_name.c_str());
        return false;
    }

    auto oit = m_object_map.find(object_name);
    assert(oit != m_object_map.end());

    auto vit = m_object_voxel_map.find(object_name);
    assert(vit != m_object_voxel_map.end());

    for (const auto& voxel_list : vit->second) {
        ROS_DEBUG("Removing %zu grid cells from the distance transform", voxel_list.size());
        m_grid->removePointsFromField(voxel_list);
    }

    m_object_voxel_map.erase(vit);
    m_object_map.erase(oit);
    return true;
}

bool CollisionWorld::moveShapes(const ObjectConstPtr& object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

bool CollisionWorld::insertShapes(const ObjectConstPtr& object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

bool CollisionWorld::removeShapes(const ObjectConstPtr& object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

bool CollisionWorld::processCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    if (object.operation == moveit_msgs::CollisionObject::ADD) {
        return addCollisionObject(object);
    }
    else if (object.operation == moveit_msgs::CollisionObject::REMOVE) {
        return removeCollisionObject(object);
    }
    else if (object.operation == moveit_msgs::CollisionObject::APPEND) {
        return appendCollisionObject(object);
    }
    else if (object.operation == moveit_msgs::CollisionObject::MOVE) {
        return moveCollisionObject(object);
    }
    else {
        ROS_ERROR("Collision object operation '%d' is not supported", object.operation);
        return false;
    }
}

void CollisionWorld::reset()
{
    m_grid->reset();
    for (const auto& entry : m_object_voxel_map) {
        for (const auto& voxel_list : entry.second) {
            m_grid->addPointsToField(voxel_list);
        }
    }
}

visualization_msgs::MarkerArray
CollisionWorld::getCollisionObjectsVisualization() const
{
    visualization_msgs::MarkerArray ma;
    for (const auto& ent : m_object_map) {
        const ObjectConstPtr& object = ent.second;
        std::vector<double> hue(object->shapes_.size(), 200);
        visualization_msgs::MarkerArray ma1 =
                getWorldObjectMarkerArray(*object, hue, object->id_, 0);
        ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
    }
    return ma;
}

visualization_msgs::MarkerArray
CollisionWorld::getCollisionObjectVoxelsVisualization() const
{
    visualization_msgs::MarkerArray ma;

    std::vector<geometry_msgs::Point> voxels;
    getAllCollisionObjectVoxels(voxels);

    visualization_msgs::Marker marker;
    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = m_grid->getReferenceFrame();
    marker.ns = "collision_object_voxels";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);
    marker.scale.x = m_grid->getResolution();
    marker.scale.y = m_grid->getResolution();
    marker.scale.z = m_grid->getResolution();
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.points.resize(voxels.size());
    for (size_t i = 0; i < voxels.size(); ++i) {
        marker.points[i].x = voxels[i].x;
        marker.points[i].y = voxels[i].y;
        marker.points[i].z = voxels[i].z;
    }
    ma.markers.push_back(marker);

    return ma;
}

bool CollisionWorld::haveObject(const std::string& name) const
{
    return m_object_map.find(name) != m_object_map.end();
}

bool CollisionWorld::checkObjectInsert(const Object& object) const
{
    if (haveObject(object.id_)) {
        ROS_ERROR("Already have collision object '%s'", object.id_.c_str());
        return false;
    }

    if (object.shapes_.size() != object.shape_poses_.size()) {
        ROS_ERROR("Mismatched sizes of shapes and shape poses");
        return false;
    }

    return true;
}

bool CollisionWorld::checkObjectRemove(const Object& object) const
{
    return checkObjectRemove(object.id_);
}

bool CollisionWorld::checkObjectRemove(const std::string& name) const
{
    return haveObject(name);
}

bool CollisionWorld::checkObjectMoveShape(const Object& object) const
{
    return haveObject(object.id_);
}

bool CollisionWorld::checkObjectInsertShape(const Object& object) const
{
    return haveObject(object.id_);
}

bool CollisionWorld::checkObjectRemoveShape(const Object& object) const
{
    return haveObject(object.id_);
}

bool CollisionWorld::voxelizeObject(
    const Object& object,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    for (size_t i = 0; i < object.shapes_.size(); ++i) {
        const shapes::ShapeConstPtr& shape = object.shapes_[i];
        const Eigen::Affine3d& pose = object.shape_poses_[i];
        std::vector<Eigen::Vector3d> voxels;
        if (!voxelizeShape(*shape, pose, voxels)) {
            all_voxels.clear();
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    return true;
}

bool CollisionWorld::voxelizeShape(
    const shapes::Shape& shape,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    switch (shape.type) {
    case shapes::SPHERE: {
        const shapes::Sphere* sphere =
                dynamic_cast<const shapes::Sphere*>(&shape);
        if (!sphere) {
            return false;
        }
        return voxelizeSphere(*sphere, pose, voxels);
    }   break;
    case shapes::CYLINDER: {
        const shapes::Cylinder* cylinder =
                dynamic_cast<const shapes::Cylinder*>(&shape);
        if (!cylinder) {
            return false;
        }
        return voxelizeCylinder(*cylinder, pose, voxels);
    }   break;
    case shapes::CONE: {
        const shapes::Cone* cone = dynamic_cast<const shapes::Cone*>(&shape);
        if (!cone) {
            return false;
        }
        return voxelizeCone(*cone, pose, voxels);
    }   break;
    case shapes::BOX: {
        const shapes::Box* box = dynamic_cast<const shapes::Box*>(&shape);
        if (!box) {
            return false;
        }
        return voxelizeBox(*box, pose, voxels);
    }   break;
    case shapes::PLANE: {
        const shapes::Plane* plane = dynamic_cast<const shapes::Plane*>(&shape);
        if (!plane) {
            return false;
        }
        return voxelizePlane(*plane, pose, voxels);
    }   break;
    case shapes::MESH: {
        const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(&shape);
        if (!mesh) {
            return false;
        }
        return voxelizeMesh(*mesh, pose, voxels);
    }   break;
    case shapes::OCTREE: {
        const shapes::OcTree* octree =
                dynamic_cast<const shapes::OcTree*>(&shape);
        if (!octree) {
            return false;
        }
        return voxelizeOcTree(*octree, pose, voxels);
    }   break;
    }

    return false;
}

bool CollisionWorld::voxelizeSphere(
    const shapes::Sphere& sphere,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double radius = sphere.radius;
    const double res = m_grid->getResolution();
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeSphere(
            radius, pose,
            res, Eigen::Vector3d(ox, oy, oz),
            voxels, false);
    return true;
}

bool CollisionWorld::voxelizeCylinder(
    const shapes::Cylinder& cylinder,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cylinder.length;
    const double radius = cylinder.radius;
    const double res = m_grid->getResolution();
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeCylinder(
            radius, height, pose,
            res, Eigen::Vector3d(ox, oy, oz),
            voxels, false);
    return true;
}

bool CollisionWorld::voxelizeCone(
    const shapes::Cone& cone,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cone.length;
    const double radius = cone.radius;
    const double res = m_grid->getResolution();
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeCone(
        radius, height, pose,
        res, Eigen::Vector3d(ox, oy, oz),
        voxels, false);
    return true;
}

bool CollisionWorld::voxelizeBox(
    const shapes::Box& box,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double length = box.size[0];
    const double width = box.size[1];
    const double height = box.size[2];
    const double res = m_grid->getResolution();
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeBox(
            length, width, height, pose,
            res, Eigen::Vector3d(ox, oy, oz),
            voxels, false);
    return true;
}

bool CollisionWorld::voxelizePlane(
    const shapes::Plane& plane,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    ROS_ERROR("Voxelization of planes is currently unsupported");
    return false;
}

bool CollisionWorld::voxelizeMesh(
    const shapes::Mesh& mesh,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    std::vector<Eigen::Vector3d> vertices(mesh.vertex_count);
    for (unsigned int i = 0; i < mesh.vertex_count; ++i) {
        vertices[i] = Eigen::Vector3d(
                mesh.vertices[3 * i + 0],
                mesh.vertices[3 * i + 1],
                mesh.vertices[3 * i + 2]);
    }
    std::vector<int> indices(mesh.triangles, mesh.triangles + 3 * mesh.triangle_count);
    const double res = m_grid->getResolution();
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeMesh(
            vertices, indices, pose,
            res, Eigen::Vector3d(ox, oy, oz),
            voxels, false);
    return true;
}

bool CollisionWorld::voxelizeOcTree(
    const shapes::OcTree& octree,
    const Eigen::Affine3d& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double gres = m_grid->getResolution();

    auto tree = octree.octree;
    for (auto lit = tree->begin_leafs(); lit != tree->end_leafs(); ++lit) {
        if (tree->isNodeOccupied(*lit)) {
            if (lit.getSize() <= gres) {
                voxels.push_back(Eigen::Vector3d(lit.getX(), lit.getY(), lit.getZ()));
            }
            else {
                double ceil_val = ceil(lit.getSize() / gres) * gres;
                for (double x = lit.getX() - ceil_val; x < lit.getX() + ceil_val; x += gres) {
                for (double y = lit.getY() - ceil_val; y < lit.getY() + ceil_val; y += gres) {
                for (double z = lit.getZ() - ceil_val; z < lit.getZ() + ceil_val; z += gres) {
                    Eigen::Vector3d pt(x, y, z);
                    pt = pose * pt;
                    voxels.push_back(pt);
                }
                }
                }
            }
        }
    }

    return true;
}

CollisionWorld::ObjectConstPtr CollisionWorld::convertCollisionObjectToObject(
    const moveit_msgs::CollisionObject& object) const
{
    ObjectPtr o(new Object(object.id));

    for (size_t pidx = 0; pidx < object.primitives.size(); ++pidx) {
        const shape_msgs::SolidPrimitive& prim = object.primitives[pidx];
        const geometry_msgs::Pose& pose = object.primitive_poses[pidx];

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

    for (size_t midx = 0; midx < object.meshes.size(); ++midx) {
        const shape_msgs::Mesh& mesh = object.meshes[midx];
        const geometry_msgs::Pose& pose = object.mesh_poses[midx];

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

    for (size_t pidx = 0; pidx < object.planes.size(); ++pidx) {
        const shape_msgs::Plane& plane = object.planes[pidx];
        const geometry_msgs::Pose& pose = object.plane_poses[pidx];

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

bool CollisionWorld::checkCollisionObjectAdd(
    const moveit_msgs::CollisionObject& object) const
{
    if (haveObject(object.id)) {
        ROS_ERROR("Already have collision object '%s'", object.id.c_str());
        return false;
    }

    if (object.header.frame_id != m_grid->getReferenceFrame()) {
        ROS_ERROR("Collision object must be specified in the grid reference frame (%s)", m_grid->getReferenceFrame().c_str());
        return false;
    }

    if (object.primitives.size() != object.primitive_poses.size()) {
        ROS_ERROR("Mismatched sizes of primitives and primitive poses");
        return false;
    }

    if (object.meshes.size() != object.mesh_poses.size()) {
        ROS_ERROR("Mismatches sizes of meshes and mesh poses");
        return false;
    }

    // check solid primitive for correct format
    for (const shape_msgs::SolidPrimitive& prim : object.primitives) {
        switch (prim.type) {
        case shape_msgs::SolidPrimitive::BOX:
        {
            if (prim.dimensions.size() != 3) {
                ROS_ERROR("Invalid number of dimensions for box of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 3, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::SPHERE:
        {
            if (prim.dimensions.size() != 1) {
                ROS_ERROR("Invalid number of dimensions for sphere of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 1, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::CYLINDER:
        {
            if (prim.dimensions.size() != 2) {
                ROS_ERROR("Invalid number of dimensions for cylinder of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 2, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::CONE:
        {
            if (prim.dimensions.size() != 2) {
                ROS_ERROR("Invalid number of dimensions for cone of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 2, prim.dimensions.size());
                return false;
            }
        }   break;
        default:
            ROS_ERROR("Unrecognized SolidPrimitive type");
            return false;
        }
    }

    return true;
}

bool CollisionWorld::checkCollisionObjectRemove(
    const moveit_msgs::CollisionObject& object) const
{
    return haveObject(object.id);
}

bool CollisionWorld::checkCollisionObjectAppend(
    const moveit_msgs::CollisionObject& object) const
{
    return m_object_map.find(object.id) != m_object_map.end();
}

bool CollisionWorld::checkCollisionObjectMove(
    const moveit_msgs::CollisionObject& object) const
{
    return m_object_map.find(object.id) != m_object_map.end();
}

bool CollisionWorld::addCollisionObject(const moveit_msgs::CollisionObject& object)
{
    if (!checkCollisionObjectAdd(object)) {
        ROS_ERROR("Rejecting addition of collision object '%s'", object.id.c_str());
        return false;
    }

    ObjectConstPtr op = convertCollisionObjectToObject(object);
    if (!op) {
        ROS_ERROR("Failed to convert collision object to internal representation");
        return false;
    }

    return insertObject(op);
}

bool CollisionWorld::removeCollisionObject(const moveit_msgs::CollisionObject& object)
{
    return removeObject(object.id);
}

bool CollisionWorld::appendCollisionObject(const moveit_msgs::CollisionObject& object)
{
    if (!checkCollisionObjectAppend(object)) {
        ROS_ERROR("Rejecting append to collision object '%s'", object.id.c_str());
        return false;
    }

    // TODO: implement
    ROS_ERROR("appendCollisionObject unimplemented");
    return false;
}

bool CollisionWorld::moveCollisionObject(const moveit_msgs::CollisionObject& object)
{
    if (!checkCollisionObjectMove(object)) {
        ROS_ERROR("Rejecting move of collision object '%s'", object.id.c_str());
        return false;
    }

    // TODO: implement
    ROS_ERROR("moveCollisionObject unimplemented");
    return false;
}

void CollisionWorld::removeAllCollisionObjects()
{
    for (const auto& entry : m_object_map) {
        removeObject(entry.first);
    }
}

bool CollisionWorld::voxelizeCollisionObject(
    const moveit_msgs::CollisionObject& object,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    // gather voxels from all primitives
    for (size_t i = 0; i < object.primitives.size(); ++i) {
        const shape_msgs::SolidPrimitive& prim = object.primitives[i];
        const geometry_msgs::Pose& pose = object.primitive_poses[i];

        std::vector<Eigen::Vector3d> voxels;
        if (!voxelizeSolidPrimitive(prim, pose, voxels)) {
            ROS_ERROR("Failed to voxelize solid primitive of collision object '%s'", object.id.c_str());
            all_voxels.clear();
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    // gather voxels from all meshes
    for (size_t i = 0; i < object.meshes.size(); ++i) {
        const shape_msgs::Mesh& mesh = object.meshes[i];
        const geometry_msgs::Pose& pose = object.mesh_poses[i];
        std::vector<Eigen::Vector3d> voxels;
        if (!voxelizeMesh(mesh, pose, voxels)) {
            ROS_ERROR("Failed to voxelize mesh of collision object '%s'", object.id.c_str());
            all_voxels.clear();
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    // gather voxels from all planes
    for (size_t i = 0; i < object.meshes.size(); ++i) {
        const shape_msgs::Plane& plane = object.planes[i];
        const geometry_msgs::Pose& pose = object.plane_poses[i];
        std::vector<Eigen::Vector3d> voxels;
        if (!voxelizePlane(plane, pose, voxels)) {
            ROS_ERROR("Failed to voxelize plane of collision object '%s'", object.id.c_str());
            all_voxels.clear();
            return false;
        }
    }

    return true;
}

bool CollisionWorld::voxelizeSolidPrimitive(
    const shape_msgs::SolidPrimitive& prim,
    const geometry_msgs::Pose& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    switch (prim.type) {
    case shape_msgs::SolidPrimitive::BOX: {
        if (!voxelizeBox(prim, pose, voxels)) {
            ROS_ERROR("Failed to voxelize box");
            return false;
        }
    }   break;
    case shape_msgs::SolidPrimitive::SPHERE: {
        if (!voxelizeSphere(prim, pose, voxels)) {
            ROS_ERROR("Failed to voxelize sphere");
            return false;
        }
    }   break;
    case shape_msgs::SolidPrimitive::CYLINDER: {
        if (!voxelizeCylinder(prim, pose, voxels)) {
            ROS_ERROR("Failed to voxelize cylinder");
            return false;
        }
    }   break;
    case shape_msgs::SolidPrimitive::CONE: {
        if (!voxelizeCone(prim, pose, voxels)) {
            ROS_ERROR("Failed to voxelize cone");
            return false;
        }
    }   break;
    }

    return true;
}

bool CollisionWorld::voxelizeBox(
    const shape_msgs::SolidPrimitive& box,
    const geometry_msgs::Pose& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double length = box.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    const double width = box.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    const double height = box.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
    const double res = m_grid->getResolution();

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeBox(
            length, width, height, eigen_pose,
            res, Eigen::Vector3d(ox, oy, oz),
            sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool CollisionWorld::voxelizeSphere(
    const shape_msgs::SolidPrimitive& sphere,
    const geometry_msgs::Pose& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double radius =
            sphere.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
    const double res = m_grid->getResolution();

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeSphere(
            radius, eigen_pose,
            res, Eigen::Vector3d(ox, oy, oz),
            sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool CollisionWorld::voxelizeCylinder(
    const shape_msgs::SolidPrimitive& cylinder,
    const geometry_msgs::Pose& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
    const double radius = cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
    const double res = m_grid->getResolution();

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeCylinder(
            radius, height, eigen_pose,
            res, Eigen::Vector3d(ox, oy, oz),
            sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool CollisionWorld::voxelizeCone(
    const shape_msgs::SolidPrimitive& cone,
    const geometry_msgs::Pose& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cone.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT];
    const double radius = cone.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS];
    const double res = m_grid->getResolution();

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeCone(
        radius, height, eigen_pose,
        res, Eigen::Vector3d(ox, oy, oz),
        sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool CollisionWorld::voxelizeMesh(
    const shape_msgs::Mesh& mesh,
    const geometry_msgs::Pose& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    std::vector<Eigen::Vector3d> vertices;
    vertices.resize(mesh.vertices.size());
    for (size_t vidx = 0; vidx < mesh.vertices.size(); ++vidx) {
        const geometry_msgs::Point& vertex = mesh.vertices[vidx];
        vertices[vidx] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
    }

    std::vector<int> indices = ConvertToVertexIndices(mesh.triangles);

    const double res = m_grid->getResolution();

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    double ox, oy, oz;
    m_grid->getOrigin(ox, oy, oz);
    sbpl::VoxelizeMesh(
            vertices, indices, eigen_pose,
            res, Eigen::Vector3d(ox, oy, oz),
            sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool CollisionWorld::voxelizePlane(
    const shape_msgs::Plane& plane,
    const geometry_msgs::Pose& pose,
    std::vector<Eigen::Vector3d>& voxels)
{
    return false;
}

void CollisionWorld::getAllCollisionObjectVoxels(
    std::vector<geometry_msgs::Point>& points) const
{
    for (const auto& entry : m_object_map) {
        const std::string& name = entry.first;
        const ObjectConstPtr& object = entry.second;
        auto vlsit = m_object_voxel_map.find(name);
        assert(vlsit != m_object_voxel_map.end());
        for (size_t sidx = 0; sidx < object->shapes_.size(); ++sidx) {
            const shapes::ShapeConstPtr& shape = object->shapes_[sidx];
            const bool is_collision_object = shape->type != shapes::OCTREE;
            if (is_collision_object) {
                assert(vlsit->second.size() > sidx);
                const VoxelList& vl = vlsit->second[sidx];
                for (const Eigen::Vector3d& voxel : vl) {
                    geometry_msgs::Point point;
                    point.x = voxel.x();
                    point.y = voxel.y();
                    point.z = voxel.z();
                    points.push_back(point);
                }
            }
        }
    }
}

visualization_msgs::MarkerArray CollisionWorld::getWorldObjectMarkerArray(
    const Object& object,
    std::vector<double>& hue,
    const std::string& ns,
    int id) const
{
    visualization_msgs::MarkerArray ma;

    if (object.shapes_.size() != object.shape_poses_.size()) {
        ROS_ERROR("Mismatched sizes of shapes and shape poses");
        return ma;
    }

    std::vector<std::vector<double>> colors;
    colors.resize(hue.size(), std::vector<double>(4, 1.0));
    for (size_t i = 0; i < colors.size(); ++i) {
        leatherman::HSVtoRGB(
                &colors[i][0], &colors[i][1], &colors[i][2], hue[i], 1.0, 1.0);
    }

    for (size_t i = 0; i < object.shapes_.size(); ++i) {
        const shapes::ShapeConstPtr& shape = object.shapes_[i];
        const Eigen::Affine3d& pose = object.shape_poses_[i];

        // fill in type and scale
        visualization_msgs::Marker m;
        if (!shapes::constructMarkerFromShape(shape.get(), m)) {
            ROS_WARN("Failed to construct marker from shape");
        }

        m.header.seq = 0;
        m.header.stamp = ros::Time(0);
        m.header.frame_id = m_grid->getReferenceFrame();
        m.ns = ns;
        m.id = id + i;
        // m.type filled in above
        m.action = visualization_msgs::Marker::ADD;
        tf::poseEigenToMsg(pose, m.pose);
        m.color.r = colors[i][0];
        m.color.g = colors[i][1];
        m.color.b = colors[i][2];
        m.color.a = colors[i][3];
        m.lifetime = ros::Duration(0);
        m.frame_locked = false;

        ma.markers.push_back(m);
    }

    return ma;
}

} // namespace collision
} // namespace sbpl
