////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Maxim Likhachev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the University of Pennsylvania nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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

#include <sbpl_collision_checking/sbpl_collision_space.h>

#include <limits>

#include <angles/angles.h>
#include <leatherman/print.h>
#include <leatherman/viz.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>

namespace sbpl {
namespace collision {

SBPLCollisionSpace::SBPLCollisionSpace(sbpl_arm_planner::OccupancyGrid* grid) :
    model_(),
    grid_(grid),
    padding_(0.0),
    group_name_(),
    object_enclosing_sphere_radius_(0.025), // TODO: ARBITRARY
    inc_(),
    min_limits_(),
    max_limits_(),
    continuous_(),
    spheres_(),
    frames_(),
    known_objects_(),
    object_map_(),
    object_voxel_map_(),
    object_attached_(false),
    attached_object_frame_num_(),
    attached_object_segment_num_(),
    attached_object_chain_num_(),
    attached_object_frame_(),
    object_spheres_(),
    collision_spheres_()
{
}

SBPLCollisionSpace::~SBPLCollisionSpace()
{
}

void SBPLCollisionSpace::setPadding(double padding)
{
    padding_ = padding;
}

bool SBPLCollisionSpace::setPlanningJoints(
    const std::vector<std::string>& joint_names)
{
    if (group_name_.empty()) {
        ROS_ERROR("[cspace] Default group name is not set. Please set it before setting planning joints.");
        return false;
    }

    inc_.resize(joint_names.size(), sbpl::utils::ToRadians(2.0));
    min_limits_.resize(joint_names.size(), 0.0);
    max_limits_.resize(joint_names.size(), 0.0);
    continuous_.resize(joint_names.size(), false);
    for (size_t i = 0; i < joint_names.size(); ++i) {
        bool cont = false;
        if (!model_.getJointLimits(
                group_name_, joint_names[i],
                min_limits_[i], max_limits_[i], cont))
        {
            ROS_ERROR("[cspace] Failed to retrieve joint limits for %s.", joint_names[i].c_str());
            return false;
        }
        continuous_[i] = cont;
    }

    ROS_INFO("[min_limits] %s", leatherman::getString(min_limits_).c_str());
    ROS_INFO("[max_limits] %s", leatherman::getString(max_limits_).c_str());
    ROS_INFO("[continuous] %s", leatherman::getString(continuous_, "yes", "no").c_str());

    // set the order of the planning joints
    model_.setOrderOfJointPositions(joint_names, group_name_);

    return true;
}

bool SBPLCollisionSpace::init(
    const std::string& urdf_string,
    const std::string& group_name,
    const CollisionModelConfig& config,
    const std::vector<std::string>& planning_joints)
{
    ROS_INFO("Initializing collision space for group '%s'", group_name.c_str());

    group_name_ = group_name;

    // initialize the collision model
    if (!model_.init(urdf_string, config)) {
        ROS_ERROR("[cspace] The robot's collision model failed to initialize.");
        return false;
    }

    // choose the group we are planning for
    if (!model_.setDefaultGroup(group_name_)) {
        ROS_ERROR("Failed to set the default group to '%s'.", group_name_.c_str());
        return false;
    }

    // get the collision spheres for the robot
    spheres_ = model_.getDefaultGroupSpheres();

    if (!setPlanningJoints(planning_joints)) {
        ROS_ERROR("Failed to set planning joints");
        return false;
    }

    if (!updateVoxelGroups()) {
        ROS_ERROR("Failed to update voxel groups");
        return false;
    }

    return true;
}

bool SBPLCollisionSpace::checkCollision(
    const std::vector<double>& angles,
    bool verbose,
    bool visualize,
    double& dist)
{
    double dist_temp = 100.0;
    dist = 100.0;
    KDL::Vector v;
    int x, y, z;
    Sphere s;
    bool in_collision = false;
    if (visualize) {
        collision_spheres_.clear();
    }

    // compute foward kinematics
    if (!model_.computeDefaultGroupFK(angles, frames_)) {
        ROS_ERROR("[cspace] Failed to compute foward kinematics.");
        return false;
    }

    // check attached object
    if (object_attached_) {
        for (size_t i = 0; i < object_spheres_.size(); ++i) {
            v = frames_[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] * object_spheres_[i].v;

            grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

            // check bounds
            if (!grid_->isInBounds(x, y, z)) {
                if (verbose) {
                    ROS_INFO("[cspace] Sphere %d %d %d is out of bounds.", x, y, z);
                }
                return false;
            }

            // check for collision with world
            if ((dist_temp = grid_->getDistance(x, y, z)) <= object_spheres_[i].radius) {
                dist = dist_temp;

                if (visualize) {
                    in_collision = true;
                    s = *(spheres_[i]);
                    s.v = v;
                    collision_spheres_.push_back(s);
                }
                else {
                    return false;
                }
            }
            if (dist_temp < dist) {
                dist = dist_temp;
            }
        }
    }

    // check robot model
    for (size_t i = 0; i < spheres_.size(); ++i) {
        v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;

        grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

        // check bounds
        if (!grid_->isInBounds(x, y, z)) {
            if (verbose) {
                ROS_INFO("[cspace] Sphere '%s' with center at {%0.2f %0.2f %0.2f} (%d %d %d) is out of bounds.", spheres_[i]->name.c_str(), v.x(), v.y(), v.z(), x, y, z);
            }
            return false;
        }

        // check for collision with world
        if ((dist_temp = grid_->getDistance(x, y, z)) <= (spheres_[i]->radius + padding_)) {
            dist = dist_temp;
            if (verbose) {
                ROS_INFO("    [sphere %zd] name: %6s  x: %d y: %d z: %d radius: %0.3fm  dist: %0.3fm  *collision*", i, spheres_[i]->name.c_str(), x, y, z, spheres_[i]->radius + padding_, grid_->getDistance(x, y, z));
            }

            if (visualize) {
                in_collision = true;
                s = *(spheres_[i]);
                s.v = v;
                collision_spheres_.push_back(s);
            }
            else {
                return false;
            }
        }

        if (dist_temp < dist) {
            dist = dist_temp;
        }
    }

    if (visualize && in_collision) {
        return false;
    }

    return true;
}

bool SBPLCollisionSpace::updateVoxelGroups()
{
    bool ret = true;
    std::vector<Group*> vg;
    model_.getVoxelGroups(vg);

    for (size_t i = 0; i < vg.size(); ++i) {
        if (!updateVoxelGroup(vg[i])) {
            ROS_ERROR("Failed to update the '%s' voxel group.", vg[i]->getName().c_str());
            ret = false;
        }
    }
    return ret;
}

bool SBPLCollisionSpace::updateVoxelGroup(std::string name)
{
    Group* g = model_.getGroup(name);
    return updateVoxelGroup(g);
}

bool SBPLCollisionSpace::updateVoxelGroup(Group* g)
{
    KDL::Vector v;
    std::vector<double> angles;
    std::vector<std::vector<KDL::Frame> > frames;
    std::vector<Eigen::Vector3d> pts;

    if (!model_.computeGroupFK(angles, g, frames)) {
        ROS_ERROR("[cspace] Failed to compute foward kinematics for group '%s'.", g->getName().c_str());
        return false;
    }

    for (size_t i = 0; i < g->links_.size(); ++i) {
        Link* l = &(g->links_[i]);
        pts.clear();
        pts.resize(l->voxels_.v.size());

        ROS_INFO("Updating Voxel Group %s with %d voxels", g->getName().c_str(), int(l->voxels_.v.size()));
        for (size_t j = 0; j < l->voxels_.v.size(); ++j) {
            v = frames[l->voxels_.kdl_chain][l->voxels_.kdl_segment] * l->voxels_.v[j];
            pts[j].x() = v.x();
            pts[j].y() = v.y();
            pts[j].z() = v.z();
            ROS_DEBUG("[%s] [%d] xyz: %0.2f %0.2f %0.2f", g->getName().c_str(), int(j), pts[j].x(), pts[j].y(), pts[j].z());
        }
        grid_->addPointsToField(pts);
    }
    return true;
}

bool SBPLCollisionSpace::checkPathForCollision(
    const std::vector<double>& start,
    const std::vector<double>& end,
    bool verbose,
    int& path_length,
    int& num_checks,
    double& dist)
{
    int inc_cc = 5;
    double dist_temp = 0;
    std::vector<double> start_norm(start);
    std::vector<double> end_norm(end);
    std::vector<std::vector<double>> path;
    dist = 100;
    num_checks = 0;

    for (size_t i = 0; i < start.size(); ++i) {
        start_norm[i] = angles::normalize_angle(start[i]);
        end_norm[i] = angles::normalize_angle(end[i]);
    }

    if (!interpolatePath(start_norm, end_norm, inc_, path)) {
        path_length = 0;
        ROS_ERROR_ONCE("[cspace] Failed to interpolate the path. It's probably infeasible due to joint limits.");
        ROS_ERROR("[interpolate]  start: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_norm[0], start_norm[1], start_norm[2], start_norm[3], start_norm[4], start_norm[5], start_norm[6]);
        ROS_ERROR("[interpolate]    end: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", end_norm[0], end_norm[1], end_norm[2], end_norm[3], end_norm[4], end_norm[5], end_norm[6]);
        ROS_ERROR("[interpolate]    min: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", min_limits_[0], min_limits_[1], min_limits_[2], min_limits_[3], min_limits_[4], min_limits_[5], min_limits_[6]);
        ROS_ERROR("[interpolate]    max: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", max_limits_[0], max_limits_[1], max_limits_[2], max_limits_[3], max_limits_[4], max_limits_[5], max_limits_[6]);
        return false;
    }

    // for debugging & statistical purposes
    path_length = path.size();

    // try to find collisions that might come later in the path earlier
    if (int(path.size()) > inc_cc) {
        for (int i = 0; i < inc_cc; i++) {
            for (size_t j = i; j < path.size(); j = j + inc_cc) {
                num_checks++;
                if (!checkCollision(path[j], verbose, false, dist_temp)) {
                    dist = dist_temp;
                    return false;
                }

                if (dist_temp < dist) {
                    dist = dist_temp;
                }
            }
        }
    }
    else {
        for (size_t i = 0; i < path.size(); i++) {
            num_checks++;
            if (!checkCollision(path[i], verbose, false, dist_temp)) {
                dist = dist_temp;
                return false;
            }

            if (dist_temp < dist) {
                dist = dist_temp;
            }
        }
    }

    return true;
}

double SBPLCollisionSpace::isValidLineSegment(
    const std::vector<int> a,
    const std::vector<int> b,
    const int radius)
{
    leatherman::bresenham3d_param_t params;
    int nXYZ[3], retvalue = 1;
    double cell_val, min_dist = 100.0;
    leatherman::CELL3V tempcell;
    std::vector<leatherman::CELL3V>* pTestedCells = NULL;

    //iterate through the points on the segment
    leatherman::get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
    do {
        leatherman::get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

        if (!grid_->isInBounds(nXYZ[0], nXYZ[1], nXYZ[2]))
            return 0;

        cell_val = grid_->getDistance(nXYZ[0], nXYZ[1], nXYZ[2]);
        if (cell_val <= radius) {
            if (pTestedCells == NULL)
                return cell_val;   //return 0
            else
                retvalue = 0;
        }

        if (cell_val < min_dist)
            min_dist = cell_val;

        //insert the tested point
        if (pTestedCells) {
            if (cell_val <= radius) {
                tempcell.bIsObstacle = true;
            }
            else {
                tempcell.bIsObstacle = false;
            }
            tempcell.x = nXYZ[0];
            tempcell.y = nXYZ[1];
            tempcell.z = nXYZ[2];
            pTestedCells->push_back(tempcell);
        }
    }
    while (leatherman::get_next_point3d(&params));

    if (retvalue) {
        return min_dist;
    }
    else {
        return 0;
    }
}

bool
SBPLCollisionSpace::getCollisionSpheres(
    const std::vector<double>& angles,
    std::vector<std::vector<double>>& spheres)
{
    std::vector<double> xyzr(4, 0);
    std::vector<std::vector<double> > object;
    KDL::Vector v;

    // compute foward kinematics
    if (!model_.computeDefaultGroupFK(angles, frames_)) {
        ROS_ERROR("[cspace] Failed to compute foward kinematics.");
        return false;
    }

    // robot
    for (size_t i = 0; i < spheres_.size(); ++i) {
        v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;
        xyzr[0] = v.x();
        xyzr[1] = v.y();
        xyzr[2] = v.z();
        xyzr[3] = spheres_[i]->radius;
        spheres.push_back(xyzr);
    }

    // attached object
    if (object_attached_) {
        getAttachedObject(angles, object);
        for (size_t i = 0; i < object.size(); ++i) {
            xyzr[0] = object[i][0];
            xyzr[1] = object[i][1];
            xyzr[2] = object[i][2];
            xyzr[3] = double(object[i][3]);// * grid_->getResolution();
            spheres.push_back(xyzr);
            
        }
    }
    return true;
}

void SBPLCollisionSpace::removeAttachedObject()
{
    object_attached_ = false;
    object_spheres_.clear();
    ROS_DEBUG("[cspace] Removed attached object.");
}

void SBPLCollisionSpace::attachSphere(
    std::string name,
    std::string link,
    geometry_msgs::Pose pose,
    double radius)
{
    object_attached_ = true;
    attached_object_frame_ = link;
    model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);

    object_spheres_.resize(1);
    object_spheres_[0].name = name;
    object_spheres_[0].v.x(pose.position.x);
    object_spheres_[0].v.y(pose.position.y);
    object_spheres_[0].v.z(pose.position.z);
    object_spheres_[0].radius = radius;
    object_spheres_[0].kdl_chain = attached_object_chain_num_;
    object_spheres_[0].kdl_segment = attached_object_segment_num_;

    ROS_DEBUG("[cspace] frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), group_name_.c_str(), attached_object_chain_num_, attached_object_segment_num_);
    ROS_INFO("[cspace] Attached '%s' sphere.  xyz: %0.3f %0.3f %0.3f   radius: %0.3fm", name.c_str(), object_spheres_[0].v.x(), object_spheres_[0].v.y(), object_spheres_[0].v.z(), radius);
}

void SBPLCollisionSpace::attachCylinder(
    std::string link,
    geometry_msgs::Pose pose,
    double radius,
    double length)
{
    object_attached_ = true;
    attached_object_frame_ = link;
    model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);

    // compute end points of cylinder
    KDL::Frame center;
    tf::PoseMsgToKDL(pose, center);
    KDL::Vector top(0.0,0.0,length/2.0), bottom(0.0,0.0,-length/2.0);
    //KDL::Vector top(center.p), bottom(center.p);
    std::vector<KDL::Vector> points;

    //top.data[2] += length / 2.0;
    //bottom.data[2] -= length / 2.0;

    // get spheres
    leatherman::getIntermediatePoints(top, bottom, radius, points);
    int start = object_spheres_.size();
    object_spheres_.resize(object_spheres_.size() + points.size());
    for (size_t i = start; i < start + points.size(); ++i) {
        object_spheres_[i].name = "attached_" + boost::lexical_cast<std::string>(i);
        object_spheres_[i].v = center * points[i - start];
        object_spheres_[i].radius = radius;
        object_spheres_[i].kdl_chain = attached_object_chain_num_;
        object_spheres_[i].kdl_segment = attached_object_segment_num_;
    }

    ROS_INFO("[cspace] [attached_object] Attaching cylinder. pose: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f spheres: %d", pose.position.x, pose.position.y, pose.position.z, radius, length, int(object_spheres_.size()));
    ROS_INFO("[cspace] [attached_object]  frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), group_name_.c_str(), attached_object_chain_num_, attached_object_segment_num_);
    ROS_INFO("[cspace] [attached_object]    top: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm", top.x(), top.y(), top.z(), radius);
    ROS_INFO("[cspace] [attached_object] bottom: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm", bottom.x(), bottom.y(), bottom.z(), radius);
}

void SBPLCollisionSpace::attachCube(
    const std::string& name,
    const std::string& link,
    const geometry_msgs::Pose& pose,
    double x_dim,
    double y_dim,
    double z_dim)
{
    object_attached_ = true;
    std::vector<std::vector<double>> spheres;
    attached_object_frame_ = link;
    if(!model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_)){
      ROS_ERROR("Could not find frame info for attached object frame %s in group name %s", attached_object_frame_.c_str(), group_name_.c_str());
      object_attached_ = false;
      return;
    }

    sbpl::SphereEncloser::encloseBox(x_dim, y_dim, z_dim, object_enclosing_sphere_radius_, spheres);

    if (spheres.size() <= 3) {
        ROS_WARN("[cspace] Attached cube is represented by %d collision spheres. Consider lowering the radius of the spheres used to populate the attached cube. (radius = %0.3fm)", int(spheres.size()), object_enclosing_sphere_radius_);
    }
    int start = object_spheres_.size();
    object_spheres_.resize(start + spheres.size());
    for (size_t i = start; i < start + spheres.size(); ++i) {
        object_spheres_[i].name = name + "_" + boost::lexical_cast<std::string>(i);
        tf::Vector3 sph_in_object_local_(spheres[i - start][0], spheres[i - start][1], spheres[i - start][2]);
        tf::Transform T_sph_offset;
        tf::poseMsgToTF(pose, T_sph_offset);
        tf::Vector3 sph_in_link_local_ = T_sph_offset * sph_in_object_local_;
        object_spheres_[i].v.x(sph_in_link_local_.getX());
        object_spheres_[i].v.y(sph_in_link_local_.getY());
        object_spheres_[i].v.z(sph_in_link_local_.getZ());
        object_spheres_[i].radius = spheres[i - start][3];
        object_spheres_[i].kdl_chain = attached_object_chain_num_;
        object_spheres_[i].kdl_segment = attached_object_segment_num_;
    }
    ROS_INFO("[cspace] Attaching '%s' represented by %d spheres with dimensions: %0.3f %0.3f %0.3f (sphere rad: %.3f) (chain id: %d)(segment id: %d)", name.c_str(), int(spheres.size()), x_dim, y_dim, z_dim, object_enclosing_sphere_radius_, attached_object_chain_num_, attached_object_segment_num_);
}

void SBPLCollisionSpace::attachMesh(
    const std::string& name,
    const std::string& link,
    const geometry_msgs::Pose& pose,
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles)
{
    object_attached_ = true;
    std::vector<std::vector<double>> spheres;
    attached_object_frame_ = link;
    model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);

    sbpl::SphereEncloser::encloseMesh(vertices, triangles, object_enclosing_sphere_radius_, spheres);

    if (spheres.size() <= 3) {
        ROS_WARN("[cspace] Attached mesh is represented by %d collision spheres. Consider lowering the radius of the spheres used to populate the attached mesh more accuratly. (radius = %0.3fm)", int(spheres.size()), object_enclosing_sphere_radius_);
    }

    object_spheres_.resize(spheres.size());
    for (size_t i = 0; i < spheres.size(); ++i) {
        object_spheres_[i].name = name + "_" + boost::lexical_cast<std::string>(i);
        object_spheres_[i].v.x(spheres[i][0]);
        object_spheres_[i].v.y(spheres[i][1]);
        object_spheres_[i].v.z(spheres[i][2]);
        object_spheres_[i].radius = spheres[i][3];
        object_spheres_[i].kdl_chain = attached_object_chain_num_;
        object_spheres_[i].kdl_segment = attached_object_segment_num_;
    }

    ROS_INFO("[cspace] Attaching '%s' represented by %d spheres with %d vertices and %d triangles.", name.c_str(), int(spheres.size()), int(vertices.size()), int(triangles.size()));
}

bool SBPLCollisionSpace::getAttachedObject(
    const std::vector<double>& angles,
    std::vector<std::vector<double>>& xyz)
{
    KDL::Vector v;
    int x, y, z;
    xyz.clear();

    if (!object_attached_) {
        return false;
    }

    // compute foward kinematics
    if (!model_.computeDefaultGroupFK(angles, frames_)) {
        ROS_ERROR("[cspace] Failed to compute foward kinematics.");
        return false;
    }

    xyz.resize(object_spheres_.size(), std::vector<double>(4, 0));
    for (size_t i = 0; i < object_spheres_.size(); ++i) {
        v = frames_[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] * object_spheres_[i].v;

        // snap to grid
        grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);
        grid_->gridToWorld(x, y, z, xyz[i][0], xyz[i][1], xyz[i][2]);

        xyz[i][3] = object_spheres_[i].radius;
    }

    return true;
}

void SBPLCollisionSpace::processCollisionObjectMsg(
    const moveit_msgs::CollisionObject& object)
{
    if (object.id.compare("all") == 0) { // ignoring the operation type
        removeAllCollisionObjects();
    }
    else if (object.operation == moveit_msgs::CollisionObject::ADD) {
        object_map_[object.id] = object;
        addCollisionObject(object);
    }
    else if (object.operation == moveit_msgs::CollisionObject::REMOVE) {
        removeCollisionObject(object);
    }
    else {
        ROS_ERROR("[cspace] Collision object operation '%d' isn't supported yet.", object.operation);
    }
}

void SBPLCollisionSpace::addCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    for (size_t i = 0; i < object.primitives.size(); ++i) {
        if (object.primitives[i].type == shape_msgs::SolidPrimitive::BOX) {
            std::vector<double> dims(3);
            dims[0] = object.primitives[i].dimensions[0];
            dims[1] = object.primitives[i].dimensions[1];
            dims[2] = object.primitives[i].dimensions[2];
            object_voxel_map_[object.id].clear();
            grid_->getOccupiedVoxels(object.primitive_poses[i], dims, object_voxel_map_[object.id]);
        }
        else if (object.primitives[i].type == shape_msgs::SolidPrimitive::SPHERE) {
            // voxelize sphere in object frame
            std::vector<std::vector<double>> voxels;
            sbpl::Voxelizer::voxelizeSphere(object.primitives[i].dimensions[0], object.primitive_poses[i], grid_->getResolution(), voxels, true);
            object_voxel_map_[object.id].clear();
            object_voxel_map_[object.id].resize(voxels.size());

            // transform voxels into parent frame
            Eigen::Affine3d m =
                    Eigen::Affine3d(
                            Eigen::Translation3d(object.primitive_poses[i].position.x, object.primitive_poses[i].position.y, object.primitive_poses[i].position.z) *
                            Eigen::Quaterniond(object.primitive_poses[i].orientation.x, object.primitive_poses[i].orientation.y, object.primitive_poses[i].orientation.z, object.primitive_poses[i].orientation.w).toRotationMatrix());
            for (size_t j = 0; j < voxels.size(); ++j) {
                if (voxels[j].size() < 3) {
                    ROS_ERROR("[cspace] Expected 'voxels' to have length 3.");
                    continue;
                }
                object_voxel_map_[object.id][j].x() = (voxels[j][0]);
                object_voxel_map_[object.id][j].y() = (voxels[j][1]);
                object_voxel_map_[object.id][j].z() = (voxels[j][2]);
                object_voxel_map_[object.id][j] = m.rotation() * object_voxel_map_[object.id][j];
                object_voxel_map_[object.id][j] += m.translation();
            }
        }
        else {
            ROS_WARN("[cspace] Collision objects of type %d are not yet supported.", object.primitives[i].type);
        }
    }

    for (size_t i = 0; i < object.meshes.size(); ++i) {
        std::vector<std::vector<double>> voxels;
        sbpl::Voxelizer::voxelizeMesh(object.meshes[i].vertices, convertToVertexIndices(object.meshes[i].triangles), grid_->getResolution(), voxels, true);
        object_voxel_map_[object.id].clear();
        object_voxel_map_[object.id].resize(voxels.size());

        // transform into the world frame
        Eigen::Affine3d m =
                Eigen::Affine3d(
                        Eigen::Translation3d(object.mesh_poses[i].position.x, object.mesh_poses[i].position.y, object.mesh_poses[i].position.z) *
                        Eigen::Quaterniond(object.mesh_poses[i].orientation.x, object.mesh_poses[i].orientation.y, object.mesh_poses[i].orientation.z, object.mesh_poses[i].orientation.w).toRotationMatrix());
        for (size_t j = 0; j < voxels.size(); ++j) {
            if (voxels[j].size() < 3) {
                ROS_ERROR("[cspace] Expected 'voxels' to have length 3.");
                continue;
            }
            object_voxel_map_[object.id][j].x() = (voxels[j][0]);
            object_voxel_map_[object.id][j].y() = (voxels[j][1]);
            object_voxel_map_[object.id][j].z() = (voxels[j][2]);
            object_voxel_map_[object.id][j] = m.rotation() * object_voxel_map_[object.id][j];
            object_voxel_map_[object.id][j] += m.translation();
        }
    }

    // add this object to list of objects that get added to grid
    bool new_object = true;
    for (size_t i = 0; i < known_objects_.size(); ++i) {
        if (known_objects_[i].compare(object.id) == 0) {
            ROS_DEBUG("[cspace] Received %s collision object again. Not adding.", object.id.c_str());
            new_object = false;
            break;
        }
    }
    if (new_object) {
        known_objects_.push_back(object.id);
    }

    grid_->addPointsToField(object_voxel_map_[object.id]);
}

void SBPLCollisionSpace::removeCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    for (size_t i = 0; i < known_objects_.size(); ++i) {
        if (known_objects_[i].compare(object.id) == 0) {
            known_objects_.erase(known_objects_.begin() + i);
            ROS_INFO("[cspace] Removing %s from list of known collision objects.", object.id.c_str());
        }
    }
}

void SBPLCollisionSpace::removeAllCollisionObjects()
{
    known_objects_.clear();
}

void SBPLCollisionSpace::putCollisionObjectsInGrid()
{
    ROS_WARN("[cspace] Putting %d known objects in grid.", int(known_objects_.size()));
    for (size_t i = 0; i < known_objects_.size(); ++i) {
        grid_->addPointsToField(object_voxel_map_[known_objects_[i]]);
        ROS_INFO("[cspace] [%d] Added %s to grid with %d voxels.", int(i), known_objects_[i].c_str(), int(object_voxel_map_[known_objects_[i]].size()));
    }
}

void SBPLCollisionSpace::getCollisionObjectVoxelPoses(
    std::vector<geometry_msgs::Pose>& points)
{
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;

    for (size_t i = 0; i < known_objects_.size(); ++i) {
        for (size_t j = 0; j < object_voxel_map_[known_objects_[i]].size(); ++j) {
            pose.position.x = object_voxel_map_[known_objects_[i]][j].x();
            pose.position.y = object_voxel_map_[known_objects_[i]][j].y();
            pose.position.z = object_voxel_map_[known_objects_[i]][j].z();
            points.push_back(pose);
        }
    }
}

void SBPLCollisionSpace::setJointPosition(
    const std::string& name,
    double position)
{
    ROS_DEBUG("[cspace] Setting %s with position = %0.3f.", name.c_str(), position);
    model_.setJointPosition(name, position);
}

bool SBPLCollisionSpace::interpolatePath(
    const std::vector<double>& start,
    const std::vector<double>& end,
    const std::vector<double>& inc,
    std::vector<std::vector<double>>& path)
{
    return sbpl::interp::InterpolatePath(start, end, min_limits_, max_limits_, inc, path);
}

bool SBPLCollisionSpace::getClearance(
    const std::vector<double>& angles,
    int num_spheres,
    double& avg_dist,
    double& min_dist)
{
    KDL::Vector v;
    int x, y, z;
    double sum = 0, dist = 100;
    min_dist = 100;

    if (!model_.computeDefaultGroupFK(angles, frames_)) {
        ROS_ERROR("[cspace] Failed to compute foward kinematics.");
        return false;
    }

    if (num_spheres > int(spheres_.size())) {
        num_spheres = spheres_.size();
    }

    for (int i = 0; i < num_spheres; ++i) {
        v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;
        grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);
        dist = grid_->getDistance(x, y, z) - spheres_[i]->radius;

        if (min_dist > dist)
            min_dist = dist;
        sum += dist;
    }

    avg_dist = sum / num_spheres;
    ROS_DEBUG("[cspace]  num_spheres: %d  avg_dist: %2.2f   min_dist: %2.2f", num_spheres, avg_dist, min_dist);
    return true;
}

bool SBPLCollisionSpace::isStateValid(
    const std::vector<double> &angles,
    bool verbose,
    bool visualize,
    double &dist)
{
    return checkCollision(angles, verbose, visualize, dist);
}

bool SBPLCollisionSpace::isStateToStateValid(
    const std::vector<double> &angles0,
    const std::vector<double> &angles1,
    int &path_length,
    int &num_checks,
    double &dist)
{
    return checkPathForCollision(angles0, angles1, false, path_length, num_checks, dist);
}

bool SBPLCollisionSpace::setPlanningScene(
    const moveit_msgs::PlanningScene &scene)
{
    ROS_INFO("Setting the Planning Scene");

    ////////////////////////////////////////////////////////////////////////////////
    // robot state
    ////////////////////////////////////////////////////////////////////////////////

    if (scene.robot_state.joint_state.name.size() != scene.robot_state.joint_state.position.size()) {
        ROS_ERROR("Robot state does not contain correct number of joint positions (Expected: %zd, Actual: %zd)", scene.robot_state.joint_state.name.size(), scene.robot_state.joint_state.position.size());
        return false;
    }

    for (size_t i = 0; i < scene.robot_state.joint_state.name.size(); ++i) {
        model_.setJointPosition(scene.robot_state.joint_state.name[i], scene.robot_state.joint_state.position[i]);
    }

    const std::string& world_frame = scene.world.octomap.header.frame_id;

    if (!model_.setModelToWorldTransform(scene.robot_state, world_frame)) {
        ROS_ERROR("Failed to set the model-to-world transform. The collision model's frame is different from the collision map's frame.");
        return false;
    }

    // reset the distance field (TODO...shouldn't have to reset everytime)
//    grid_->reset();

    ////////////////////////////////////////////////////////////////////////////////
    // attached collision objects
    ////////////////////////////////////////////////////////////////////////////////

    for (const moveit_msgs::AttachedCollisionObject& attached_collision_object :
         scene.robot_state.attached_collision_objects)
    {
        if (!model_.doesLinkExist(attached_collision_object.link_name, group_name_)) {
            ROS_WARN("[cspace] This attached object is not intended for the planning joints of the robot.");
        }
        else if (attached_collision_object.object.operation == moveit_msgs::CollisionObject::ADD) {
            // add object
            ROS_DEBUG("[cspace] Received a message to ADD an object (%s) with %zd shapes.", attached_collision_object.object.id.c_str(), attached_collision_object.object.primitives.size());
            attachObject(attached_collision_object);
        }
        else if (attached_collision_object.object.operation == moveit_msgs::CollisionObject::REMOVE) {
            // remove object
            ROS_DEBUG("[cspace] Removing object (%s) from gripper.", attached_collision_object.object.id.c_str());
            removeAttachedObject();
        }
        else {
            ROS_WARN("Received a collision object with an unknown operation");
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // collision objects
    ////////////////////////////////////////////////////////////////////////////////

    ROS_INFO("Processing %zd collision objects", scene.world.collision_objects.size());
    for (const moveit_msgs::CollisionObject& collision_object : scene.world.collision_objects) {
        object_map_[collision_object.id] = collision_object;
        processCollisionObjectMsg(collision_object);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // todo: octomap
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    // collision map
    ////////////////////////////////////////////////////////////////////////////////

//    if (scene.world.collision_map.header.frame_id != grid_->getReferenceFrame()) {
//        ROS_WARN_ONCE("collision_map_occ is in %s not in %s", scene.world.collision_map.header.frame_id.c_str(), grid_->getReferenceFrame().c_str());
//    }
//
//    if (!scene.world.collision_map.boxes.empty()) {
//        grid_->updateFromCollisionMap(scene.world.collision_map);
//    }

    // self collision
    // todo: move this up if possible
    updateVoxelGroups();
    return true;
}

void SBPLCollisionSpace::attachObject(
    const moveit_msgs::AttachedCollisionObject &obj)
{
    geometry_msgs::PoseStamped pose_in;
    std::string link_name = obj.link_name;
    moveit_msgs::CollisionObject object(obj.object);
    ROS_INFO("Received a collision object message with %zd shape primitives and %zd meshes attached to %s.", object.primitives.size(), object.meshes.size(), link_name.c_str());

    for (size_t i = 0; i < object.primitives.size(); i++) {
        pose_in.header = object.header;
        pose_in.header.stamp = ros::Time();
        pose_in.pose = object.primitive_poses[i];
        ROS_WARN("[cspace] [attach_object] Converted shape from %s (%0.2f %0.2f %0.2f) to %s", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str());

        if (object.primitives[i].type == shape_msgs::SolidPrimitive::SPHERE) {
            ROS_INFO("[cspace] Attaching a '%s' sphere with radius: %0.3fm", object.id.c_str(), object.primitives[i].dimensions[0]);
            attachSphere(object.id, link_name, object.primitive_poses[i], object.primitives[i].dimensions[0]);
        }
        else if (object.primitives[i].type == shape_msgs::SolidPrimitive::CYLINDER) {
            ROS_INFO("[cspace] Attaching a '%s' cylinder with radius: %0.3fm & length %0.3fm", object.id.c_str(), object.primitives[i].dimensions[0], object.primitives[i].dimensions[1]);
            attachCylinder(link_name, object.primitive_poses[i], object.primitives[i].dimensions[1], object.primitives[i].dimensions[0]);
        }
        else if (object.primitives[i].type == shape_msgs::SolidPrimitive::BOX) {
            ROS_INFO("[cspace] Attaching a '%s' cube with dimensions {%0.3fm x %0.3fm x %0.3fm}.", object.id.c_str(), object.primitives[i].dimensions[0], object.primitives[i].dimensions[1], object.primitives[i].dimensions[2]);
            attachCube(object.id, link_name, object.primitive_poses[i], object.primitives[i].dimensions[0], object.primitives[i].dimensions[1], object.primitives[i].dimensions[2]);
        }
        else {
            ROS_WARN("[cspace] Currently attaching objects of type '%d' aren't supported.", object.primitives[i].type);
        }
    }

    for (size_t i = 0; i < object.meshes.size(); i++) {
        pose_in.header = object.header;
        pose_in.header.stamp = ros::Time();
        pose_in.pose = object.mesh_poses[i];

        ROS_WARN("[cspace] [attach_object] Converted shape from %s (%0.2f %0.2f %0.2f) to %s", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str());

        ROS_INFO("[cspace] Attaching a '%s' mesh with %d triangles & %d vertices is NOT supported right now...", object.id.c_str(), int(object.meshes[i].triangles.size() / 3), int(object.meshes[i].vertices.size()));
        attachMesh(object.id, link_name, object.mesh_poses[i], object.meshes[i].vertices, convertToVertexIndices(object.meshes[i].triangles));
    }

    if (!object.planes.empty()) {
        ROS_WARN("[cspace] [attach_object] Attempted to attach object with %zd planes. Ignoring plane components...", object.planes.size());
    }

    ROS_WARN("Attached object has %zd spheres!", object_spheres_.size());
}

visualization_msgs::MarkerArray SBPLCollisionSpace::getVisualization(
    const std::string& type)
{
    visualization_msgs::MarkerArray ma;

    if (type.compare("collision_objects") == 0) {
        visualization_msgs::MarkerArray ma1;
        for (size_t i = 0; i < known_objects_.size(); ++i) {
            if (object_map_.find(known_objects_[i]) != object_map_.end()) {
                std::vector<double> hue(object_map_[known_objects_[i]].primitives.size(), 200);
                ma1 = viz::getCollisionObjectMarkerArray(object_map_[known_objects_[i]], hue, object_map_[known_objects_[i]].id, 0);
                ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
            }
        }
    }
    else if (type.compare("collisions") == 0) {
        std::vector<double> rad(collision_spheres_.size());
        std::vector<std::vector<double>> sph(collision_spheres_.size(), std::vector<double>(3, 0));
        for (size_t i = 0; i < collision_spheres_.size(); ++i) {
            sph[i][0] = collision_spheres_[i].v.x();
            sph[i][1] = collision_spheres_[i].v.y();
            sph[i][2] = collision_spheres_[i].v.z();
            rad[i] = spheres_[i]->radius;
        }
        ma = viz::getSpheresMarkerArray(sph, rad, 10, grid_->getReferenceFrame(), "collision_spheres", 0);
    }
    else if (type.compare("collision_object_voxels") == 0) {
        visualization_msgs::Marker marker;
        std::vector<std::vector<double>> points(1, std::vector<double>(3, 0));
        std::vector<double> color(4, 1);
        color[2] = 0;
        std::vector<geometry_msgs::Pose> vposes;
        getCollisionObjectVoxelPoses(vposes);

        marker.header.seq = 0;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = grid_->getReferenceFrame();
        marker.ns = "collision_object_voxels";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.0);
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 1;
        marker.points.resize(vposes.size());
        for (size_t i = 0; i < vposes.size(); ++i) {
            marker.points[i].x = vposes[i].position.x;
            marker.points[i].y = vposes[i].position.y;
            marker.points[i].z = vposes[i].position.z;
        }
        ma.markers.push_back(marker);
    }
    else {
        ma = grid_->getVisualization(type);
    }

    return ma;
}

visualization_msgs::MarkerArray
SBPLCollisionSpace::getCollisionModelVisualization(
    const std::vector<double> &angles)
{
    std::vector<double> rad;
    std::vector<std::vector<double> > sph;
    visualization_msgs::MarkerArray ma;

    getCollisionSpheres(angles, sph);

    if (sph.empty() || sph[0].size() < 4) {
        return ma;
    }

    rad.resize(sph.size());
    for (size_t i = 0; i < sph.size(); ++i) {
        rad[i] = sph[i][3];
    }

    ma = viz::getSpheresMarkerArray(sph, rad, 90, grid_->getReferenceFrame(), "collision_model", 0);

    // debugging
    //visualization_msgs::MarkerArray ma2 = getMeshModelVisualization("arm", angles);
    //ma.markers.insert(ma.markers.end(), ma2.markers.begin(), ma2.markers.end());

    return ma;
}

visualization_msgs::MarkerArray
SBPLCollisionSpace::getMeshModelVisualization(
    const std::string& group_name,
    const std::vector<double> &angles)
{
    visualization_msgs::MarkerArray ma;
    geometry_msgs::Pose fpose;
    geometry_msgs::PoseStamped lpose, mpose;
    std::string robot_description, mesh_resource;
    Group* g = model_.getGroup(group_name);

    ros::NodeHandle nh;
    if (!nh.getParam("robot_description", robot_description)) {
        ROS_ERROR("Failed to get robot_description from param server.");
        return ma;
    }

    // compute foward kinematics
    if (!model_.computeGroupFK(angles, g, frames_)) {
        ROS_ERROR("[cspace] Failed to compute foward kinematics.");
        return ma;
    }

    // get link mesh_resources
    for (size_t i = 0; i < g->links_.size(); ++i) {
        if (!leatherman::getLinkMesh(robot_description, g->links_[i].root_name_, false, mesh_resource, lpose)) {
            ROS_ERROR("Failed to get mesh for '%s'.", g->links_[i].root_name_.c_str());
            continue;
        }

        ROS_INFO("Got the mesh! (%s)", mesh_resource.c_str());
        // TODO: Has to be a spheres group
        leatherman::msgFromPose(frames_[g->links_[i].spheres_[0].kdl_chain][g->links_[i].spheres_[0].kdl_segment], fpose);
        leatherman::multiply(fpose, lpose.pose, mpose.pose);
        mpose.header.frame_id = "base_link"; //getReferenceFrame();
        ma.markers.push_back(viz::getMeshMarker(mpose, mesh_resource, 180, "robot_model", i));
    }
    return ma;
}

std::vector<int>
SBPLCollisionSpace::convertToVertexIndices(
    const std::vector<shape_msgs::MeshTriangle>& triangles) const
{
    std::vector<int> triangle_indices(3 * triangles.size());
    for (int j = 0; j < triangles.size(); ++j) {
        triangle_indices[3 * j + 0] = triangles[j].vertex_indices[0];
        triangle_indices[3 * j + 1] = triangles[j].vertex_indices[1];
        triangle_indices[3 * j + 2] = triangles[j].vertex_indices[2];
    }
    return triangle_indices;
}

} // namespace collision
} // namespace sbpl
