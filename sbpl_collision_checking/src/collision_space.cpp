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

#include <sbpl_collision_checking/collision_space.h>

// standard includes
#include <assert.h>
#include <limits>
#include <utility>

// system includes
#include <angles/angles.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <leatherman/bresenham.h>
#include <leatherman/print.h>
#include <leatherman/viz.h>
#include <moveit_msgs/RobotState.h>
#include <sbpl_geometry_utils/SphereEncloser.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>
#include <tf_conversions/tf_kdl.h>

namespace sbpl {
namespace collision {

CollisionSpace::CollisionSpace(OccupancyGrid* grid) :
    m_grid(grid),
    m_world(grid),
    m_model(),
    object_attached_(false),
    attached_object_frame_num_(),
    attached_object_segment_num_(),
    attached_object_chain_num_(),
    attached_object_frame_(),
    object_spheres_(),
    object_enclosing_sphere_radius_(0.025), // TODO: ARBITRARY
    m_group_name(),
    m_group_index(-1),
    m_sphere_indices(),
    m_voxels_indices(),
    m_planning_joint_to_collision_model_indices(),
    m_increments(),
    m_min_limits(),
    m_max_limits(),
    m_continuous(),
    m_acm(),
    m_padding(0.0),
    m_collision_spheres()
{
}

CollisionSpace::~CollisionSpace()
{
}

void CollisionSpace::setPadding(double padding)
{
    m_padding = padding;
}

bool CollisionSpace::setPlanningJoints(
    const std::vector<std::string>& joint_names)
{
    for (const std::string& joint_name : joint_names) {
        if (!m_model.hasJointVar(joint_name)) {
            ROS_ERROR("Joint variable '%s' not found in Robot Collision Model", joint_name.c_str());
            return false;
        }
    }

    // map planning joint indices to collision model indices
    m_planning_joint_to_collision_model_indices.resize(joint_names.size(), -1);

    m_increments.resize(joint_names.size(), sbpl::utils::ToRadians(2.0));
    m_min_limits.resize(joint_names.size(), 0.0);
    m_max_limits.resize(joint_names.size(), 0.0);
    m_continuous.resize(joint_names.size(), false);
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];;
        int jidx = m_model.jointVarIndex(joint_name);

        m_planning_joint_to_collision_model_indices[i] = jidx;
        m_continuous[i] = m_model.jointVarIsContinuous(jidx);
        m_min_limits[i] = m_model.jointVarMinPosition(jidx);
        m_max_limits[i] = m_model.jointVarMaxPosition(jidx);
    }

    ROS_DEBUG("min limits: %s", to_string(m_min_limits).c_str());
    ROS_DEBUG("max limits: %s", to_string(m_max_limits).c_str());
    ROS_DEBUG("continuous: %s", to_string(m_continuous).c_str());

    return true;
}

bool CollisionSpace::init(
    const urdf::ModelInterface& urdf,
    const std::string& group_name,
    const CollisionModelConfig& config,
    const std::vector<std::string>& planning_joints)
{
    ROS_DEBUG("Initializing collision space for group '%s'", group_name.c_str());

    if (!m_model.init(urdf, config)) {
        ROS_ERROR("Failed to initialize the Robot Collision Model");
        return false;
    }

    initAllowedCollisionMatrix(config);
    m_acm.print(std::cout);

    if (!setPlanningJoints(planning_joints)) {
        ROS_ERROR("Failed to set planning joints");
        return false;
    }

    if (!m_model.hasGroup(group_name)) {
        ROS_ERROR("Group '%s' was not found in the Robot Collision Model", group_name.c_str());
        return false;
    }

    m_group_name = group_name;
    m_group_index = m_model.groupIndex(m_group_name);

    m_sphere_indices = m_model.groupSphereStateIndices(m_group_index);

    // sort sphere state indices by priority
    std::sort(m_sphere_indices.begin(), m_sphere_indices.end(),
            [&](int ssidx1, int ssidx2)
            {
                const CollisionSphereModel* sph1 =
                        m_model.sphereState(ssidx1).model;
                const CollisionSphereModel* sph2 =
                        m_model.sphereState(ssidx2).model;
                return sph1->priority < sph2->priority;
            });

    m_voxels_indices = m_model.groupOutsideVoxelsStateIndices(m_group_index);

    return true;
}

bool CollisionSpace::init(
    const std::string& urdf_string,
    const std::string& group_name,
    const CollisionModelConfig& config,
    const std::vector<std::string>& planning_joints)
{
    auto urdf = boost::make_shared<urdf::Model>();
    if (!urdf->initString(urdf_string)) {
        ROS_ERROR("Failed to parse URDF");
        return false;
    }

    return init(*urdf, group_name, config, planning_joints);
}

void CollisionSpace::updateVoxelsStates()
{
    // update voxel groups; gather voxels before updating so as to impose only
    // a single distance field update (TODO: does the distance field recompute
    // with every call to insert/remove/update points?)
    std::vector<Eigen::Vector3d> voxel_removals;
    std::vector<Eigen::Vector3d> voxel_insertions;
    for (int vsidx : m_voxels_indices) {
        if (m_model.voxelsStateDirty(vsidx)) {
            const CollisionVoxelsState& voxels_state = m_model.voxelsState(vsidx);

            // copy over voxels to be removed before updating
            voxel_removals.insert(
                    voxel_removals.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            m_model.updateVoxelsState(vsidx);

            // copy over voxels to be inserted
            voxel_insertions.insert(
                    voxel_insertions.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            ROS_DEBUG("Updating Occupancy Grid with change to Collision Voxels State (%zu displaced)", voxel_removals.size());
        }
    }

    // update occupancy grid with new voxel data
    if (!voxel_removals.empty()) {
        m_grid->removePointsFromField(voxel_removals);
    }
    if (!voxel_insertions.empty()) {
        m_grid->addPointsToField(voxel_insertions);
    }
}

void CollisionSpace::initAllowedCollisionMatrix(
    const CollisionModelConfig& config)
{
    for (size_t i = 0; i < config.spheres.size(); ++i) {
        const std::string& sphere1 = config.spheres[i].name;
        std::string link1;
        if (!findAttachedLink(config, sphere1, link1)) {
            continue;
        }

        if (!m_acm.hasEntry(sphere1)) {
            ROS_INFO("Adding entry '%s' to the ACM", sphere1.c_str());
            m_acm.setEntry(sphere1, false);
        }

        for (size_t j = i + 1; j < config.spheres.size(); ++j) {
            const std::string& sphere2 = config.spheres[j].name;
            std::string link2;
            if (!findAttachedLink(config, sphere2, link2)) {
                continue;
            }

            if (!m_acm.hasEntry(sphere2)) {
                ROS_INFO("Adding entry '%s' to the ACM", sphere2.c_str());
                m_acm.setEntry(sphere2, false);
            }

            if (link1 == link2) {
                ROS_INFO("Spheres '%s' and '%s' attached to the same link...allowing collision", sphere1.c_str(), sphere2.c_str());
                m_acm.setEntry(sphere1, sphere2, true);
                assert(m_acm.hasEntry(sphere1, sphere2));
            }
        }
    }

    // add in additional allowed collisions from config
    std::vector<std::string> config_entries;
    config.acm.getAllEntryNames(config_entries);
    for (size_t i = 0; i < config_entries.size(); ++i) {
        const std::string& entry1 = config_entries[i];
        if (!m_acm.hasEntry(entry1)) {
            ROS_WARN("Configured allowed collision entry '%s' was not found in the collision model", entry1.c_str());
            continue;
        }
        for (size_t j = i; j < config_entries.size(); ++j) {
            const std::string& entry2 = config_entries[j];
            if (!m_acm.hasEntry(entry2)) {
                ROS_WARN("Configured allowed collision entry '%s' was not found in the collision model", entry2.c_str());
                continue;
            }

            if (!config.acm.hasEntry(entry1, entry2)) {
                continue;
            }

            collision_detection::AllowedCollision::Type type;
            config.acm.getEntry(entry1, entry2, type);
            switch (type) {
            case collision_detection::AllowedCollision::NEVER:
                // NOTE: not that it matters, but this disallows config freeing
                // collisions
                break;
            case collision_detection::AllowedCollision::ALWAYS:
                ROS_INFO("Configuration allows spheres '%s' and '%s' to be in collision", entry1.c_str(), entry2.c_str());
                m_acm.setEntry(entry1, entry2, true);
                break;
            case collision_detection::AllowedCollision::CONDITIONAL:
                ROS_WARN("Conditional collisions not supported in SBPL Collision Detection");
                break;
            }
        }
    }
}

bool CollisionSpace::findAttachedLink(
    const CollisionModelConfig& config,
    const std::string& sphere,
    std::string& link_name) const
{
    for (const auto& spheres_model : config.spheres_models) {
        if (std::find(
                spheres_model.spheres.begin(),
                spheres_model.spheres.end(),
                sphere) !=
            spheres_model.spheres.end())
        {
            link_name = spheres_model.link_name;
            return true;
        }
    }

    return false;
}

bool CollisionSpace::checkPathForCollision(
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
        // TODO: normalizing joints that don't need normalized makes me sad
        start_norm[i] = angles::normalize_angle(start[i]);
        end_norm[i] = angles::normalize_angle(end[i]);
    }

    if (!interpolatePath(start_norm, end_norm, path)) {
        path_length = 0;
        ROS_ERROR_ONCE("Failed to interpolate the path. It's probably infeasible due to joint limits.");
        ROS_ERROR("[interpolate]  start: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_norm[0], start_norm[1], start_norm[2], start_norm[3], start_norm[4], start_norm[5], start_norm[6]);
        ROS_ERROR("[interpolate]    end: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", end_norm[0], end_norm[1], end_norm[2], end_norm[3], end_norm[4], end_norm[5], end_norm[6]);
        ROS_ERROR("[interpolate]    min: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", m_min_limits[0], m_min_limits[1], m_min_limits[2], m_min_limits[3], m_min_limits[4], m_min_limits[5], m_min_limits[6]);
        ROS_ERROR("[interpolate]    max: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", m_max_limits[0], m_max_limits[1], m_max_limits[2], m_max_limits[3], m_max_limits[4], m_max_limits[5], m_max_limits[6]);
        return false;
    }

    // for debugging & statistical purposes
    path_length = path.size();

    // TODO: Looks like the idea here is to collision check the path starting at
    // the most coarse resolution (just the endpoints) and increasing the
    // granularity until all points are checked. This could probably be made
    // irrespective of the number of waypoints by bisecting the path and
    // checking the endpoints recursively.

    // try to find collisions that might come later in the path earlier
    if (int(path.size()) > inc_cc) {
        for (int i = 0; i < inc_cc; i++) {
            for (size_t j = i; j < path.size(); j = j + inc_cc) {
                num_checks++;
                if (!isStateValid(path[j], verbose, false, dist_temp)) {
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
            if (!isStateValid(path[i], verbose, false, dist_temp)) {
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

bool CollisionSpace::checkRobotCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
    bool in_collision = false;
    for (int ssidx : m_sphere_indices) {
        m_model.updateSphereState(ssidx);
        const CollisionSphereState& ss = m_model.sphereState(ssidx);

        int gx, gy, gz;
        m_grid->worldToGrid(ss.pos.x(), ss.pos.y(), ss.pos.z(), gx, gy, gz);

        // check bounds
        if (!m_grid->isInBounds(gx, gy, gz)) {
            if (verbose) {
                ROS_INFO("Sphere '%s' with center at (%0.3f, %0.3f, %0.3f) (%d, %d, %d) is out of bounds.", m_model.sphereState(ssidx).model->name.c_str(), ss.pos.x(), ss.pos.y(), ss.pos.z(), gx, gy, gz);
            }
            return false;
        }

        // check for collision with world
        double obs_dist = m_grid->getDistance(gx, gy, gz);
        const double effective_radius =
                m_model.sphereState(ssidx).model->radius +
                0.5 * m_grid->getResolution() +
                m_padding;

        if (obs_dist <= effective_radius) {
            if (verbose) {
                ROS_INFO("    *collision* idx: %d, name: %s, x: %d, y: %d, z: %d, radius: %0.3fm, dist: %0.3fm", ssidx, m_model.sphereState(ssidx).model->name.c_str(), gx, gy, gz, m_model.sphereState(ssidx).model->radius, obs_dist);
            }

            if (visualize) {
                in_collision = true;
                Sphere s;
                s.center = ss.pos;
                s.radius = m_model.sphereState(ssidx).model->radius;
                m_collision_spheres.push_back(s);
            }
            else {
                dist = obs_dist;
                return false;
            }
        }

        if (obs_dist < dist) {
            dist = obs_dist;
        }
    }

    return !in_collision;
}

bool CollisionSpace::checkSelfCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
    bool self_collision = false;
    // check self collisions
    for (size_t sidx1 = 0; sidx1 < m_sphere_indices.size(); ++sidx1) {
        const CollisionSphereState& ss1 = m_model.sphereState(sidx1);
        const CollisionSphereModel& smodel1 = *ss1.model;

        for (size_t sidx2 = 0; sidx2 < m_sphere_indices.size(); ++sidx2) {
            const CollisionSphereState& ss2 = m_model.sphereState(sidx2);
            const CollisionSphereModel& smodel2 = *ss2.model;

            Eigen::Vector3d dx = ss2.pos - ss1.pos;
            const double radius_combined = smodel1.radius + smodel2.radius;
            if (dx.squaredNorm() < radius_combined * radius_combined) {
                collision_detection::AllowedCollision::Type type;
                if (!m_acm.getEntry(smodel1.name, smodel2.name, type)) {
                    ROS_ERROR("An allowed collisions entry wasn't found for a collision sphere");
                }
                if (type == collision_detection::AllowedCollision::NEVER) {
                    if (visualize) {
                        self_collision = true;
                        Sphere s1, s2;
                        s1.center = ss1.pos;
                        s1.radius = smodel1.radius;
                        s2.center = ss2.pos;
                        s2.radius = smodel2.radius;
                        m_collision_spheres.push_back(s1);
                        m_collision_spheres.push_back(s2);
                    }
                    else {
                        return false;
                    }
                }
            }
        }
    }

    return !self_collision;
}

bool CollisionSpace::checkAttachedObjectCollision()
{
//    if (object_attached_) {
//        for (size_t i = 0; i < object_spheres_.size(); ++i) {
//            const Sphere& sphere = object_spheres_[i];
//            KDL::Vector v = frames_[sphere.kdl_chain][sphere.kdl_segment] * sphere.v;
//
//            int x, y, z;
//            m_grid->worldToGrid(v.x(), v.y(), v.z(), x, y, z);
//
//            // check bounds
//            if (!m_grid->isInBounds(x, y, z)) {
//                if (verbose) {
//                    ROS_INFO("Sphere %d %d %d is out of bounds.", x, y, z);
//                }
//                return false;
//            }
//
//            // check for collision with world
//            if ((dist_temp = m_grid->getDistance(x, y, z)) <= sphere.radius) {
//                dist = dist_temp;
//
//                if (visualize) {
//                    in_collision = true;
//                    Sphere s = *(spheres_[i]);
//                    s.v = v;
//                    m_collision_spheres.push_back(s);
//                }
//                else {
//                    return false;
//                }
//            }
//            if (dist_temp < dist) {
//                dist = dist_temp;
//            }
//        }
//    }

    return true;
}

double CollisionSpace::isValidLineSegment(
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

        if (!m_grid->isInBounds(nXYZ[0], nXYZ[1], nXYZ[2]))
            return 0;

        cell_val = m_grid->getDistance(nXYZ[0], nXYZ[1], nXYZ[2]);
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

void CollisionSpace::setAllowedCollisionMatrix(
    const collision_detection::AllowedCollisionMatrix& acm)
{
    m_acm = acm;
}

bool CollisionSpace::insertObject(const ObjectConstPtr& object)
{
    return m_world.insertObject(object);
}

bool CollisionSpace::removeObject(const ObjectConstPtr& object)
{
    return m_world.removeObject(object);
}

bool CollisionSpace::removeObject(const std::string& object_name)
{
    return m_world.removeObject(object_name);
}

bool CollisionSpace::moveShapes(const ObjectConstPtr& object)
{
    return m_world.moveShapes(object);
}

bool CollisionSpace::insertShapes(const ObjectConstPtr& object)
{
    return m_world.insertShapes(object);
}

bool CollisionSpace::removeShapes(const ObjectConstPtr& object)
{
    return m_world.removeShapes(object);
}

void CollisionSpace::removeAttachedObject()
{
//    object_attached_ = false;
//    object_spheres_.clear();
//    ROS_DEBUG("Removed attached object.");
}

void CollisionSpace::attachSphere(
    const std::string& name,
    const std::string& link,
    const geometry_msgs::Pose& pose,
    double radius)
{
//    object_attached_ = true;
//    attached_object_frame_ = link;
////    m_model.getFrameInfo(
////            attached_object_frame_,
////            m_group_name,
////            attached_object_chain_num_,
////            attached_object_segment_num_);
//
//    object_spheres_.resize(1);
//    object_spheres_[0].name = name;
//    object_spheres_[0].v.x(pose.position.x);
//    object_spheres_[0].v.y(pose.position.y);
//    object_spheres_[0].v.z(pose.position.z);
//    object_spheres_[0].radius = radius;
//    object_spheres_[0].kdl_chain = attached_object_chain_num_;
//    object_spheres_[0].kdl_segment = attached_object_segment_num_;
//
//    ROS_DEBUG("frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), m_group_name.c_str(), attached_object_chain_num_, attached_object_segment_num_);
//    ROS_DEBUG("Attached '%s' sphere.  xyz: %0.3f %0.3f %0.3f   radius: %0.3fm", name.c_str(), object_spheres_[0].v.x(), object_spheres_[0].v.y(), object_spheres_[0].v.z(), radius);
}

void CollisionSpace::attachCylinder(
    const std::string& link,
    const geometry_msgs::Pose& pose,
    double radius,
    double length)
{
//    object_attached_ = true;
//    attached_object_frame_ = link;
////    m_model.getFrameInfo(attached_object_frame_, m_group_name, attached_object_chain_num_, attached_object_segment_num_);
//
//    // compute end points of cylinder
//    KDL::Frame center;
//    tf::PoseMsgToKDL(pose, center);
//    KDL::Vector top(0.0,0.0,length/2.0);
//    KDL::Vector bottom(0.0,0.0,-length/2.0);
//    //KDL::Vector top(center.p), bottom(center.p);
//    std::vector<KDL::Vector> points;
//
//    //top.data[2] += length / 2.0;
//    //bottom.data[2] -= length / 2.0;
//
//    // get spheres
//    leatherman::getIntermediatePoints(top, bottom, radius, points);
//    int start = object_spheres_.size();
//    object_spheres_.resize(object_spheres_.size() + points.size());
//    for (size_t i = start; i < start + points.size(); ++i) {
//        object_spheres_[i].name = "attached_" + boost::lexical_cast<std::string>(i);
//        object_spheres_[i].v = center * points[i - start];
//        object_spheres_[i].radius = radius;
//        object_spheres_[i].kdl_chain = attached_object_chain_num_;
//        object_spheres_[i].kdl_segment = attached_object_segment_num_;
//    }
//
//    ROS_DEBUG("[attached_object] Attaching cylinder. pose: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f spheres: %d", pose.position.x, pose.position.y, pose.position.z, radius, length, int(object_spheres_.size()));
//    ROS_DEBUG("[attached_object]  frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), m_group_name.c_str(), attached_object_chain_num_, attached_object_segment_num_);
//    ROS_DEBUG("[attached_object]    top: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm", top.x(), top.y(), top.z(), radius);
//    ROS_DEBUG("[attached_object] bottom: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm", bottom.x(), bottom.y(), bottom.z(), radius);
}

void CollisionSpace::attachCube(
    const std::string& name,
    const std::string& link,
    const geometry_msgs::Pose& pose,
    double x_dim,
    double y_dim,
    double z_dim)
{
//    object_attached_ = true;
//    std::vector<std::vector<double>> spheres;
//    attached_object_frame_ = link;
////    if (!m_model.getFrameInfo(
////            attached_object_frame_,
////            m_group_name,
////            attached_object_chain_num_,
////            attached_object_segment_num_))
////    {
////        ROS_ERROR("Could not find frame info for attached object frame %s in group name %s", attached_object_frame_.c_str(), m_group_name.c_str());
////        object_attached_ = false;
////        return;
////    }
//
//    sbpl::SphereEncloser::encloseBox(
//            x_dim, y_dim, z_dim, object_enclosing_sphere_radius_, spheres);
//
//    if (spheres.size() <= 3) {
//        ROS_WARN("Attached cube is represented by %d collision spheres. Consider lowering the radius of the spheres used to populate the attached cube. (radius = %0.3fm)", int(spheres.size()), object_enclosing_sphere_radius_);
//    }
//    int start = object_spheres_.size();
//    object_spheres_.resize(start + spheres.size());
//    for (size_t i = start; i < start + spheres.size(); ++i) {
//        object_spheres_[i].name = name + "_" + boost::lexical_cast<std::string>(i);
//        tf::Vector3 sph_in_object_local_(spheres[i - start][0], spheres[i - start][1], spheres[i - start][2]);
//        tf::Transform T_sph_offset;
//        tf::poseMsgToTF(pose, T_sph_offset);
//        tf::Vector3 sph_in_link_local_ = T_sph_offset * sph_in_object_local_;
//        object_spheres_[i].v.x(sph_in_link_local_.getX());
//        object_spheres_[i].v.y(sph_in_link_local_.getY());
//        object_spheres_[i].v.z(sph_in_link_local_.getZ());
//        object_spheres_[i].radius = spheres[i - start][3];
//        object_spheres_[i].kdl_chain = attached_object_chain_num_;
//        object_spheres_[i].kdl_segment = attached_object_segment_num_;
//    }
//    ROS_DEBUG("Attaching '%s' represented by %d spheres with dimensions: %0.3f %0.3f %0.3f (sphere rad: %.3f) (chain id: %d)(segment id: %d)", name.c_str(), int(spheres.size()), x_dim, y_dim, z_dim, object_enclosing_sphere_radius_, attached_object_chain_num_, attached_object_segment_num_);
}

void CollisionSpace::attachMesh(
    const std::string& name,
    const std::string& link,
    const geometry_msgs::Pose& pose,
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles)
{
//    object_attached_ = true;
//    std::vector<std::vector<double>> spheres;
//    attached_object_frame_ = link;
////    m_model.getFrameInfo(
////            attached_object_frame_,
////            m_group_name,
////            attached_object_chain_num_,
////            attached_object_segment_num_);
//
//    sbpl::SphereEncloser::encloseMesh(
//            vertices, triangles, object_enclosing_sphere_radius_, spheres);
//
//    if (spheres.size() <= 3) {
//        ROS_WARN("Attached mesh is represented by %zu collision spheres. Consider lowering the radius of the spheres used to populate the attached mesh more accuratly. (radius = %0.3fm)", spheres.size(), object_enclosing_sphere_radius_);
//    }
//
//    object_spheres_.resize(spheres.size());
//    for (size_t i = 0; i < spheres.size(); ++i) {
//        object_spheres_[i].name = name + "_" + boost::lexical_cast<std::string>(i);
//        object_spheres_[i].v.x(spheres[i][0]);
//        object_spheres_[i].v.y(spheres[i][1]);
//        object_spheres_[i].v.z(spheres[i][2]);
//        object_spheres_[i].radius = spheres[i][3];
//        object_spheres_[i].kdl_chain = attached_object_chain_num_;
//        object_spheres_[i].kdl_segment = attached_object_segment_num_;
//    }
//
//    ROS_DEBUG("Attaching '%s' represented by %d spheres with %d vertices and %d triangles.", name.c_str(), int(spheres.size()), int(vertices.size()), int(triangles.size()));
}

bool CollisionSpace::getAttachedObject(
    const std::vector<double>& angles,
    std::vector<std::vector<double>>& xyz)
{
//    KDL::Vector v;
//    int x, y, z;
//    xyz.clear();
//
//    if (!object_attached_) {
//        return false;
//    }
//
//    // compute foward kinematics
//    if (!m_model.computeDefaultGroupFK(angles, frames_)) {
//        ROS_ERROR("Failed to compute foward kinematics.");
//        return false;
//    }
//
//    xyz.resize(object_spheres_.size(), std::vector<double>(4, 0));
//    for (size_t i = 0; i < object_spheres_.size(); ++i) {
//        v = frames_[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] * object_spheres_[i].v;
//
//        // snap to grid
//        m_grid->worldToGrid(v.x(), v.y(), v.z(), x, y, z);
//        m_grid->gridToWorld(x, y, z, xyz[i][0], xyz[i][1], xyz[i][2]);
//
//        xyz[i][3] = object_spheres_[i].radius;
//    }
//
    return true;
}

bool CollisionSpace::processCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    return m_world.processCollisionObject(object);
}

bool CollisionSpace::processOctomapMsg(
    const octomap_msgs::OctomapWithPose& octomap)
{
    return m_world.insertOctomap(octomap);
}

bool CollisionSpace::setJointPosition(
    const std::string& name,
    double position)
{
    if (m_model.hasJointVar(name)) {
        m_model.setJointPosition(name, position);
        return true;
    }
    else {
        return false;
    }
}

void CollisionSpace::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    m_model.setWorldToModelTransform(transform);
}

bool CollisionSpace::interpolatePath(
    const std::vector<double>& start,
    const std::vector<double>& end,
    std::vector<std::vector<double>>& path)
{
    return sbpl::interp::InterpolatePath(
            start, end, m_min_limits, m_max_limits, m_increments, path);
}

bool CollisionSpace::getClearance(
    const std::vector<double>& angles,
    int num_spheres,
    double& avg_dist,
    double& min_dist)
{
//    KDL::Vector v;
//    int x, y, z;
//    double sum = 0, dist = 100;
//    min_dist = 100;
//
//    if (!m_model.computeDefaultGroupFK(angles, frames_)) {
//        ROS_ERROR("Failed to compute foward kinematics.");
//        return false;
//    }
//
//    if (num_spheres > int(spheres_.size())) {
//        num_spheres = spheres_.size();
//    }
//
//    for (int i = 0; i < num_spheres; ++i) {
//        v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;
//        m_grid->worldToGrid(v.x(), v.y(), v.z(), x, y, z);
//        dist = m_grid->getDistance(x, y, z) - spheres_[i]->radius;
//
//        if (min_dist > dist)
//            min_dist = dist;
//        sum += dist;
//    }
//
//    avg_dist = sum / num_spheres;
//    ROS_DEBUG(" num_spheres: %d  avg_dist: %2.2f   min_dist: %2.2f", num_spheres, avg_dist, min_dist);
    return true;
}

bool CollisionSpace::isStateValid(
    const std::vector<double>& angles,
    bool verbose,
    bool visualize,
    double& dist)
{
    // allow subroutines to update minimum distance
    dist = std::numeric_limits<double>::max();

    if (visualize) {
        // allow subroutines to gather collision spheres for visualization
        m_collision_spheres.clear();
    }

    // update the robot state
    for (size_t i = 0; i < angles.size(); ++i) {
        int jidx = m_planning_joint_to_collision_model_indices[i];
        m_model.setJointPosition(jidx, angles[i]);
    }

    updateVoxelsStates();

    bool attached_object_world_valid = checkAttachedObjectCollision();
    if (!visualize && !attached_object_world_valid) {
        return false;
    }

    bool robot_world_valid = checkRobotCollision(verbose, visualize, dist);
    if (!visualize && !robot_world_valid) {
        return false;
    }

    bool robot_robot_valid = checkSelfCollision(verbose, visualize, dist);
    if (!visualize && !robot_robot_valid) {
        return false;
    }

    return attached_object_world_valid &&
            robot_world_valid &&
            robot_robot_valid;
}

bool CollisionSpace::isStateToStateValid(
    const std::vector<double>& angles0,
    const std::vector<double>& angles1,
    int& path_length,
    int& num_checks,
    double &dist)
{
    return checkPathForCollision(
            angles0, angles1, false, path_length, num_checks, dist);
}

bool CollisionSpace::setPlanningScene(
    const moveit_msgs::PlanningScene& scene)
{
    ROS_INFO("Setting the Planning Scene");

    // TODO: currently ignored fields from moveit_msgs::PlanningScene
    // * name
    // * --robot_state--
    // * robot_model_name
    // * fixed_frame_transforms
    // * allowed_collision_matrix
    // * link_padding
    // * link_scale
    // * object_colors
    // * --world--
    // * --is_diff--

    if (scene.is_diff) {
        ROS_ERROR("Collision space does not support differential planning scene updates");
        return false;
    }

    /////////////////
    // robot state //
    /////////////////

    const moveit_msgs::RobotState& robot_state = scene.robot_state;

    const sensor_msgs::JointState& joint_state = robot_state.joint_state;
    if (joint_state.name.size() != joint_state.position.size()) {
        ROS_ERROR("Robot state does not contain correct number of joint positions (Expected: %zd, Actual: %zd)", scene.robot_state.joint_state.name.size(), scene.robot_state.joint_state.position.size());
        return false;
    }

    // set the single-dof joint state
    for (size_t i = 0; i < joint_state.name.size(); ++i) {
        const std::string& joint_name = joint_state.name[i];
        double joint_position = joint_state.position[i];
        m_model.setJointPosition(joint_name, joint_position);
    }

    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state =
            robot_state.multi_dof_joint_state;
    for (size_t i = 0; i < multi_dof_joint_state.joint_names.size(); ++i) {
        // TODO: handle multi-dof joint state
    }

    // TODO: handle world -> model transform

    const auto& attached_collision_objects =
            robot_state.attached_collision_objects;
    for (const auto& attached_collision_object : attached_collision_objects) {
        // TODO: handle attached collision objects
    }

    //////////////////////////
    // planning scene world //
    //////////////////////////

    const auto& planning_scene_world = scene.world;

    const auto& collision_objects = planning_scene_world.collision_objects;
    ROS_INFO("Processing %zd collision objects", scene.world.collision_objects.size());
    for (const moveit_msgs::CollisionObject& collision_object : scene.world.collision_objects) {
        if (!processCollisionObject(collision_object)) {
            ROS_ERROR("Failed to process collision object '%s'", collision_object.id.c_str());
        }
    }

    const auto& octomap = planning_scene_world.octomap;
    if (!processOctomapMsg(octomap)) {
        ROS_ERROR("Failed to process octomap '%s'", octomap.octomap.id.c_str());
    }

    // self collision
    // TODO: This now occurs during checkCollision under the assumption that
    // some planning joints may force voxels model updates. It may be worthwhile
    // to assert whether planning joints can affect the state of voxels models
    // here and make a single update here instead of on every call to
    // checkCollision
//    updateVoxelGroups();

    return true;
}

void CollisionSpace::attachObject(
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
        ROS_WARN("[attach_object] Converted shape from %s (%0.2f %0.2f %0.2f) to %s", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str());

        if (object.primitives[i].type == shape_msgs::SolidPrimitive::SPHERE) {
            ROS_INFO("Attaching a '%s' sphere with radius: %0.3fm", object.id.c_str(), object.primitives[i].dimensions[0]);
            attachSphere(object.id, link_name, object.primitive_poses[i], object.primitives[i].dimensions[0]);
        }
        else if (object.primitives[i].type == shape_msgs::SolidPrimitive::CYLINDER) {
            ROS_INFO("Attaching a '%s' cylinder with radius: %0.3fm & length %0.3fm", object.id.c_str(), object.primitives[i].dimensions[0], object.primitives[i].dimensions[1]);
            attachCylinder(link_name, object.primitive_poses[i], object.primitives[i].dimensions[1], object.primitives[i].dimensions[0]);
        }
        else if (object.primitives[i].type == shape_msgs::SolidPrimitive::BOX) {
            ROS_INFO("Attaching a '%s' cube with dimensions {%0.3fm x %0.3fm x %0.3fm}.", object.id.c_str(), object.primitives[i].dimensions[0], object.primitives[i].dimensions[1], object.primitives[i].dimensions[2]);
            attachCube(object.id, link_name, object.primitive_poses[i], object.primitives[i].dimensions[0], object.primitives[i].dimensions[1], object.primitives[i].dimensions[2]);
        }
        else {
            ROS_WARN("Currently attaching objects of type '%d' aren't supported.", object.primitives[i].type);
        }
    }

    for (size_t i = 0; i < object.meshes.size(); i++) {
        pose_in.header = object.header;
        pose_in.header.stamp = ros::Time();
        pose_in.pose = object.mesh_poses[i];

        ROS_WARN("[attach_object] Converted shape from %s (%0.2f %0.2f %0.2f) to %s", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str());

        ROS_INFO("Attaching a '%s' mesh with %d triangles & %d vertices is NOT supported right now...", object.id.c_str(), int(object.meshes[i].triangles.size() / 3), int(object.meshes[i].vertices.size()));
        attachMesh(object.id, link_name, object.mesh_poses[i], object.meshes[i].vertices, ConvertToVertexIndices(object.meshes[i].triangles));
    }

    if (!object.planes.empty()) {
        ROS_WARN("Attempted to attach object with %zd planes. Ignoring plane components...", object.planes.size());
    }

    ROS_WARN("Attached object has %zd spheres!", object_spheres_.size());
}

visualization_msgs::MarkerArray
CollisionSpace::getCollisionObjectsVisualization() const
{
    return m_world.getCollisionObjectsVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getCollisionsVisualization() const
{
//    std::vector<double> rad(m_collision_spheres.size());
//    std::vector<std::vector<double>> sph(
//            m_collision_spheres.size(), std::vector<double>(3, 0));
//    for (size_t i = 0; i < m_collision_spheres.size(); ++i) {
//        sph[i][0] = m_collision_spheres[i].v.x();
//        sph[i][1] = m_collision_spheres[i].v.y();
//        sph[i][2] = m_collision_spheres[i].v.z();
//        rad[i] = spheres_[i]->radius;
//    }
//    return viz::getSpheresMarkerArray(
//            sph, rad, 10, m_grid->getReferenceFrame(), "collision_spheres", 0);
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
CollisionSpace::getCollisionObjectVoxelsVisualization() const
{
    return m_world.getCollisionObjectVoxelsVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getBoundingBoxVisualization() const
{
    return m_grid->getBoundingBoxVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getDistanceFieldVisualization() const
{
    return m_grid->getDistanceFieldVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getOccupiedVoxelsVisualization() const
{
    return m_grid->getOccupiedVoxelsVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getVisualization(
    const std::string& type)
{
    if (type == "collision_objects") {
        return getCollisionObjectsVisualization();
    }
    else if (type == "collisions") {
        return getCollisionsVisualization();
    }
    else if (type == "collision_object_voxels") {
        return getCollisionObjectVoxelsVisualization();
    }
    else if (type == "collision_model") {
        auto markers = m_model.getVisualization();
        for (auto& m : markers.markers) {
            m.header.frame_id = getReferenceFrame();
        }
        return markers;
    }
    else if (type == "attached_object") {
//        std::vector<double> angles, rad;
//        std::vector<std::vector<double>> sph;
//        if (object_attached_) {
//            getAttachedObject(angles, sph);
//        }
//
//        if (sph.empty() || sph[0].size() < 4) {
//            return ma;
//        }
//
//        rad.resize(sph.size());
//        for (size_t i = 0; i < sph.size(); ++i) {
//            rad[i] = sph[i][3];
//        }
//
//        ma = viz::getSpheresMarkerArray(sph, rad, 200, m_grid->getReferenceFrame(), "attached_object", 0);
        return visualization_msgs::MarkerArray();
    }
    else {
        return m_grid->getVisualization(type);
    }
}

visualization_msgs::MarkerArray
CollisionSpace::getMeshModelVisualization(
    const std::string& group_name,
    const std::vector<double> &angles)
{
//    visualization_msgs::MarkerArray ma;
//    geometry_msgs::Pose fpose;
//    geometry_msgs::PoseStamped lpose, mpose;
//    std::string robot_description, mesh_resource;
//    Group* g = m_model.getGroup(group_name);
//
//    ros::NodeHandle nh;
//    if (!nh.getParam("robot_description", robot_description)) {
//        ROS_ERROR("Failed to get robot_description from param server.");
//        return ma;
//    }
//
//    // compute foward kinematics
//    if (!m_model.computeGroupFK(angles, g, frames_)) {
//        ROS_ERROR("Failed to compute foward kinematics.");
//        return ma;
//    }
//
//    // get link mesh_resources
//    for (size_t i = 0; i < g->links_.size(); ++i) {
//        if (!leatherman::getLinkMesh(robot_description, g->links_[i].root_name_, false, mesh_resource, lpose)) {
//            ROS_ERROR("Failed to get mesh for '%s'.", g->links_[i].root_name_.c_str());
//            continue;
//        }
//
//        ROS_INFO("Got the mesh! (%s)", mesh_resource.c_str());
//        // TODO: Has to be a spheres group
//        leatherman::msgFromPose(frames_[g->links_[i].spheres_[0].kdl_chain][g->links_[i].spheres_[0].kdl_segment], fpose);
//        leatherman::multiply(fpose, lpose.pose, mpose.pose);
//        mpose.header.frame_id = "base_link"; //getReferenceFrame();
//        ma.markers.push_back(viz::getMeshMarker(mpose, mesh_resource, 180, "robot_model", i));
//    }
//    return ma;
    return visualization_msgs::MarkerArray();
}

} // namespace collision
} // namespace sbpl
