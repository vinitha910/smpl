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
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <leatherman/bresenham.h>
#include <leatherman/print.h>
#include <leatherman/viz.h>
#include <moveit_msgs/RobotState.h>
#include <sbpl_geometry_utils/SphereEncloser.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>

namespace sbpl {
namespace collision {

static const char* CC_LOGGER = "cspace";

CollisionSpace::CollisionSpace(OccupancyGrid* grid) :
    m_grid(grid),
    m_world(grid),
    m_model(),
    m_state(&m_model),
    m_group_name(),
    m_group_index(-1),
    m_sphere_indices(),
    m_voxels_indices(),
    m_ao_sphere_indices(),
    m_ao_voxels_indices(),
    m_planning_joint_to_collision_model_indices(),
    m_increments(),
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
            ROS_ERROR_NAMED(CC_LOGGER, "Joint variable '%s' not found in Robot Collision Model", joint_name.c_str());
            return false;
        }
    }

    // map planning joint indices to collision model indices
    m_planning_joint_to_collision_model_indices.resize(joint_names.size(), -1);

    m_increments.resize(joint_names.size(), utils::ToRadians(2.0));
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];;
        int jidx = m_model.jointVarIndex(joint_name);

        m_planning_joint_to_collision_model_indices[i] = jidx;
    }

    return true;
}

bool CollisionSpace::init(
    const urdf::ModelInterface& urdf,
    const std::string& group_name,
    const CollisionModelConfig& config,
    const std::vector<std::string>& planning_joints)
{
    ROS_DEBUG_NAMED(CC_LOGGER, "Initializing collision space for group '%s'", group_name.c_str());

    if (!m_model.init(urdf, config)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to initialize the Robot Collision Model");
        return false;
    }

    initAllowedCollisionMatrix(config);
    m_acm.print(std::cout);

    if (!setPlanningJoints(planning_joints)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to set planning joints");
        return false;
    }

    if (!m_model.hasGroup(group_name)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Group '%s' was not found in the Robot Collision Model", group_name.c_str());
        return false;
    }

    m_group_name = group_name;
    m_group_index = m_model.groupIndex(m_group_name);

    // aggregate all the sphere indices from all spheres states
    std::vector<int> ss_indices = m_state.groupSpheresStateIndices(m_group_index);
    for (int ssidx : ss_indices) {
        const CollisionSpheresState& spheres_state = m_state.spheresState(ssidx);
        std::vector<int> s_indices(spheres_state.spheres.size());
        int n = 0;
        std::generate(s_indices.begin(), s_indices.end(), [&]() { return n++; });
        for (int sidx : s_indices) {
            m_sphere_indices.emplace_back(ssidx, sidx);
        }
    }

    // sort sphere state indices by priority
    std::sort(m_sphere_indices.begin(), m_sphere_indices.end(),
            [&](const SphereIndex& sidx1, const SphereIndex& sidx2)
            {
                const CollisionSphereState& ss1 = m_state.sphereState(sidx1);
                const CollisionSphereState& ss2 = m_state.sphereState(sidx2);
                const CollisionSphereModel* sph1 = ss1.model;
                const CollisionSphereModel* sph2 = ss2.model;
                return sph1->priority < sph2->priority;
            });

    m_voxels_indices = m_state.groupOutsideVoxelsStateIndices(m_group_index);

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
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to parse URDF");
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
        if (m_state.voxelsStateDirty(vsidx)) {
            const CollisionVoxelsState& voxels_state = m_state.voxelsState(vsidx);

            // copy over voxels to be removed before updating
            voxel_removals.insert(
                    voxel_removals.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            m_state.updateVoxelsState(vsidx);

            // copy over voxels to be inserted
            voxel_insertions.insert(
                    voxel_insertions.end(),
                    voxels_state.voxels.begin(),
                    voxels_state.voxels.end());

            ROS_DEBUG_NAMED(CC_LOGGER, "Updating Occupancy Grid with change to Collision Voxels State (%zu displaced)", voxel_removals.size());
        }
    }

    // TODO: check for voxel state updates from attached objects

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
    // add allowed collisions between spheres on the same link
    for (const auto& spheres_config : config.spheres_models) {
        for (size_t i = 0; i < spheres_config.spheres.size(); ++i) {
            const std::string& s1_name = spheres_config.spheres[i].name;
            if (!m_acm.hasEntry(s1_name)) {
                ROS_INFO_NAMED(CC_LOGGER, "Adding entry '%s' to the ACM", s1_name.c_str());
                m_acm.setEntry(s1_name, false);
            }
            for (size_t j = i + 1; j < spheres_config.spheres.size(); ++j) {
                const std::string& s2_name = spheres_config.spheres[j].name;
                if (!m_acm.hasEntry(s2_name)) {
                    ROS_INFO_NAMED(CC_LOGGER, "Adding entry '%s' to the ACM", s2_name.c_str());
                    m_acm.setEntry(s2_name, false);
                }

                ROS_INFO_NAMED(CC_LOGGER, "Spheres '%s' and '%s' attached to the same link...allowing collision", s1_name.c_str(), s2_name.c_str());
                m_acm.setEntry(s1_name, s2_name, true);
            }
        }
    }

    // add in additional allowed collisions from config
    std::vector<std::string> config_entries;
    config.acm.getAllEntryNames(config_entries);
    for (size_t i = 0; i < config_entries.size(); ++i) {
        const std::string& entry1 = config_entries[i];
        if (!m_acm.hasEntry(entry1)) {
            ROS_WARN_NAMED(CC_LOGGER, "Configured allowed collision entry '%s' was not found in the collision model", entry1.c_str());
            continue;
        }
        for (size_t j = i; j < config_entries.size(); ++j) {
            const std::string& entry2 = config_entries[j];
            if (!m_acm.hasEntry(entry2)) {
                ROS_WARN_NAMED(CC_LOGGER, "Configured allowed collision entry '%s' was not found in the collision model", entry2.c_str());
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
                ROS_INFO_NAMED(CC_LOGGER, "Configuration allows spheres '%s' and '%s' to be in collision", entry1.c_str(), entry2.c_str());
                m_acm.setEntry(entry1, entry2, true);
                break;
            case collision_detection::AllowedCollision::CONDITIONAL:
                ROS_WARN_NAMED(CC_LOGGER, "Conditional collisions not supported in SBPL Collision Detection");
                break;
            }
        }
    }
}

void CollisionSpace::updateAttachedBodyIndices()
{
//    std::vector<int> static_sphere_indices = m_sphere_indices;
//    std::vector<int> static_voxels_indices = m_voxels_indices;
//    std::sort(static_sphere_indices.begin(), static_sphere_indices.end());
//    std::sort(static_voxels_indices.begin(), static_sphere_indices.end());
//
//    std::vector<int> ss_indices =
//            m_state.groupSphereStateIndices(m_group_index);
//    std::vector<int> ovs_indices =
//            m_state.groupOutsideVoxelsStateIndices(m_group_index);
//    std::sort(ss_indices.begin(), ss_indices.end());
//    std::sort(ovs_indices.begin(), ovs_indices.end());
//
//    std::vector<int> dynamic_sphere_indices;
//    std::vector<int> dynamic_voxels_indices;
//
//    std::set_difference(
//            ss_indices.begin(), ss_indices.end(),
//            static_sphere_indices.begin(), static_sphere_indices.end(),
//            std::back_inserter(dynamic_sphere_indices));
//
//    std::set_difference(
//            ovs_indices.begin(), ovs_indices.end(),
//            static_voxels_indices.begin(), static_voxels_indices.end(),
//            std::back_inserter(dynamic_voxels_indices));
//
//    m_ao_sphere_indices = dynamic_sphere_indices;
//    m_ao_voxels_indices = dynamic_voxels_indices;
}

bool CollisionSpace::withinJointPositionLimits(
    const std::vector<double>& positions) const
{
    assert(positions.size() == planningVariableCount());
    bool inside = true;
    for (size_t vidx = 0; vidx < planningVariableCount(); ++vidx) {
        const double pos = positions[vidx];
        if (!(isContinuous(vidx) ||
            !hasLimit(vidx) ||
            (pos >= minLimit(vidx) && pos <= maxLimit(vidx))))
        {
            inside = false;
            break;
        }
    }
    return inside;
}

bool CollisionSpace::checkSphereCollision(
    const SphereIndex& sidx,
    bool verbose,
    double& dist)
{
    m_state.updateSphereState(sidx);
    const CollisionSphereState& ss = m_state.sphereState(sidx);

    int gx, gy, gz;
    m_grid->worldToGrid(ss.pos.x(), ss.pos.y(), ss.pos.z(), gx, gy, gz);

    // check bounds
    if (!m_grid->isInBounds(gx, gy, gz)) {
        if (verbose) {
            const CollisionSphereModel* sm = ss.model;
            ROS_INFO_NAMED(CC_LOGGER, "Sphere '%s' with center at (%0.3f, %0.3f, %0.3f) (%d, %d, %d) is out of bounds.", sm->name.c_str(), ss.pos.x(), ss.pos.y(), ss.pos.z(), gx, gy, gz);
        }
        dist = 0.0;
        return false;
    }

    // check for collision with world
    double obs_dist = m_grid->getDistance(gx, gy, gz);
    const double effective_radius =
            ss.model->radius + 0.5 * m_grid->getResolution() + m_padding;

    if (obs_dist <= effective_radius) {
        dist = obs_dist;
        return false;
    }

    dist = obs_dist;
    return true;
}

bool CollisionSpace::checkRobotCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
    bool in_collision = false;
    for (const auto& sidx : m_sphere_indices) {
        double obs_dist;
        if (!checkSphereCollision(sidx, verbose, obs_dist)) {
            if (verbose) {
                const CollisionSphereModel* sm = m_state.sphereState(sidx).model;
                ROS_INFO_NAMED(CC_LOGGER, "    *collision* idx: %s, name: %s, radius: %0.3fm, dist: %0.3fm", to_string(sidx).c_str(), sm->name.c_str(), sm->radius, obs_dist);
            }

            if (visualize) {
                in_collision = true;
                Sphere s;
                s.center = m_state.sphereState(sidx).pos;
                s.radius = m_state.sphereState(sidx).model->radius;
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
    for (size_t i = 0; i < m_sphere_indices.size(); ++i) {
        const auto& sidx1 = m_sphere_indices[i];
        const CollisionSphereState& ss1 = m_state.sphereState(sidx1);
        const CollisionSphereModel& smodel1 = *ss1.model;

        for (size_t j = i + 1; j < m_sphere_indices.size(); ++j) {
            const auto& sidx2 = m_sphere_indices[j];
            const CollisionSphereState& ss2 = m_state.sphereState(sidx2);
            const CollisionSphereModel& smodel2 = *ss2.model;

            Eigen::Vector3d dx = ss2.pos - ss1.pos;
            const double radius_combined = smodel1.radius + smodel2.radius;
            if (dx.squaredNorm() < radius_combined * radius_combined) {
                collision_detection::AllowedCollision::Type type;
                if (!m_acm.getEntry(smodel1.name, smodel2.name, type)) {
                    ROS_ERROR_NAMED(CC_LOGGER, "An allowed collisions entry wasn't found for a collision sphere");
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

bool CollisionSpace::checkAttachedObjectCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
//    bool in_collision = false;
//    for (int ssidx : m_ao_sphere_indices) {
//        double obs_dist;
//        if (!checkSphereCollision(ssidx, verbose, obs_dist)) {
//            if (verbose) {
//                const CollisionSphereModel* sm = m_state.sphereState(sidx).model;
//                ROS_INFO_NAMED(CC_LOGGER, "    *collision* idx: %d, name: %s, radius: %0.3f, dist: %0.3fm", ssidx, sm->name.c_str(), sm->radius, obs_dist);
//            }
//
//            if (visualize) {
//                in_collision = true;
//                Sphere s;
//                s.center = m_state.sphereState(ssidx).pos;
//                s.radius = m_state.sphereState(ssidx).model->radius;
//                m_collision_spheres.push_back(s);
//            }
//            else {
//                dist = obs_dist;
//                return false;
//            }
//        }
//
//        if (obs_dist < dist) {
//            dist = obs_dist;
//        }
//    }
//
//    return !in_collision;
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
        m_state.setJointVarPosition(name, position);
        return true;
    }
    else {
        return false;
    }
}

void CollisionSpace::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    m_state.setWorldToModelTransform(transform);
}

bool CollisionSpace::interpolatePath(
    const std::vector<double>& start,
    const std::vector<double>& finish,
    std::vector<std::vector<double>>& opath)
{
    assert(start.size() == m_planning_joint_to_collision_model_indices.size() &&
            finish.size() == m_planning_joint_to_collision_model_indices.size());

    // check joint limits on the start and finish points
    if (!(withinJointPositionLimits(start) &&
            withinJointPositionLimits(finish)))
    {
        ROS_ERROR_NAMED(CC_LOGGER, "Joint limits violated");
        return false;
    }

    // compute distance traveled by each joint
    std::vector<double> diffs(planningVariableCount(), 0.0);
    for (size_t vidx = 0; vidx < planningVariableCount(); ++vidx) {
        if (isContinuous(vidx)) {
            diffs[vidx] = angles::ShortestAngleDiff(finish[vidx], start[vidx]);
        }
        else {
            diffs[vidx] = finish[vidx] - start[vidx];
        }
    }

    // compute the number of intermediate waypoints including start and end
    int waypoint_count = 0;
    for (size_t vidx = 0; vidx < planningVariableCount(); vidx++) {
        int angle_waypoints = (int)(std::fabs(diffs[vidx]) / m_increments[vidx]) + 1;
        waypoint_count = std::max(waypoint_count, angle_waypoints);
    }
    waypoint_count = std::max(waypoint_count, 2);

    // fill intermediate waypoints
    std::vector<std::vector<double>> path(
            waypoint_count,
            std::vector<double>(planningVariableCount(), 0.0));
    for (size_t vidx = 0; vidx < planningVariableCount(); ++vidx) {
        for (size_t widx = 0; widx < waypoint_count; ++widx) {
            double alpha = (double)widx / (double)(waypoint_count - 1);
            double pos = start[vidx] + alpha * diffs[vidx];
            path[widx][vidx] = pos;
        }
    }

    // normalize output angles
    for (size_t vidx = 0; vidx < planningVariableCount(); ++vidx) {
        if (isContinuous(vidx)) {
            for (size_t widx = 0; widx < waypoint_count; ++widx) {
                path[widx][vidx] = angles::NormalizeAngle(path[widx][vidx]);
            }
        }
    }

    opath = std::move(path);
    return true;
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
//        ROS_ERROR_NAMED(CC_LOGGER, "Failed to compute foward kinematics.");
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
//    ROS_DEBUG_NAMED(CC_LOGGER, " num_spheres: %d  avg_dist: %2.2f   min_dist: %2.2f", num_spheres, avg_dist, min_dist);
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
        m_state.setJointVarPosition(jidx, angles[i]);
    }

    updateVoxelsStates();

    bool robot_world_valid = checkRobotCollision(verbose, visualize, dist);
    if (!visualize && !robot_world_valid) {
        return false;
    }

    bool robot_robot_valid = checkSelfCollision(verbose, visualize, dist);
    if (!visualize && !robot_robot_valid) {
        return false;
    }

    bool attached_object_world_valid = checkAttachedObjectCollision(
            verbose, visualize, dist);
    if (!visualize && !attached_object_world_valid) {
        return false;
    }

    return attached_object_world_valid &&
            robot_world_valid &&
            robot_robot_valid;
}

bool CollisionSpace::isStateToStateValid(
    const std::vector<double>& start,
    const std::vector<double>& finish,
    int& path_length,
    int& num_checks,
    double &dist)
{
    const bool verbose = false;

    int inc_cc = 5;
    double dist_temp = 0;
    std::vector<double> start_norm(start);
    std::vector<double> end_norm(finish);
    std::vector<std::vector<double>> path;
    dist = 100;
    num_checks = 0;

    if (!interpolatePath(start_norm, end_norm, path)) {
        path_length = 0;
        ROS_ERROR_ONCE("Failed to interpolate the path. It's probably infeasible due to joint limits.");
        ROS_ERROR_NAMED(CC_LOGGER, "[interpolate]  start: %s", to_string(start_norm).c_str());
        ROS_ERROR_NAMED(CC_LOGGER, "[interpolate]    finish: %s", to_string(end_norm).c_str());
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

bool CollisionSpace::setPlanningScene(
    const moveit_msgs::PlanningScene& scene)
{
    ROS_INFO_NAMED(CC_LOGGER, "Setting the Planning Scene");

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
        ROS_ERROR_NAMED(CC_LOGGER, "Collision space does not support differential planning scene updates");
        return false;
    }

    /////////////////
    // robot state //
    /////////////////

    const moveit_msgs::RobotState& robot_state = scene.robot_state;

    const sensor_msgs::JointState& joint_state = robot_state.joint_state;
    if (joint_state.name.size() != joint_state.position.size()) {
        ROS_ERROR_NAMED(CC_LOGGER, "Robot state does not contain correct number of joint positions (Expected: %zd, Actual: %zd)", scene.robot_state.joint_state.name.size(), scene.robot_state.joint_state.position.size());
        return false;
    }

    // set the single-dof joint state
    for (size_t i = 0; i < joint_state.name.size(); ++i) {
        const std::string& joint_name = joint_state.name[i];
        double joint_position = joint_state.position[i];
        m_state.setJointVarPosition(joint_name, joint_position);
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
    ROS_INFO_NAMED(CC_LOGGER, "Processing %zd collision objects", scene.world.collision_objects.size());
    for (const moveit_msgs::CollisionObject& collision_object : scene.world.collision_objects) {
        if (!processCollisionObject(collision_object)) {
            ROS_ERROR_NAMED(CC_LOGGER, "Failed to process collision object '%s'", collision_object.id.c_str());
        }
    }

    const auto& octomap = planning_scene_world.octomap;
    if (!processOctomapMsg(octomap)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to process octomap '%s'", octomap.octomap.id.c_str());
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

bool CollisionSpace::attachObject(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name)
{
//    if (!m_model.attachBody(id, shapes, transforms, link_name)) {
//        return false;
//    }
//    updateAttachedBodyIndices();
//    return true;
}

bool CollisionSpace::removeAttachedObject(const std::string& id)
{
//    if (!m_model.detachBody(id)) {
//        return false;
//    }
//    updateAttachedBodyIndices();
//    return true;
}

bool CollisionSpace::processAttachedCollisionObject(
    const moveit_msgs::AttachedCollisionObject& ao)
{
    switch (ao.object.operation) {
    case moveit_msgs::CollisionObject::ADD:
    {
        ObjectConstPtr o = ConvertCollisionObjectToObject(ao.object);
        if (!attachObject(
                ao.object.id, o->shapes_, o->shape_poses_, ao.link_name))
        {
            return false;
        }

        // TODO: handle touch links

        updateAttachedBodyIndices();
        return true;
    }   break;
    case moveit_msgs::CollisionObject::REMOVE:
    {
        if (!removeAttachedObject(ao.object.id)) {
            return false;
        }

        updateAttachedBodyIndices();
        return true;
    }   break;
    case moveit_msgs::CollisionObject::APPEND:
    case moveit_msgs::CollisionObject::MOVE:
    default:
        throw std::runtime_error("unimplemented");
    }

    return false;
}

visualization_msgs::MarkerArray
CollisionSpace::getWorldVisualization() const
{
    return m_world.getWorldVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getRobotVisualization() const
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
CollisionSpace::getCollisionWorldVisualization() const
{
    return m_world.getCollisionWorldVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getCollisionRobotVisualization() const
{
    auto markers = m_state.getVisualization();
    for (auto& m : markers.markers) {
        m.header.frame_id = getReferenceFrame();
    }
    return markers;
}

visualization_msgs::MarkerArray
CollisionSpace::getCollisionDetailsVisualization(
    const std::vector<double>& vals) const
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
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
CollisionSpace::getCollisionModelVisualization(const std::vector<double>& vals)
{
    // TODO: fill in input values
    return getCollisionRobotVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getVisualization(
    const std::string& type)
{
    if (type == "world") {
        return getWorldVisualization();
    }
    else if (type == "collision_world") {
        return getCollisionWorldVisualization();
    }
    else if (type == "robot") {
        return getRobotVisualization();
    }
    else if (type == "collision_robot") {
        return getCollisionRobotVisualization();
    }
    else if (type == "collision_details") {
        // TODO: save the last state checked for collision?
        return getCollisionDetailsVisualization(std::vector<double>(planningVariableCount(), 0));
    }
    else if (type == "attached_object") {
        return visualization_msgs::MarkerArray();
    }
    else {
        return m_grid->getVisualization(type);
    }
}

} // namespace collision
} // namespace sbpl
