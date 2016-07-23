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
    m_rcm(),
    // TODO: objects dependent on robot collision model should be instantiated
    // after its initialization
    m_abcm(&m_rcm),
    m_rcs(&m_rcm),
    m_abcs(&m_abcm, &m_rcs),
    m_wcm(grid),
    m_scm(grid, &m_rcm),
    m_group_name(),
    m_gidx(-1),
    m_planning_joint_to_collision_model_indices(),
    m_increments()
{
}

CollisionSpace::~CollisionSpace()
{
}

bool CollisionSpace::init(
    const urdf::ModelInterface& urdf,
    const std::string& group_name,
    const CollisionModelConfig& config,
    const std::vector<std::string>& planning_joints)
{
    ROS_DEBUG_NAMED(CC_LOGGER, "Initializing collision space for group '%s'", group_name.c_str());

    if (!m_rcm.init(urdf, config)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to initialize the Robot Collision Model");
        return false;
    }

    if (!setPlanningJoints(planning_joints)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to set planning joints");
        return false;
    }

    if (!m_rcm.hasGroup(group_name)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Group '%s' was not found in the Robot Collision Model", group_name.c_str());
        return false;
    }

    AllowedCollisionMatrix acm;
    initAllowedCollisionMatrix(config, acm);
    acm.print(std::cout);
    m_scm.setAllowedCollisionMatrix(acm);

    m_group_name = group_name;
    m_gidx = m_rcm.groupIndex(m_group_name);

    return true;
}

/// \brief Initialize the Collision Space
/// \param urdf_string string description of the robot in URDF format
/// \param group_name collision group for which collision detection is
///        performed
/// \param config collision model configuration
/// \param planning_joints The set of joint variable names being planned for
///        in the order they will appear in calls to isStateValid and
///        friends
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
        m_rcs.setJointVarPosition(joint_name, joint_position);
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


/// \brief Set a joint variable in the robot state
/// \return true if the joint variable exists; false otherwise
bool CollisionSpace::setJointPosition(
    const std::string& name,
    double position)
{
    if (m_rcm.hasJointVar(name)) {
        m_rcs.setJointVarPosition(name, position);
        return true;
    }
    else {
        return false;
    }
}

/// \brief Set the reference to robot model frame transform
void CollisionSpace::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    m_rcs.setWorldToModelTransform(transform);
}

/// \brief Set the padding applied to the collision model
void CollisionSpace::setPadding(double padding)
{
    m_wcm.setPadding(padding);
    m_scm.setPadding(padding);
}

void CollisionSpace::setAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    m_scm.setAllowedCollisionMatrix(acm);
}

bool CollisionSpace::insertObject(const ObjectConstPtr& object)
{
    return m_wcm.insertObject(object);
}

bool CollisionSpace::removeObject(const ObjectConstPtr& object)
{
    return m_wcm.removeObject(object);
}

bool CollisionSpace::removeObject(const std::string& object_name)
{
    return m_wcm.removeObject(object_name);
}

bool CollisionSpace::moveShapes(const ObjectConstPtr& object)
{
    return m_wcm.moveShapes(object);
}

bool CollisionSpace::insertShapes(const ObjectConstPtr& object)
{
    return m_wcm.insertShapes(object);
}

bool CollisionSpace::removeShapes(const ObjectConstPtr& object)
{
    return m_wcm.removeShapes(object);
}

bool CollisionSpace::processCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    return m_wcm.processCollisionObject(object);
}

bool CollisionSpace::processOctomapMsg(
    const octomap_msgs::OctomapWithPose& octomap)
{
    return m_wcm.insertOctomap(octomap);
}

bool CollisionSpace::attachObject(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name)
{
//    if (!m_rcm.attachBody(id, shapes, transforms, link_name)) {
//        return false;
//    }
//    updateAttachedBodyIndices();
//    return true;
}

bool CollisionSpace::removeAttachedObject(const std::string& id)
{
//    if (!m_rcm.detachBody(id)) {
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
    return m_wcm.getWorldVisualization();
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
    return m_wcm.getCollisionWorldVisualization();
}

visualization_msgs::MarkerArray
CollisionSpace::getCollisionRobotVisualization() const
{
    auto markers = m_rcs.getVisualization();
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

bool CollisionSpace::isStateValid(
    const std::vector<double>& angles,
    bool verbose,
    bool visualize,
    double& dist)
{
    // allow subroutines to update minimum distance
    dist = std::numeric_limits<double>::max();

    // update the robot state
    for (size_t i = 0; i < angles.size(); ++i) {
        int jidx = m_planning_joint_to_collision_model_indices[i];
        m_rcs.setJointVarPosition(jidx, angles[i]);
    }

    // world collisions are implicitly checked via the self collision model
    // since the two models share the same occupancy grid
//    bool robot_world_valid = checkRobotCollision(verbose, visualize, dist);
//    if (!visualize && !robot_world_valid) {
//        return false;
//    }

    bool robot_robot_valid = checkSelfCollision(verbose, visualize, dist);
    if (!visualize && !robot_robot_valid) {
        return false;
    }

    bool attached_object_world_valid = checkAttachedObjectCollision(
            verbose, visualize, dist);
    if (!visualize && !attached_object_world_valid) {
        return false;
    }

    return attached_object_world_valid && robot_robot_valid;
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

visualization_msgs::MarkerArray
CollisionSpace::getCollisionModelVisualization(const std::vector<double>& vals)
{
    // TODO: fill in input values
    return getCollisionRobotVisualization();
}

/// \brief Retrieve visualization of the collision space
///
/// The visualization_msgs::MarkerArray's contents vary depending on the
/// argument:
///
///     "world": markers representing all objects managed by the world
///     "collision_world": cube list representing all occupied cells checked
///             against the configured group
///     "robot": visualization of the robot model at its current state
///     "collision_robot": visualization of the robot collision model at its
///             current state
///     "collision_details": spheres representing the location of the most
///             recent collision check
///     "attached_object": spheres representing the bounding volumes of all
///             attached objects
///     <any argument excepted by OccupancyGrid::getVisualization>:
///         <the corresponding visualization provided by OccupancyGrid>
///
/// \param type The type of visualization to get
/// \return The visualization
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

bool CollisionSpace::setPlanningJoints(
    const std::vector<std::string>& joint_names)
{
    for (const std::string& joint_name : joint_names) {
        if (!m_rcm.hasJointVar(joint_name)) {
            ROS_ERROR_NAMED(CC_LOGGER, "Joint variable '%s' not found in Robot Collision Model", joint_name.c_str());
            return false;
        }
    }

    // map planning joint indices to collision model indices
    m_planning_joint_to_collision_model_indices.resize(joint_names.size(), -1);

    m_increments.resize(joint_names.size(), utils::ToRadians(2.0));
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];;
        int jidx = m_rcm.jointVarIndex(joint_name);

        m_planning_joint_to_collision_model_indices[i] = jidx;
    }

    return true;
}

void CollisionSpace::initAllowedCollisionMatrix(
    const CollisionModelConfig& config,
    AllowedCollisionMatrix& acm)
{
    // add allowed collisions between spheres on the same link
    for (const auto& spheres_config : config.spheres_models) {
        for (size_t i = 0; i < spheres_config.spheres.size(); ++i) {
            const std::string& s1_name = spheres_config.spheres[i].name;
            if (!acm.hasEntry(s1_name)) {
                ROS_INFO_NAMED(CC_LOGGER, "Adding entry '%s' to the ACM", s1_name.c_str());
                acm.setEntry(s1_name, false);
            }
            for (size_t j = i + 1; j < spheres_config.spheres.size(); ++j) {
                const std::string& s2_name = spheres_config.spheres[j].name;
                if (!acm.hasEntry(s2_name)) {
                    ROS_INFO_NAMED(CC_LOGGER, "Adding entry '%s' to the ACM", s2_name.c_str());
                    acm.setEntry(s2_name, false);
                }

                ROS_INFO_NAMED(CC_LOGGER, "Spheres '%s' and '%s' attached to the same link...allowing collision", s1_name.c_str(), s2_name.c_str());
                acm.setEntry(s1_name, s2_name, true);
            }
        }
    }

    // add in additional allowed collisions from config
    std::vector<std::string> config_entries;
    config.acm.getAllEntryNames(config_entries);
    for (size_t i = 0; i < config_entries.size(); ++i) {
        const std::string& entry1 = config_entries[i];
        if (!acm.hasEntry(entry1)) {
            ROS_WARN_NAMED(CC_LOGGER, "Configured allowed collision entry '%s' was not found in the collision model", entry1.c_str());
            continue;
        }
        for (size_t j = i; j < config_entries.size(); ++j) {
            const std::string& entry2 = config_entries[j];
            if (!acm.hasEntry(entry2)) {
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
                acm.setEntry(entry1, entry2, true);
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
//            m_rcs.groupSphereStateIndices(m_gidx);
//    std::vector<int> ovs_indices =
//            m_rcs.groupOutsideVoxelsStateIndices(m_gidx);
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

bool CollisionSpace::checkRobotCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
    return m_wcm.checkCollision(m_rcs, m_gidx, dist);
}

bool CollisionSpace::checkSelfCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
    return m_scm.checkCollision(m_rcs, m_gidx, dist);
}

bool CollisionSpace::checkAttachedObjectCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
    return false;
}

double CollisionSpace::isValidLineSegment(
    const std::vector<int>& a,
    const std::vector<int>& b,
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

} // namespace collision
} // namespace sbpl
