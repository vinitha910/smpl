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
#include <sbpl_geometry_utils/utils.h>

namespace sbpl {
namespace collision {

static const char* CC_LOGGER = "cspace";

CollisionSpace::~CollisionSpace()
{
}

/// \brief Set the planning scene
/// \param scene The scene
/// \return true if the scene was updated correctly; false otherwise
bool CollisionSpace::setPlanningScene(const moveit_msgs::PlanningScene& scene)
{
    ROS_INFO_NAMED(CC_LOGGER, "Updating the Collision Space from Planning Scene '%s'", scene.name.c_str());

    // TODO: currently ignored fields from moveit_msgs::PlanningScene
    // * fixed_frame_transforms
    // * link_padding
    // * link_scale
    // * object_colors

    //////////////////////
    // robot model name //
    //////////////////////

    if (scene.robot_model_name != m_rcm->name()) {
        ROS_ERROR_NAMED(CC_LOGGER, "Planning Scene passed to CollisionSpace::setPlanningScene is not for this robot");
        return false;
    }

    // TODO: for full planning scenes, remove all collision objects from the
    // world collision model
    if (!scene.is_diff) {
        ROS_ERROR_NAMED(CC_LOGGER, "Collision space does not support complete planning scene updates");
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
        if (m_rcm->hasJointVar(joint_name)) {
            double joint_position = joint_state.position[i];
            int jidx = m_rcm->jointVarIndex(joint_name);
            m_joint_vars[jidx] = joint_position;
        }
        else {
            ROS_WARN("joint variable '%s' not found within the robot collision model", joint_name.c_str());
        }
    }

    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state =
            robot_state.multi_dof_joint_state;
    for (size_t i = 0; i < multi_dof_joint_state.joint_names.size(); ++i) {
        // TODO: handle multi-dof joint state
    }

    // TODO: handle world -> model transform by looking up the transform from
    // the occupancy grid reference frame to the robot collision model frame

    const auto& attached_collision_objects =
            robot_state.attached_collision_objects;
    for (const auto& aco : attached_collision_objects) {
        if (!processAttachedCollisionObject(aco)) {
            ROS_ERROR_NAMED(CC_LOGGER, "Failed to process attached collision object");
            return false;
        }
    }

    //////////////////////////////
    // allowed collision matrix //
    //////////////////////////////

    AllowedCollisionMatrix acm(scene.allowed_collision_matrix);
    if (scene.is_diff) {
        m_scm->updateAllowedCollisionMatrix(acm);
    }
    else {
        m_scm->setAllowedCollisionMatrix(acm);
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
            return false;
        }
    }

    const auto& octomap = planning_scene_world.octomap;
    if (!processOctomapMsg(octomap)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to process octomap '%s'", octomap.octomap.id.c_str());
        return false;
    }

    return true;
}

/// \brief Set a joint variable in the robot state
/// \param The joint variable name
/// \param The joint variable position
/// \return true if the joint variable exists; false otherwise
bool CollisionSpace::setJointPosition(
    const std::string& name,
    double position)
{
    if (m_rcm->hasJointVar(name)) {
        int jidx = m_rcm->jointVarIndex(name);
        m_joint_vars[jidx] = position;
        return true;
    }
    else {
        return false;
    }
}

/// \brief Set the transform from the reference frame to the robot model frame
/// \param transform The transform from the reference frame to the robot frame
void CollisionSpace::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    m_rcs->setWorldToModelTransform(transform);
}

/// \brief Set the padding applied to the collision model
void CollisionSpace::setPadding(double padding)
{
    m_wcm->setPadding(padding);
    m_scm->setPadding(padding);
}

/// \brief Return the allowed collision matrix
/// \return The allowed collision matrix
const AllowedCollisionMatrix& CollisionSpace::allowedCollisionMatrix() const
{
    return m_scm->allowedCollisionMatrix();
}

/// \brief Update the allowed collisoin matrix
/// \param acm The allowed collision matrix
void CollisionSpace::updateAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    return m_scm->updateAllowedCollisionMatrix(acm);
}

/// \brief Set the allowed collision matrix
/// \param acm The allowed collision matrix
void CollisionSpace::setAllowedCollisionMatrix(
    const AllowedCollisionMatrix& acm)
{
    m_scm->setAllowedCollisionMatrix(acm);
}

/// \brief Insert an object into the world
/// \param object The object
/// \return true if the object was inserted; false otherwise
bool CollisionSpace::insertObject(const ObjectConstPtr& object)
{
    return m_wcm->insertObject(object);
}

/// \brief Remove an object from the world
/// \param object The object
/// \return true if the object was removed; false otherwise
bool CollisionSpace::removeObject(const ObjectConstPtr& object)
{
    return m_wcm->removeObject(object);
}

/// \brief Remove an object from the world
/// \param object_name The name of the object
/// \return true if the object was removed; false otherwise
bool CollisionSpace::removeObject(const std::string& object_name)
{
    return m_wcm->removeObject(object_name);
}

/// \brief Move an object in the world
/// \param object The object to be moved, with its new shapes
/// \return true if the object was moved; false otherwise
bool CollisionSpace::moveShapes(const ObjectConstPtr& object)
{
    return m_wcm->moveShapes(object);
}

/// \brief Append shapes to an object
/// \param object The object to append shapes to, with its new shapes
/// \return true if the shapes were appended to the object; false otherwise
bool CollisionSpace::insertShapes(const ObjectConstPtr& object)
{
    return m_wcm->insertShapes(object);
}

/// \brief Remove shapes from an object
/// \param object The object to remove shapes from, with the shapes to be removed
/// \return true if the shapes were removed; false otherwise
bool CollisionSpace::removeShapes(const ObjectConstPtr& object)
{
    return m_wcm->removeShapes(object);
}

/// \brief Process a collision object
/// \param object The collision object to be processed
/// \return true if the object was processed successfully; false otherwise
bool CollisionSpace::processCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    return m_wcm->processCollisionObject(object);
}

/// \brief Process an octomap
/// \param octomap The octomap
/// \return true if the octomap was processed successfully; false otherwise
bool CollisionSpace::processOctomapMsg(
    const octomap_msgs::OctomapWithPose& octomap)
{
    return m_wcm->insertOctomap(octomap);
}

/// \brief Attach a collision object to the robot
/// \param id The name of the object
/// \param shapes The shapes composing the object
/// \param transforms The pose of each shape with respect to the attached link
/// \param link_name The link to attach the object to
/// \return true if the object was attached; false otherwise
bool CollisionSpace::attachObject(
    const std::string& id,
    const std::vector<shapes::ShapeConstPtr>& shapes,
    const Affine3dVector& transforms,
    const std::string& link_name)
{
    return m_abcm->attachBody(id, shapes, transforms, link_name);
}

/// \brief Detach a collision object from the robot
/// \param id The name of the object
/// \return true if the object was detached; false otherwise
bool CollisionSpace::detachObject(const std::string& id)
{
    return m_abcm->detachBody(id);
}

/// \brief Process an attached collision object
/// \param ao The attached collision object
/// \return true if the attached collision object was processed successfully;
///     false otherwise
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
        return true;
    }   break;
    case moveit_msgs::CollisionObject::REMOVE:
    {
        if (!detachObject(ao.object.id)) {
            return false;
        }
        return true;
    }   break;
    case moveit_msgs::CollisionObject::APPEND:
    case moveit_msgs::CollisionObject::MOVE:
    default:
        throw std::runtime_error("unimplemented");
    }

    return false;
}

/// \brief Return a visualization of the current world
///
/// The visualization is of the set of collision object geometries
visualization_msgs::MarkerArray
CollisionSpace::getWorldVisualization() const
{
    return m_wcm->getWorldVisualization();
}

/// \brief Return a visualization of the current robot state
///
/// The visualization is of the visual robot geometry
visualization_msgs::MarkerArray
CollisionSpace::getRobotVisualization() const
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

/// \brief Return a visualization of the current collision world
///
/// The visualization is of the occupied voxels in the grid
visualization_msgs::MarkerArray
CollisionSpace::getCollisionWorldVisualization() const
{
    return m_wcm->getCollisionWorldVisualization();
}

/// \brief Return a visualization of the current collision robot state
///
/// The visualization is of the robot bounding geometry
visualization_msgs::MarkerArray
CollisionSpace::getCollisionRobotVisualization() const
{
    auto markers = m_rcs->getVisualization();
    for (auto& m : markers.markers) {
        m.header.frame_id = getReferenceFrame();
    }
    return markers;
}

/// \brief Return a visualization of the collision robot at a given state
visualization_msgs::MarkerArray
CollisionSpace::getCollisionRobotVisualization(
    const std::vector<double>& vals)
{
    updateState(vals);
    // update the spheres within the group
    for (int ssidx : m_rcs->groupSpheresStateIndices(m_gidx)) {
        m_rcs->updateSphereStates(ssidx);
    }
    auto markers = m_rcs->getVisualization(m_gidx);
    for (auto& m : markers.markers) {
        m.header.frame_id = m_grid->getReferenceFrame();
    }
    return markers;
}

/// \brief Return a visualization of the collision details for the current state
visualization_msgs::MarkerArray
CollisionSpace::getCollisionDetailsVisualization() const
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

/// \brief Return a visualization of the collision details for a given state
visualization_msgs::MarkerArray
CollisionSpace::getCollisionDetailsVisualization(
    const std::vector<double>& vals)
{
    // TODO: implement me
    return visualization_msgs::MarkerArray();
}

/// \brief Return a visualization of the bounding box
visualization_msgs::MarkerArray
CollisionSpace::getBoundingBoxVisualization() const
{
    return m_grid->getBoundingBoxVisualization();
}

/// \brief Return a visualization of the distance field
visualization_msgs::MarkerArray
CollisionSpace::getDistanceFieldVisualization() const
{
    return m_grid->getDistanceFieldVisualization();
}

/// \brief Return a visualization of the occupied voxels
visualization_msgs::MarkerArray
CollisionSpace::getOccupiedVoxelsVisualization() const
{
    return m_grid->getOccupiedVoxelsVisualization();
}

bool CollisionSpace::checkCollision(
    const std::vector<double>& vars,
    const AllowedCollisionsInterface& aci,
    bool verbose,
    bool visualize,
    double& dist)
{
    // allow subroutines to update minimum distance
    dist = std::numeric_limits<double>::max();

    updateState(vars);

    // NOTE: see comment in isStateValid

    bool robot_robot_valid = m_scm->checkCollision(*m_rcs, aci, m_gidx, dist);
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

bool CollisionSpace::checkCollision(
    const std::vector<double>& vars,
    double& dist)
{
    updateState(vars);
    return m_scm->checkCollision(*m_rcs, m_gidx, dist);
}

double CollisionSpace::collisionDistance(const std::vector<double>& vars)
{
    updateState(vars);
    return m_scm->collisionDistance(*m_rcs, m_gidx);
}

bool CollisionSpace::collisionDetails(
    const std::vector<double>& vars,
    CollisionDetails& details)
{
    updateState(vars);
    return m_scm->collisionDetails(*m_rcs, m_gidx, details);
}

bool CollisionSpace::isStateValid(
    const std::vector<double>& angles,
    bool verbose,
    bool visualize,
    double& dist)
{
    // allow subroutines to update minimum distance
    dist = std::numeric_limits<double>::max();

    updateState(angles);

    // world collisions are implicitly checked via the self collision model
    // since the two models share the same occupancy grid
//    bool robot_world_vlaid = return m_wcm->checkCollision(*m_rcs, m_gidx, dist);
//    if (!visualize && !robot_world_valid) {
//        return false;
//    }

    bool robot_robot_valid = m_scm->checkCollision(*m_rcs, m_gidx, dist);
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
    return getCollisionRobotVisualization(vals);
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

CollisionSpace::CollisionSpace() :
    m_grid(),
    m_rcm(),
    m_abcm(),
    m_rcs(),
    m_abcs(),
    m_wcm(),
    m_scm(),
    m_group_name(),
    m_gidx(-1),
    m_planning_joint_to_collision_model_indices(),
    m_increments()
{
}

/// \brief Initialize the Collision Space
/// \param urdf_string String description of the robot in URDF format
/// \param config Collision model configuration
/// \param group_name The group for which collision detection is performed
/// \param planning_joints The set of joint variable names in the order they
///     will appear in calls to isStateValid and friends
bool CollisionSpace::init(
    OccupancyGrid* grid,
    const std::string& urdf_string,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    auto urdf = boost::make_shared<urdf::Model>();
    if (!urdf->initString(urdf_string)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to parse URDF");
        return false;
    }

    return init(grid, *urdf, config, group_name, planning_joints);
}

/// \brief Initialize the Collision Space
/// \param urdf The URDF of the robot
/// \param config Collision model configuration
/// \param group_name Group for which collision detection is performed
/// \param planning_joints The set of joint variable names in the order they
///     will appear in calls to isStateValid and friends
bool CollisionSpace::init(
    OccupancyGrid* grid,
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    ROS_DEBUG_NAMED(CC_LOGGER, "Initializing collision space for group '%s'", group_name.c_str());

    auto rcm = RobotCollisionModel::Load(urdf, config);
    return init(grid, rcm, group_name, planning_joints);
}

/// \brief Initialize the Collision Space
/// \param rcm The robot collision model
/// \param group_name The group for which collision detection is performed
/// \param planning_joints The set of joint variable names in the order they
///     will appear in calls to isStateValid and friends
bool CollisionSpace::init(
    OccupancyGrid* grid,
    const RobotCollisionModelConstPtr& rcm,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    m_grid = grid;
    m_rcm = rcm;
    if (!m_rcm) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to initialize the Robot Collision Model");
        return false;
    }

    if (!setPlanningJoints(planning_joints)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Failed to set planning joints");
        return false;
    }

    if (!m_rcm->hasGroup(group_name)) {
        ROS_ERROR_NAMED(CC_LOGGER, "Group '%s' was not found in the Robot Collision Model", group_name.c_str());
        return false;
    }

    m_group_name = group_name;
    m_gidx = m_rcm->groupIndex(m_group_name);

    m_abcm = std::make_shared<AttachedBodiesCollisionModel>(m_rcm.get()),
    m_rcs = std::make_shared<RobotCollisionState>(m_rcm.get());
    m_abcs = std::make_shared<AttachedBodiesCollisionState>(m_abcm.get(), m_rcs.get());
    m_wcm = std::make_shared<WorldCollisionModel>(m_grid);
    m_scm = std::make_shared<SelfCollisionModel>(m_grid, m_rcm.get(), m_abcm.get());

    m_joint_vars.assign(
        m_rcs->getJointVarPositions(),
        m_rcs->getJointVarPositions() + m_rcm->jointVarCount());

    return true;
}

/// \brief Set the joint variables in the order they appear to isStateValid calls
bool CollisionSpace::setPlanningJoints(
    const std::vector<std::string>& joint_names)
{
    for (const std::string& joint_name : joint_names) {
        if (!m_rcm->hasJointVar(joint_name)) {
            ROS_ERROR_NAMED(CC_LOGGER, "Joint variable '%s' not found in Robot Collision Model", joint_name.c_str());
            return false;
        }
    }

    // map planning joint indices to collision model indices
    m_planning_joint_to_collision_model_indices.resize(joint_names.size(), -1);

    m_increments.resize(joint_names.size(), utils::ToRadians(2.0));
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const std::string& joint_name = joint_names[i];;
        int jidx = m_rcm->jointVarIndex(joint_name);

        m_planning_joint_to_collision_model_indices[i] = jidx;
    }

    return true;
}

void CollisionSpace::updateState(const std::vector<double>& vals)
{
    // update the robot state
    for (size_t i = 0; i < vals.size(); ++i) {
        int jidx = m_planning_joint_to_collision_model_indices[i];
        m_joint_vars[jidx] = vals[i];
    }
    copyState();
}

void CollisionSpace::copyState()
{
    m_rcs->setJointVarPositions(m_joint_vars.data());
}

/// \brief Check whether the planning joint variables are within limits
/// \return true if all variables are within limits; false otherwise
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

/// \brief TODO
/// \return TODO
bool CollisionSpace::checkAttachedObjectCollision(
    bool verbose,
    bool visualize,
    double& dist)
{
    return true;
}

/// \brief Return the distance from a line segment to the nearest occupied cell
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

CollisionSpacePtr CollisionSpaceBuilder::build(
    OccupancyGrid* grid,
    const std::string& urdf_string,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    CollisionSpacePtr cspace(new CollisionSpace);
    if (cspace->init(grid, urdf_string, config, group_name, planning_joints)) {
        return cspace;
    }
    else {
        return CollisionSpacePtr();
    }
}

CollisionSpacePtr CollisionSpaceBuilder::build(
    OccupancyGrid* grid,
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    CollisionSpacePtr cspace(new CollisionSpace);
    if (cspace->init(grid, urdf, config, group_name, planning_joints)) {
        return cspace;
    }
    else {
        return CollisionSpacePtr();
    }
}

CollisionSpacePtr CollisionSpaceBuilder::build(
    OccupancyGrid* grid,
    const RobotCollisionModelConstPtr& rcm,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints)
{
    CollisionSpacePtr cspace(new CollisionSpace);
    if (cspace->init(grid, rcm, group_name, planning_joints)) {
        return cspace;
    }
    else {
        return CollisionSpacePtr();
    }
}

} // namespace collision
} // namespace sbpl
