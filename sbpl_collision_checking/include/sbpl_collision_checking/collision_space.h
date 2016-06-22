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

#ifndef sbpl_collision_collision_space_h
#define sbpl_collision_collision_space_h

// standard includes
#include <string>
#include <memory>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <sbpl_arm_planner/collision_checker.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/world_collision_model.h>
#include <sbpl_collision_checking/types.h>

namespace sbpl {
namespace collision {

class CollisionSpace : public manip::CollisionChecker
{
public:

    CollisionSpace(OccupancyGrid* grid);
    ~CollisionSpace();

    bool init(
        const urdf::ModelInterface& urdf,
        const std::string& group_name,
        const CollisionModelConfig& config,
        const std::vector<std::string>& planning_joints);

    /// \brief Initialize the Collision Space
    /// \param urdf_string string description of the robot in URDF format
    /// \param group_name collision group for which collision detection is
    ///        performed
    /// \param config collision model configuration
    /// \param planning_joints The set of joint variable names being planned for
    ///        in the order they will appear in calls to isStateValid and
    ///        friends
    bool init(
        const std::string& urdf_string,
        const std::string& group_name,
        const CollisionModelConfig& config,
        const std::vector<std::string>& planning_joints);

    bool setPlanningScene(const moveit_msgs::PlanningScene& scene);

    /// \brief Return the reference frame of the occupancy grid
    const std::string& getReferenceFrame() const;

    /// \brief Return the group being collision checked
    const std::string& getGroupName() const;

    /// \name Robot Collision Model
    ///@{
    ///@}

    /// \name Robot Collision State
    ///@{

    /// \brief Set the padding applied to the collision model
    void setPadding(double padding);

    /// \brief Set a joint variable in the robot state
    /// \return true if the joint variable exists; false otherwise
    bool setJointPosition(const std::string& name, double position);

    /// \brief Set the reference to robot model frame transform
    void setWorldToModelTransform(const Eigen::Affine3d& transform);

    ///@}

    /// \name Self Collisions
    ///@{

    const collision_detection::AllowedCollisionMatrix&
    allowedCollisionMatrix() const;

    void setAllowedCollisionMatrix(
        const collision_detection::AllowedCollisionMatrix& acm);

    ///@}

    /// \name World
    ///@{
    bool insertObject(const ObjectConstPtr& object);
    bool removeObject(const ObjectConstPtr& object);
    bool moveShapes(const ObjectConstPtr& object);
    bool insertShapes(const ObjectConstPtr& object);
    bool removeShapes(const ObjectConstPtr& object);

    bool processCollisionObject(const moveit_msgs::CollisionObject& object);
    bool processOctomapMsg(const octomap_msgs::OctomapWithPose& octomap);

    bool removeObject(const std::string& object_name);
    ///@}

    /// \name Attached Objects
    ///@{

    void attachObject(const moveit_msgs::AttachedCollisionObject& obj);
    void removeAttachedObject();

    bool getAttachedObject(
        const std::vector<double>& angles,
        std::vector<std::vector<double>>& xyz);

    ///@}

    /// \name Visualization
    ///@{

    // THE DREAM
//    visualization_msgs::MarkerArray getWorldVisualization() const; // visualization of the world
//    visualization_msgs::MarkerArray getRobotVisualization() const; // visualization of the robot
//    visualization_msgs::MarkerArray getCollisionWorldVisualization() const; // visualization of the collision world
//    visualization_msgs::MarkerArray getCollisionRobotVisualization() const; // visualization of the collision robot
//    visualization_msgs::MarkerArray getCollisionDetailsVisualization() const; // visualization of collisions between world and robot

    visualization_msgs::MarkerArray getCollisionObjectsVisualization() const;
    visualization_msgs::MarkerArray getCollisionsVisualization() const;
    visualization_msgs::MarkerArray getCollisionObjectVoxelsVisualization() const;
    visualization_msgs::MarkerArray getBoundingBoxVisualization() const;
    visualization_msgs::MarkerArray getDistanceFieldVisualization() const;
    visualization_msgs::MarkerArray getOccupiedVoxelsVisualization() const;

    /// \brief Retrive visualization of the collision space
    ///
    /// The visualization_msgs::MarkerArray's contents vary depending on the
    /// argument:
    ///
    ///     "collision_objects": markers representing all collision objects
    ///     "collisions": spheres representing the collisions during the last
    ///         call to isStateValid
    ///     "collision_object_voxels": points representing all voxels occupied
    ///         by collision objects
    ///     <any argument excepted by OccupancyGrid::getVisualization>:
    ///         <the corresponding visualization provided by OccupancyGrid>
    ///
    /// \param type The type of visualization to get
    /// \return The visualization
    visualization_msgs::MarkerArray getVisualization(const std::string& type);

    visualization_msgs::MarkerArray getMeshModelVisualization(
        const std::string& group_name,
        const std::vector<double>& angles);

    ///@}

    /// \name Reimplemented Public Function
    ///@{
    bool isStateValid(
        const std::vector<double>& angles,
        bool verbose,
        bool visualize,
        double &dist);

    bool isStateToStateValid(
        const std::vector<double>& angles0,
        const std::vector<double>& angles1,
        int& path_length,
        int& num_checks,
        double& dist);

    bool interpolatePath(
        const std::vector<double>& start,
        const std::vector<double>& end,
        std::vector<std::vector<double>>& path);
    ///@}

private:

    OccupancyGrid* m_grid;
    WorldCollisionModel m_world;
    RobotCollisionModel m_model;

    // TODO: attached object variables that could probably be moved into the
    // robot collision model
    bool object_attached_;
    int attached_object_frame_num_;
    int attached_object_segment_num_;
    int attached_object_chain_num_;
    std::string attached_object_frame_;
    std::vector<Sphere> object_spheres_;
    double object_enclosing_sphere_radius_;

    // Collision Group
    std::string m_group_name;
    int m_group_index;
    std::vector<int> m_sphere_indices;
    std::vector<int> m_voxels_indices;

    // Planning Joint Information
    std::vector<int> m_planning_joint_to_collision_model_indices;
    std::vector<double> m_increments;
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool> m_continuous;

    // Collision Checking Policies
    collision_detection::AllowedCollisionMatrix m_acm;
    double m_padding;

    // Visualization
    std::vector<Sphere> m_collision_spheres; // cached between check and vis

    ////////////////////
    // Initialization //
    ////////////////////

    bool setPlanningJoints(const std::vector<std::string>& joint_names);

    ////////////////////
    // Self Collision //
    ////////////////////

    void updateVoxelsStates();

    void initAllowedCollisionMatrix(const CollisionModelConfig& config);
    bool findAttachedLink(
        const CollisionModelConfig& config,
        const std::string& sphere,
        std::string& link_name) const;

    //////////////////////
    // Attached Objects //
    //////////////////////

    void attachSphere(
        const std::string& name,
        const std::string& link,
        const geometry_msgs::Pose& pose,
        double radius);

    void attachCylinder(
        const std::string& link,
        const geometry_msgs::Pose& pose,
        double radius,
        double length);

    void attachCube(
        const std::string& name,
        const std::string& link,
        const geometry_msgs::Pose& pose,
        double x_dim,
        double y_dim,
        double z_dim);

    void attachMesh(
        const std::string& name,
        const std::string& link,
        const geometry_msgs::Pose& pose,
        const std::vector<geometry_msgs::Point>& vertices,
        const std::vector<int>& triangles);

    ////////////////////////
    // Collision Checking //
    ////////////////////////

    // TODO: THE DREAM is 3-4 variants of checkCollision. One to explicitly
    // check as fast as possible, with all shortcutting policies enabled; a
    // second for returning the nearest distance (or highest penetration
    // distance), a third for returning a minimal but complete representation of
    // the collision details (contact points, offending spheres, etc), and a
    // fourth for visualizations

    bool checkPathForCollision(
        const std::vector<double>& start,
        const std::vector<double>& end,
        bool verbose,
        int& path_length,
        int& num_checks,
        double& dist);

    bool checkRobotCollision(bool verbose, bool visualize, double& dist);
    bool checkSelfCollision(bool verbose, bool visualize, double& dist);
    bool checkAttachedObjectCollision();

    double isValidLineSegment(
        const std::vector<int> a,
        const std::vector<int> b,
        const int radius);

    bool getClearance(
        const std::vector<double>& angles,
        int num_spheres,
        double& avg_dist,
        double& min_dist);
};

inline
const std::string& CollisionSpace::getReferenceFrame() const
{
    return m_grid->getReferenceFrame();
}

inline
const std::string& CollisionSpace::getGroupName() const
{
    return m_group_name;
}

typedef std::shared_ptr<CollisionSpace> CollisionSpacePtr;
typedef std::shared_ptr<const CollisionSpace> CollisionSpaceConstPtr;

} // namespace collision
} // namespace sbpl

#endif
