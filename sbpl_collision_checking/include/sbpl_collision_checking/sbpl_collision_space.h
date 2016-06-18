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
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <kdl/kdl.hpp>
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
#include <sbpl_collision_checking/collision_world.h>
#include <sbpl_collision_checking/sbpl_collision_model.h>
#include <sbpl_collision_checking/group.h>

namespace sbpl {
namespace collision {

class CollisionSpace : public manip::CollisionChecker
{
public:

    typedef CollisionWorld::Object Object;
    typedef CollisionWorld::ObjectPtr ObjectPtr;
    typedef CollisionWorld::ObjectConstPtr ObjectConstPtr;

    CollisionSpace(OccupancyGrid* grid);
    ~CollisionSpace();

    /// \name manip::CollisionChecker API Requirements
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

    /// \brief Initialize the collision space
    /// \param urdf_string string description of the robot in URDF format
    /// \param group_name The collision group for which collision detection is
    ///         performed
    /// \param config Configuration of the collision model
    /// \param planning_joints The list of joint names being planned for. Just
    ///         set this to the list of parent joints of links in the collision
    ///         group and you will be happy.
    bool init(
        const std::string& urdf_string,
        const std::string& group_name,
        const CollisionModelConfig& config,
        const std::vector<std::string>& planning_joints);

    bool setPlanningScene(const moveit_msgs::PlanningScene& scene);

    const std::string& getReferenceFrame() const {
        return grid_->getReferenceFrame();
    }

    const std::string& getGroupName() const { return group_name_; }

    /// \name Collision Robot Model
    ///@{
    ///@}

    /// \name Collision Robot State
    ///@{

    /// \brief Set the padding applied to the collision group model
    void setPadding(double padding);

    void setJointPosition(const std::string& name, double position);

    void setWorldToModelTransform(const Eigen::Affine3d& transform);

    bool getCollisionSpheres(
        const std::vector<double>& angles,
        std::vector<std::vector<double>>& spheres);

    ///@}

    /// \name Self Collisions
    ///@{

    const collision_detection::AllowedCollisionMatrix&
    allowedCollisionMatrix() const;

    void setAllowedCollisionMatrix(
        const collision_detection::AllowedCollisionMatrix& acm);

    bool updateVoxelGroups();

    ///@}

    /// \name Generic Objects
    ///@{

    bool insertObject(const CollisionWorld::ObjectConstPtr& object);
    bool removeObject(const CollisionWorld::ObjectConstPtr& object);
    bool removeObject(const std::string& object_name);
    bool moveShapes(const CollisionWorld::ObjectConstPtr& object);
    bool insertShapes(const CollisionWorld::ObjectConstPtr& object);
    bool removeShapes(const CollisionWorld::ObjectConstPtr& object);

    /// @}

    /// \name Collision Objects
    ///@{

    bool processCollisionObject(const moveit_msgs::CollisionObject& object);

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

    visualization_msgs::MarkerArray getCollisionModelVisualization(
        const std::vector<double>& angles);

    visualization_msgs::MarkerArray getMeshModelVisualization(
        const std::string& group_name,
        const std::vector<double>& angles);

    ///@}

private:

    ///////////////////////////////
    // Collision World Variables //
    ///////////////////////////////

    CollisionWorld m_world;
    OccupancyGrid* grid_;

    ///////////////////////////////
    // Collision Robot Variables //
    ///////////////////////////////

    RobotCollisionModel model_;
    std::string group_name_;
    double padding_;
    double object_enclosing_sphere_radius_;
    std::vector<double> inc_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<bool> continuous_;
    std::vector<const Sphere*> spheres_; // temp
    std::vector<std::vector<KDL::Frame>> frames_; // temp

    collision_detection::AllowedCollisionMatrix m_acm;

    ////////////////////////////////
    // Attached Objects Varibles //
    ////////////////////////////////

    bool object_attached_;
    int attached_object_frame_num_;
    int attached_object_segment_num_;
    int attached_object_chain_num_;
    std::string attached_object_frame_;
    std::vector<Sphere> object_spheres_;

    // cached between collision check and visualization
    std::vector<Sphere> collision_spheres_;

    ////////////////////
    // Initialization //
    ////////////////////

    bool setPlanningJoints(const std::vector<std::string>& joint_names);

    ////////////////////
    // Self Collision //
    ////////////////////

    void initAllowedCollisionMatrix(const CollisionModelConfig& config);
    bool findAttachedLink(
        const CollisionModelConfig& config,
        const std::string& sphere,
        std::string& link_name) const;
    bool updateVoxelGroup(Group* g);
    bool updateVoxelGroup(const std::string& name);

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

    bool checkCollision(
        const std::vector<double>& angles,
        bool verbose,
        bool visualize,
        double& dist);

    bool checkPathForCollision(
        const std::vector<double>& start,
        const std::vector<double>& end,
        bool verbose,
        int& path_length,
        int& num_checks,
        double& dist);

    bool isValidCell(int x, int y, int z, int radius) const;

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

} // namespace collision
} // namespace sbpl

#endif
