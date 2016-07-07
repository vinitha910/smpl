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

    const RobotCollisionModel& robotCollisionModel() const;

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

    /// \name World Collision Model
    ///@{

    const WorldCollisionModel& worldCollisionModel() const;

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

    bool attachObject(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        const std::string& link_namme);

    bool removeAttachedObject(const std::string& id);

    bool processAttachedCollisionObject(
        const moveit_msgs::AttachedCollisionObject& obj);

    ///@}

    /// \name Visualization
    ///@{

    visualization_msgs::MarkerArray getWorldVisualization() const;
    visualization_msgs::MarkerArray getRobotVisualization() const;

    visualization_msgs::MarkerArray getCollisionWorldVisualization() const;
    visualization_msgs::MarkerArray getCollisionRobotVisualization() const;

    visualization_msgs::MarkerArray getCollisionDetailsVisualization(
        const std::vector<double>& vals) const;

    visualization_msgs::MarkerArray getBoundingBoxVisualization() const;
    visualization_msgs::MarkerArray getDistanceFieldVisualization() const;
    visualization_msgs::MarkerArray getOccupiedVoxelsVisualization() const;

    ///@}

    /// \name Reimplemented Public Functions
    ///@{
    bool isStateValid(
        const std::vector<double>& angles,
        bool verbose,
        bool visualize,
        double &dist) override;

    bool isStateToStateValid(
        const std::vector<double>& start,
        const std::vector<double>& finish,
        int& path_length,
        int& num_checks,
        double& dist) override;

    bool interpolatePath(
        const std::vector<double>& start,
        const std::vector<double>& finish,
        std::vector<std::vector<double>>& path) override;

    visualization_msgs::MarkerArray getCollisionModelVisualization(
        const std::vector<double>& vals) override;

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
    visualization_msgs::MarkerArray getVisualization(
        const std::string& type) override;

    ///@}

private:

    OccupancyGrid* m_grid;
    WorldCollisionModel m_world;
    RobotCollisionModel m_model;

    // Collision Group
    std::string m_group_name;
    int m_group_index;
    std::vector<int> m_sphere_indices;
    std::vector<int> m_voxels_indices;

    // Planning Joint Information
    std::vector<int> m_planning_joint_to_collision_model_indices;
    std::vector<double> m_increments;

    // Collision Checking Policies
    collision_detection::AllowedCollisionMatrix m_acm;
    double m_padding;

    // Visualization
    std::vector<Sphere> m_collision_spheres; // cached between check and vis

    ////////////////////
    // Initialization //
    ////////////////////

    bool setPlanningJoints(const std::vector<std::string>& joint_names);
    size_t planningVariableCount() const;

    ///////////////////////////////////
    // Planning Variable Information //
    ///////////////////////////////////

    bool isContinuous(int vidx) const;
    bool hasLimit(int vidx) const;
    double minLimit(int vidx) const;
    double maxLimit(int vidx) const;

    ////////////////////
    // Self Collision //
    ////////////////////

    void updateVoxelsStates();

    void initAllowedCollisionMatrix(const CollisionModelConfig& config);
    bool findAttachedLink(
        const CollisionModelConfig& config,
        const std::string& sphere,
        std::string& link_name) const;

    ////////////////////////
    // Collision Checking //
    ////////////////////////

    // TODO: THE DREAM is 3-4 variants of checkCollision. One to explicitly
    // check as fast as possible, with all shortcutting policies enabled; a
    // second for returning the nearest distance (or highest penetration
    // distance), a third for returning a minimal but complete representation of
    // the collision details (contact points, offending spheres, etc), and a
    // fourth for visualizations

    bool withinJointPositionLimits(const std::vector<double>& positions) const;

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

inline
const RobotCollisionModel& CollisionSpace::robotCollisionModel() const
{
    return m_model;
}

inline
const WorldCollisionModel& CollisionSpace::worldCollisionModel() const
{
    return m_world;
}

inline
size_t CollisionSpace::planningVariableCount() const
{
    return m_planning_joint_to_collision_model_indices.size();
}

inline
bool CollisionSpace::isContinuous(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_model.jointVarIsContinuous(jidx);
}

inline
bool CollisionSpace::hasLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_model.jointVarHasPositionBounds(jidx);
}

inline
double CollisionSpace::minLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_model.jointVarMaxPosition(jidx);
}

inline
double CollisionSpace::maxLimit(int vidx) const
{
    const int jidx = m_planning_joint_to_collision_model_indices[vidx];
    return m_model.jointVarMinPosition(jidx);
}

typedef std::shared_ptr<CollisionSpace> CollisionSpacePtr;
typedef std::shared_ptr<const CollisionSpace> CollisionSpaceConstPtr;

} // namespace collision
} // namespace sbpl

#endif
