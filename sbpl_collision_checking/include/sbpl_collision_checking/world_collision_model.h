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

#ifndef sbpl_collision_world_collision_model_h
#define sbpl_collision_world_collision_model_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <moveit_msgs/CollisionObject.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_collision_checking/attached_bodies_collision_state.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/types.h>

namespace sbpl {
namespace collision {

class WorldCollisionModelImpl;

class WorldCollisionModel
{
public:

    WorldCollisionModel(OccupancyGrid* grid);
    WorldCollisionModel(const WorldCollisionModel& o, OccupancyGrid* grid);

    ~WorldCollisionModel();

    bool insertObject(const ObjectConstPtr& object);
    bool removeObject(const ObjectConstPtr& object);
    bool moveShapes(const ObjectConstPtr& object);
    bool insertShapes(const ObjectConstPtr& object);
    bool removeShapes(const ObjectConstPtr& object);

    bool processCollisionObject(const moveit_msgs::CollisionObject& object);
    bool insertOctomap(const octomap_msgs::OctomapWithPose& octomap);

    bool removeObject(const std::string& object_name);

    /// \brief Reset the underlying occupancy grid.
    ///
    /// Resets the WorldCollisionModel by clearing the underlying occupancy grid and
    /// revoxelizing all of the managed objects.
    void reset();

    visualization_msgs::MarkerArray getWorldVisualization() const;
    visualization_msgs::MarkerArray getCollisionWorldVisualization() const;

    void setPadding(double padding);

    bool checkCollision(
        RobotCollisionState& state,
        const std::string& group_name,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        const int gidx,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const std::string& group_name,
        double& dist);

    bool checkCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const int gidx,
        double& dist);

private:

    std::unique_ptr<WorldCollisionModelImpl> m_impl;
};

typedef std::shared_ptr<WorldCollisionModel> WorldCollisionModelPtr;
typedef std::shared_ptr<const WorldCollisionModel> WorldCollisionModelConstPtr;

} // namespace collision
} // namespace sbpl

#endif
