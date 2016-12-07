////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen
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

#ifndef sbpl_collision_TestSBPLCollisionSpace_h
#define sbpl_collision_TestSBPLCollisionSpace_h

// standard includes
#include <iostream>
#include <map>

// system includes
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Pose.h>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kinematics_msgs/GetPositionFK.h>
#include <message_filters/subscriber.h>
#include <moveit_msgs/CollisionMap.h>
#include <moveit_msgs/CollisionObject.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <ros/ros.h>
#include <smpl/occupancy_grid.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// project includes
#include <sbpl_collision_checking/sbpl_collision_space.h>

namespace sbpl {
namespace collision {

class TestSBPLCollisionSpace
{
public:

    /** \brief Constructor-doesn't initialize class, you need to run init() */
    TestSBPLCollisionSpace();

    ~TestSBPLCollisionSpace();

    /** \brief Initialize the planner node. */
    bool init();

    /** \brief Run the node! */
    int run();

private:

    ros::NodeHandle node_handle_, root_handle_;
    ros::Subscriber object_subscriber_;
    ros::Subscriber joint_states_subscriber_;
    message_filters::Subscriber<moveit_msgs::CollisionMap> collision_map_subscriber_;
    tf::MessageFilter<moveit_msgs::CollisionMap>* collision_map_filter_;

    //remove
    ros::Subscriber collision_object_subscriber_;
    void collisionObjectCallback(
        const moveit_msgs::CollisionObjectConstPtr& collision_object);
    std::vector<std::vector<double> > col_objects_;

    std::string left_fk_service_name_;
    std::string left_ik_service_name_;
    std::string right_fk_service_name_;
    std::string right_ik_service_name_;

    /* params */
    int throttle_;
    int num_joints_;

    std::string collision_map_topic_;
    std::string node_name_;
    std::string reference_frame_;
    std::string map_frame_;
    std::string arm_name_;
    std::vector<std::string> ljoint_names_;
    std::vector<std::string> rjoint_names_;
    std::map<std::string, moveit_msgs::CollisionObject> object_map_;

    std::vector<double> angles_;

    bool verbose_;

    /** \brief size of collision space */
    double sizeX_;
    double sizeY_;
    double sizeZ_;

    /** \brief origin of collision space */
    double originX_;
    double originY_;
    double originZ_;

    /* planner & environment */
    double resolution_;
    CollisionSpace* cspace_;
    OccupancyGrid* grid_;
    smpl::VisualizeArm* aviz_;
    KDL::Frame kdl_transform_;

    boost::mutex colmap_mutex_;
    boost::mutex object_mutex_;

    /* transforms and kinematics */
    tf::TransformListener tf_;
    tf::StampedTransform transform_;

    void updateMapFromCollisionMap(
        const moveit_msgs::CollisionMapConstPtr& collision_map);

    void collisionMapCallback(
        const moveit_msgs::CollisionMapConstPtr& collision_map);

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& state);

    void visualizeCollisionObjects();
};

} // namespace collision
} // namespace sbpl

#endif
