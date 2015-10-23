/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#ifndef _SBPL_COLLISION_MODEL_
#define _SBPL_COLLISION_MODEL_

#include <ros/ros.h>
#include <iostream>
#include <map>
#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <urdf/model.h>
#include <sbpl_collision_checking/group.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <moveit_msgs/RobotState.h>

namespace sbpl {
namespace manipulation {

class CollisionModelImpl;

} // namespace manipulation
} // namespace sbpl

namespace sbpl_arm_planner {

/// \brief Represents the collision model of the robot used for planning.
class SBPLCollisionModel
{
public:

    SBPLCollisionModel();
    ~SBPLCollisionModel();

    bool init(const std::string& urdf_string);

    bool initAllGroups();

    void getGroupNames(std::vector<std::string>& names);

    bool getJointLimits(
        const std::string& group_name,
        const std::string& joint_name,
        double& min_limit,
        double& max_limit,
        bool& continuous);

    bool setDefaultGroup(const std::string& group_name);

    void getDefaultGroupSpheres(std::vector<Sphere*>& spheres);

    void getVoxelGroups(std::vector<Group*>& vg);

    bool computeDefaultGroupFK(
        const std::vector<double>& angles,
        std::vector<std::vector<KDL::Frame>>& frames);

    bool computeGroupFK(
        const std::vector<double>& angles,
        Group* group,
        std::vector<std::vector<KDL::Frame>>& frames);

    void setOrderOfJointPositions(
        const std::vector<std::string>& joint_names,
        const std::string& group_name);

    void setJointPosition(const std::string& name, double position);

    bool getFrameInfo(
        const std::string& name,
        const std::string& group_name,
        int& chain,
        int& segment);

    bool doesLinkExist(const std::string& name, const std::string& group_name);

    std::string getReferenceFrame(const std::string& group_name);

    Group* getGroup(const std::string& name);

    void printGroups();

    void printDebugInfo(const std::string& group_name);

    bool setModelToWorldTransform(
        const moveit_msgs::RobotState& state,
        const std::string& world_frame);

private:

    std::unique_ptr<sbpl::manipulation::CollisionModelImpl> impl_;
};

} // namespace sbpl_arm_planner

#endif
