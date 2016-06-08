////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
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

// system includes
#include <ros/ros.h>

// project includes
#include <sbpl_collision_checking/sbpl_collision_model.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sbpl_collision_model_visualizer");
    sbpl::collision::SBPLCollisionModel model;
    ros::NodeHandle ph("~");
    sleep(1);
    std::string group_name;
    std::vector<std::string> joint_names(7);
    ph.param<std::string>("group_name", group_name, "");
    ph.param<std::string>("joint_0", joint_names[0], "");
    ph.param<std::string>("joint_1", joint_names[1], "");
    ph.param<std::string>("joint_2", joint_names[2], "");
    ph.param<std::string>("joint_3", joint_names[3], "");
    ph.param<std::string>("joint_4", joint_names[4], "");
    ph.param<std::string>("joint_5", joint_names[5], "");
    ph.param<std::string>("joint_6", joint_names[6], "");

    // remove empty joint names (for arms with fewer than 7 joints)
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i].empty()) {
            joint_names.erase(joint_names.begin()+i);
        }
    }

    if (!model.init()) {
        ROS_ERROR("[test] Model failed to initialize.");
    }

    if (!model.initGroup(group_name)) {
        ROS_ERROR("[test] Model failed to initialize group '%s'.", group_name.c_str());
    }

    //  model.printGroups();

    std::vector<double> min_limits, max_limits;
    std::vector<bool> continuous;
    std::vector<std::string> joint_names;

    if (!model.getJointLimits("base_link", "link7", joint_names, min_limits, max_limits, continuous)) {
        return false;
    }

    // group names
    std::vector<std::string> group_names;
    model.getGroupNames(group_names);
    ROS_INFO("[test] number of groups: %d", int(group_names.size()));
    for (size_t i = 0; i < group_names.size(); ++i) {
        ROS_INFO("[test] [%d] group_name: %s", int(i), group_names[i].c_str());
    }

    std::vector<std::string> names(7);
    names[0] = "joint2_shoulder_pitch";
    names[1] = "joint6_wrist_pitch";
    names[2] = "joint3_arm_roll";
    names[3] = "joint1_shoulder_yaw";
    names[4] = "joint7_wrist_roll";
    names[5] = "joint5_forearm_roll";
    names[6] = "joint4_elbow";

    ROS_INFO("[test] Setting the order of the joint positions now.");
    model.setOrderOfJointPositions(joint_names, group_name);
    model.setDefaultGroup(group_name);
    //model.printDebugInfo(group_name);

    std::vector<double> angles(7,0);
    angles[0] = 0.7;
    angles[1] = 0.3;
    angles[2] = 0.0;
    angles[3] = 0.5;
    angles[4] = 0.6;
    angles[5] = 0.8;
    angles[6] = 0.4;

    std::vector<std::vector<KDL::Frame> > frames;
    if (!model.computeDefaultGroupFK(angles, frames)) {
        ROS_ERROR("[test] FK Solver failed");
    }

    model.printFrames(frames);
    //model.printGroups();
    ROS_INFO("Done");
    return 0;
}
