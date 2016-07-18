////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen, Andrew Dornbush

// standard includes
#include <iostream>
#include <string>

// system includes
#include <geometric_shapes/shapes.h>
#include <leatherman/print.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_collision_model");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);

    ////////////////////////////////////////////
    // inspect configuration loaded from yaml //
    ////////////////////////////////////////////

    ROS_WARN("Loading Configuration");

    sbpl::collision::CollisionModelConfig config;
    if (!sbpl::collision::CollisionModelConfig::Load(ph, config)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    ROS_INFO("Successfully loaded Collision Model Config");

    ROS_INFO("Spheres:");
    for (const auto& sphere : config.spheres) {
        ROS_INFO_STREAM("  " << sphere);
    }

    ROS_INFO("Spheres Models:");
    for (const auto& spheres_model : config.spheres_models) {
        ROS_INFO_STREAM("  " << spheres_model);
    }

    ROS_INFO("Voxel Models:");
    for (const auto& voxel_model : config.voxel_models) {
        ROS_INFO_STREAM("  " << voxel_model);
    }

    ROS_INFO("Groups:");
    for (const auto& group : config.groups) {
        ROS_INFO_STREAM("  " << group);
    }

    ROS_INFO("Allowed Collision Matrix:");
    config.acm.print(std::cout);

    ROS_WARN("Initialize Robot Model");

    std::string robot_description_param;
    if (!nh.searchParam("robot_description", robot_description_param)) {
        ROS_ERROR("Failed to find param 'robot_description' on the param server");
        return 1;
    }

    std::string robot_description;
    nh.getParam(robot_description_param, robot_description);

    sbpl::collision::RobotCollisionModel model;
    auto urdf = boost::make_shared<urdf::Model>();
    if (!urdf->initString(robot_description)) {
        ROS_ERROR("Failed to parse URDF");
        return 1;
    }

    if (urdf->getName() != "ur5") {
        ROS_ERROR("This test is intended for the UR5 robot");
        return 1;
    }

    if (!model.init(*urdf, config)) {
        ROS_ERROR("Failed to initialize Robot Collision Model");
        return 1;
    }

    sbpl::collision::RobotCollisionState state(&model);

    visualization_msgs::MarkerArray prev_ma;
    auto publish_model_viz = [&]()
    {
        // delete previously published markers
        for (auto& marker : prev_ma.markers) {
            marker.action = visualization_msgs::Marker::DELETE;
        }
        vis_pub.publish(prev_ma);

        auto ma = state.getVisualization("manipulator");
        for (auto& marker : ma.markers) {
            marker.header.frame_id = "world";
        }

        vis_pub.publish(ma);
        prev_ma = std::move(ma); // record these for deletion later

        // let the publisher do its job
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    };

    ROS_WARN("Publishing Zero State Visualization");

    /////////////////////////////////////////////
    // publish visualization of the zero state //
    /////////////////////////////////////////////

    // let ros and the publisher set up
    ros::Duration(1.0).sleep();

    for (size_t jidx = 0; jidx < model.jointVarCount(); ++jidx) {
        state.setJointPosition(jidx, 0.0);
    }

    state.updateSphereStates();
    publish_model_viz();

    ROS_WARN("Publishing Modified State Visualization");

    state.setJointPosition(0, 0.8);
    state.updateSphereStates();
    publish_model_viz();

    ROS_WARN("Attaching Cylinder and Publishing Visualization");

    std::vector<shapes::ShapeConstPtr> shapes;
    sbpl::collision::Affine3dVector transforms;

    auto ao_shape = boost::make_shared<const shapes::Cylinder>(0.10, 0.20);

    shapes.push_back(ao_shape);
    transforms.push_back(Eigen::Affine3d::Identity());

    const std::string attach_link = "ee_link";
    const std::string attached_body_id = "ao1";
    if (!model.attachBody(attached_body_id, shapes, transforms, attach_link)) {
        ROS_ERROR("Failed to attach body to '%s'", attach_link.c_str());
        return 1;
    }

    ROS_INFO("Attached Body Count %zu", model.attachedBodyCount());
    ROS_INFO("Has Attached Body(%s): %s", attached_body_id.c_str(), model.hasAttachedBody(attached_body_id.c_str()) ? "true" : "false");
    const int abidx = model.attachedBodyIndex(attached_body_id);
    ROS_INFO("Attached Body Index: %d", abidx);
    ROS_INFO("Attached Body Name(%d): %s", abidx, model.attachedBodyName(abidx).c_str());
    ROS_INFO("Attached Body Indices: %s", to_string(model.attachedBodyIndices(attach_link)).c_str());

    state.updateSphereStates();
    publish_model_viz();

    ROS_WARN("Detaching Cylinder and Publishing Visualization");

    if (!model.detachBody(attached_body_id)) {
        ROS_ERROR("Failed to detach body '%s'", attached_body_id.c_str());
        return 1;
    }

    {
        ROS_INFO("Attached Body Count %zu", model.attachedBodyCount());
        ROS_INFO("Has Attached Body(%s): %s", attached_body_id.c_str(), model.hasAttachedBody(attached_body_id.c_str()) ? "true" : "false");
        ROS_INFO("Attached Body Indices: %s", to_string(model.attachedBodyIndices(attach_link)).c_str());
    }

    state.updateSphereStates();
    publish_model_viz();

    return 0;
}
