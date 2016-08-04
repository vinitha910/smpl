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

// standard includes
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <random>

// system includes
#include <ros/ros.h>

#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_collision_checking/collision_space.h>
#include <urdf/model.h>

sbpl::OccupancyGridPtr CreateGrid(const ros::NodeHandle& nh)
{
    const char* world_collision_model_param = "world_collision_model";
    std::string wcm_key;
    if (!nh.searchParam(world_collision_model_param, wcm_key)) {
        ROS_ERROR("Failed to find 'world_collision_model' key on the param server");
        return sbpl::OccupancyGridPtr();
    }

    XmlRpc::XmlRpcValue wcm_config;
    if (!nh.getParam(wcm_key, wcm_config)) {
        ROS_ERROR("Failed to retrieve '%s' from the param server", wcm_key.c_str());
        return sbpl::OccupancyGridPtr();
    }

    if (wcm_config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !wcm_config.hasMember("frame") ||
        !wcm_config.hasMember("size_x") ||
        !wcm_config.hasMember("size_y") ||
        !wcm_config.hasMember("size_z") ||
        !wcm_config.hasMember("origin_x") ||
        !wcm_config.hasMember("origin_y") ||
        !wcm_config.hasMember("origin_z") ||
        !wcm_config.hasMember("res_m") ||
        !wcm_config.hasMember("max_distance_m"))
    {
        ROS_ERROR("'%s' param is malformed", world_collision_model_param);
        ROS_ERROR_STREAM("has size_x member " << wcm_config.hasMember("size_x"));
        ROS_ERROR_STREAM("has size_y member " << wcm_config.hasMember("size_y"));
        ROS_ERROR_STREAM("has size_z member " << wcm_config.hasMember("size_z"));
        ROS_ERROR_STREAM("has origin_x member " << wcm_config.hasMember("origin_x"));
        ROS_ERROR_STREAM("has origin_y member " << wcm_config.hasMember("origin_y"));
        ROS_ERROR_STREAM("has origin_z member " << wcm_config.hasMember("origin_z"));
        ROS_ERROR_STREAM("has res_m member " << wcm_config.hasMember("res_m"));
        ROS_ERROR_STREAM("has max_distance_m member " << wcm_config.hasMember("max_distance_m"));
        return sbpl::OccupancyGridPtr();
    }

    const std::string world_frame = "odom_combined";
    const double size_x = wcm_config["size_x"];
    const double size_y = wcm_config["size_y"];
    const double size_z = wcm_config["size_z"];
    const double res_m = wcm_config["res_m"];
    const double origin_x = wcm_config["origin_x"];
    const double origin_y = wcm_config["origin_y"];
    const double origin_z = wcm_config["origin_z"];
    const double max_distance_m = wcm_config["max_distance_m"];
    const bool propagate_negative_distances = false;
    const bool ref_counted = true;

    auto grid = std::make_shared<sbpl::OccupancyGrid>(
            size_x, size_y, size_z,
            res_m,
            origin_x, origin_y, origin_z,
            propagate_negative_distances,
            ref_counted);
    return grid;
}

template <typename RNG>
std::vector<double> CreateRandomVariables(
    const sbpl::collision::RobotCollisionModel& rcm,
    const std::vector<std::string>& var_names,
    RNG& rng)
{
    std::vector<double> out;
    out.reserve(var_names.size());
    for (const std::string& var_name : var_names) {
        if (rcm.jointVarIsContinuous(var_name)) {
            std::uniform_real_distribution<double> dist(-M_PI, M_PI);
            out.push_back(dist(rng));

        }
        else if (!rcm.jointVarHasPositionBounds(var_name)) {
            std::uniform_real_distribution<double> dist;
            out.push_back(dist(rng));
        }
        else {
            std::uniform_real_distribution<double> dist(
                    rcm.jointVarMinPosition(var_name),
                    rcm.jointVarMaxPosition(var_name));
            out.push_back(dist(rng));
        }
    }
    return out;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "benchmark");
    ros::NodeHandle nh;

    const double time_limit = argc > 1 ? std::atof(argv[1]) : 10.0;
    if (time_limit == 0.0) {
        ROS_WARN("Did you make a mistake?");
    }

    auto grid = CreateGrid(nh);

    std::string robot_description_key;
    if (!nh.searchParam("robot_description", robot_description_key)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    urdf::Model urdf;
    if (!urdf.initParam(robot_description_key)) {
        ROS_ERROR("Failed to initialize URDF from parameter '%s'", robot_description_key.c_str());
        return 1;
    }

    if (urdf.getName() != "pr2") {
        ROS_ERROR("This benchmark is intended for the PR2 robot");
        return 1;
    }

    sbpl::collision::CollisionModelConfig config;
    if (!sbpl::collision::CollisionModelConfig::Load(nh, config)) {
        ROS_ERROR("Failed to load collision model config");
        return 1;
    }

    auto rcm = sbpl::collision::RobotCollisionModel::Load(urdf, config);

    const std::string group_name = "right_arm";

    // hardcoded joint names corresponding to 'right_arm' joint group from SRDF
    const std::vector<std::string> planning_joints = {
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint",
    };

    sbpl::collision::CollisionSpaceBuilder builder;
    auto cspace = builder.build(grid.get(), rcm, group_name, planning_joints);

    if (!cspace) {
        ROS_ERROR("Failed to build Collision Space");
        return 1;
    }

    std::default_random_engine rng;

    ROS_INFO("Begin collision check benchmarking");

    int check_count = 0;
    double elapsed = 0.0;
    while (elapsed < time_limit) {
        std::vector<double> variables = CreateRandomVariables(*rcm, planning_joints, rng);
        double dist;
        auto start = std::chrono::high_resolution_clock::now();
        bool res = cspace->isStateValid(variables, false, false, dist);
        auto finish = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration<double>(finish - start).count();
        ++check_count;
    }

    ROS_INFO("check count: %d", check_count);
    ROS_INFO("checks / second: %0.3f", check_count / elapsed);
    ROS_INFO("seconds / check: %0.3f", elapsed / check_count);

    return 0;
}

