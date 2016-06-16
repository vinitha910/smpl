////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
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

#include <sbpl_collision_checking/collision_model_config.h>

// standard includes
#include <limits>
#include <sstream>
#include <utility>

namespace sbpl {
namespace collision {

bool IsNumeric(const XmlRpc::XmlRpcValue& value)
{
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
            value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
}

double ToDouble(XmlRpc::XmlRpcValue& value)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return (double)((int)value);
    }
    else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return (double)value;
    }
    else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

bool CollisionLinkConfig::Load(
    XmlRpc::XmlRpcValue& config,
    CollisionLinkConfig& cfg)
{
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config.hasMember("name") ||
        !config.hasMember("root"))
        // note: spheres member optional
    {
        ROS_WARN("collision_links config is malformed");
        return false;
    }

    XmlRpc::XmlRpcValue& name_value = config["name"];
    XmlRpc::XmlRpcValue& root_value = config["root"];

    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Collision link config 'name' element must be a string");
        return false;
    }

    if (root_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Collision link config 'root' element must be a string");
        return false;
    }

    cfg.name = (std::string)name_value;
    cfg.root = (std::string)root_value;

    // TODO: value checking? enforce spheres if parent group type is 'spheres'?

    if (config.hasMember("spheres")) {
        XmlRpc::XmlRpcValue& spheres_value = config["spheres"];
        if (spheres_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_WARN("Collision link config 'spheres' value must be a string");
            return false;
        }

        std::stringstream ss((std::string)spheres_value);
        std::string sphere_name;
        while (ss >> sphere_name) {
            cfg.spheres.push_back(sphere_name);
        }
    }

    return true;
}

bool CollisionGroupConfig::Load(
    XmlRpc::XmlRpcValue& config,
    CollisionGroupConfig& cfg)
{
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config.hasMember("name") ||
        !config.hasMember("type") ||
        !config.hasMember("root_name") ||
        !config.hasMember("tip_name") ||
        !config.hasMember("collision_links"))
    {
        ROS_WARN("Group config is malformed");
        return false;
    }

    XmlRpc::XmlRpcValue& name_value = config["name"];
    XmlRpc::XmlRpcValue& type_value = config["type"];
    XmlRpc::XmlRpcValue& root_name_value = config["root_name"];
    XmlRpc::XmlRpcValue& tip_name_value = config["tip_name"];
    XmlRpc::XmlRpcValue& collision_links_value = config["collision_links"];

    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Collision group config 'name' element must be a string");
        return false;
    }
    if (type_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Collision group config 'type' element must be a string");
        return false;
    }
    if (root_name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Collision group config 'root_name' element must be a string");
        return false;
    }
    if (tip_name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Collision group config 'tip_name' element must be a string");
        return false;
    }
    if (collision_links_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("Collision group config 'collision_links' element must be an array");
        return false;
    }

    // TODO: value checking?

    cfg.name = (std::string)name_value;
    cfg.type = (std::string)type_value;
    cfg.root_name = (std::string)root_name_value;
    cfg.tip_name = (std::string)tip_name_value;
    cfg.collision_links.reserve(collision_links_value.size());

    // read in collision links
    for (int j = 0; j < collision_links_value.size(); ++j) {
        XmlRpc::XmlRpcValue& link_config = collision_links_value[j];
        CollisionLinkConfig collision_link_config;
        if (!CollisionLinkConfig::Load(link_config, collision_link_config)) {
            ROS_WARN("Failed to load collision link config");
            return false;
        }

        cfg.collision_links.push_back(std::move(collision_link_config));
    }

    return true;
}

bool CollisionSphereConfig::Load(
    XmlRpc::XmlRpcValue& config,
    CollisionSphereConfig& cfg)
{
    // check sphere config structure
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config.hasMember("name") ||
        !config.hasMember("x") ||
        !config.hasMember("y") ||
        !config.hasMember("z") ||
        !config.hasMember("priority") ||
        !config.hasMember("radius"))
    {
        ROS_WARN("Sphere config is malformed");
        return false;
    }

    // type check sphere config elements

    XmlRpc::XmlRpcValue& name_value = config["name"];
    XmlRpc::XmlRpcValue& x_value = config["x"];
    XmlRpc::XmlRpcValue& y_value = config["y"];
    XmlRpc::XmlRpcValue& z_value = config["z"];
    XmlRpc::XmlRpcValue& radius_value = config["radius"];
    XmlRpc::XmlRpcValue& priority_value = config["priority"];

    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN("Sphere config 'name' element must be a string");
        return false;
    }
    if (!IsNumeric(x_value)) {
        ROS_WARN("Sphere config 'x' element must be numeric");
        return false;
    }
    if (!IsNumeric(y_value)) {
        ROS_WARN("Sphere config 'y' element must be numeric");
        return false;
    }
    if (!IsNumeric(z_value)) {
        ROS_WARN("Sphere config 'z' element must be numeric");
        return false;
    }
    if (!IsNumeric(radius_value)) {
        ROS_WARN("Sphere config 'radius' element must be numeric");
        return false;
    }
    if (priority_value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_WARN("Sphere config 'priority' element must be an integer");
        return false;
    }

    // TODO: value checking?

    cfg.name = (std::string)name_value;
    cfg.x = ToDouble(x_value);
    cfg.y = ToDouble(y_value);
    cfg.z = ToDouble(z_value);
    cfg.radius = ToDouble(radius_value);
    cfg.priority = (int)priority_value;
    return true;
}

bool CollisionModelConfig::Load(
    const ros::NodeHandle& nh,
    CollisionModelConfig& cfg)
{
    std::vector<CollisionSphereConfig> spheres_config;
    std::vector<CollisionSpheresModelConfig> spheres_models_config;
    std::vector<CollisionVoxelModelConfig> voxel_models_config;
    std::vector<CollisionGroupConfig> groups_config;

    // read in collision_spheres
    const std::string spheres_param_name = "collision_spheres";
    if (nh.hasParam(spheres_param_name)) {
        XmlRpc::XmlRpcValue all_spheres;
        nh.getParam(spheres_param_name, all_spheres);

        if (all_spheres.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("param '%s' must be an array", spheres_param_name.c_str());
            return false;
        }

        if (all_spheres.size() == 0) {
            ROS_WARN("No spheres in %s", spheres_param_name.c_str());
        }

        spheres_config.reserve(all_spheres.size());
        for (int i = 0; i < all_spheres.size(); ++i) {
            XmlRpc::XmlRpcValue& sphere_config = all_spheres[i];
            CollisionSphereConfig collision_sphere_config;
            if (!CollisionSphereConfig::Load(
                    sphere_config, collision_sphere_config))
            {
                ROS_ERROR("Failed to load collision sphere config");
                return false;
            }
            spheres_config.push_back(std::move(collision_sphere_config));
        }
    }
    else {
        ROS_WARN("No param '%s' on the param server", spheres_param_name.c_str());
    }

    // read in spheres models
    const std::string spheres_models_param_name = "spheres_models";
    if (nh.hasParam(spheres_models_param_name)) {
        XmlRpc::XmlRpcValue all_spheres_models;
        nh.getParam(spheres_models_param_name, all_spheres_models);

        if (all_spheres_models.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("param '%s' must be an array", spheres_models_param_name.c_str());
            return false;
        }

        if (all_spheres_models.size() == 0) {
            ROS_WARN("No spheres models in '%s'", spheres_models_param_name.c_str());
        }

        spheres_models_config.reserve(all_spheres_models.size());
        for (int i = 0; i < all_spheres_models.size(); ++i) {
            XmlRpc::XmlRpcValue& spheres_models_config = all_spheres_models[i];
            CollisionSpheresModelConfig collision_spheres_models_config;
            if (!CollisionSpheresModelConfig::Load(
                    spheres_model_config, collision_spheres_models_config))
            {
                ROS_ERROR("Faile to load collision spheres model config");
                return false;
            }
            spheres_models_config.push_back(std::move(collision_spheres_model_config));
        }
    }
    else {
        ROS_WARN("No param '%s' on the param server")
    }

    // read in collision_groups

    const std::string groups_param_name = "collision_groups";
    if (!nh.hasParam(groups_param_name)) {
        ROS_WARN_STREAM("No groups for planning specified in " << groups_param_name);
        return false;
    }

    XmlRpc::XmlRpcValue all_groups;
    nh.getParam(groups_param_name, all_groups);

    if (all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("param '%s' is not an array", groups_param_name.c_str());
        return false;
    }

    if (all_groups.size() == 0) {
        ROS_WARN("No groups in groups");
        return false;
    }

    std::vector<CollisionGroupConfig> collision_groups_config;
    collision_groups_config.reserve(all_groups.size());
    for (int i = 0; i < all_groups.size(); ++i) {
        XmlRpc::XmlRpcValue& group_config = all_groups[i];
        CollisionGroupConfig collision_group_config;
        if (!CollisionGroupConfig::Load(group_config, collision_group_config)) {
            ROS_WARN("Failed to load collision group config");
            return false;
        }
        collision_groups_config.push_back(collision_group_config);
    }

    // TODO: check references to spheres in collision_groups?

    const std::string acm_param_name = "allowed_collisions";
    collision_detection::AllowedCollisionMatrix acm;
    if (nh.hasParam(acm_param_name)) {
        XmlRpc::XmlRpcValue all_entries;
        nh.getParam(acm_param_name, all_entries);

        if (all_entries.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("param '%s' is not an array", acm_param_name.c_str());
            return false;
        }

        for (int i = 0; i < all_entries.size(); ++i) {
            XmlRpc::XmlRpcValue& entry_config = all_entries[i];
            if (entry_config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN("Allowed collision entry is not a struct");
                return false;
            }

            if (!entry_config.hasMember("sphere1") || !entry_config.hasMember("sphere2")) {
                ROS_WARN("Allowed collision entry is missing 'sphere1' or 'sphere2' fields");
                return false;
            }

            if (entry_config["sphere1"].getType() != XmlRpc::XmlRpcValue::TypeString ||
                entry_config["sphere2"].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_WARN("'sphere1' and 'sphere2' fields must be strings");
                return false;
            }

            std::string sphere1_value = entry_config["sphere1"];
            std::string sphere2_value = entry_config["sphere2"];
            acm.setEntry(sphere1_value, sphere2_value, true);
        }
    }

    // TODO: check references to spheres?

    cfg.collision_groups = std::move(collision_groups_config);
    cfg.collision_spheres = std::move(collision_spheres_config);
    cfg.acm = acm;
    return true;
}

} // namespace collision
} // namespace sbpl
