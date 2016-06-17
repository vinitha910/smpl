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

// system includes
#include <leatherman/print.h>

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
        ROS_ERROR("sphere config is malformed");
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
        ROS_ERROR("sphere config 'name' element must be a string");
        return false;
    }
    if (!IsNumeric(x_value)) {
        ROS_ERROR("sphere config 'x' element must be numeric");
        return false;
    }
    if (!IsNumeric(y_value)) {
        ROS_ERROR("sphere config 'y' element must be numeric");
        return false;
    }
    if (!IsNumeric(z_value)) {
        ROS_ERROR("sphere config 'z' element must be numeric");
        return false;
    }
    if (!IsNumeric(radius_value)) {
        ROS_ERROR("sphere config 'radius' element must be numeric");
        return false;
    }
    if (priority_value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_ERROR("sphere config 'priority' element must be an integer");
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

std::ostream& operator<<(std::ostream& o, const CollisionSphereConfig& c)
{
    o << "{ name: " << c.name << ", x: " << c.x << ", y: " << c.y << ", z: " <<
            c.z << ", radius: " << c.radius << ", priority: " << c.priority <<
            " }";
    return o;
}

bool CollisionSpheresModelConfig::Load(
    XmlRpc::XmlRpcValue& config,
    CollisionSpheresModelConfig& cfg)
{
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config.hasMember("link_name") ||
        !config.hasMember("spheres"))
    {
        ROS_ERROR("spheres model config is malformed");
        return false;
    }

    XmlRpc::XmlRpcValue& link_name_value = config["link_name"];
    XmlRpc::XmlRpcValue& spheres_value = config["spheres"];

    if (link_name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("spheres model config 'link_name' element must be a string");
        return false;
    }

    if (spheres_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("spheres model config 'spheres' element must be an array");
        return false;
    }

    cfg.link_name = (std::string)link_name_value;

    std::stringstream ss((std::string)spheres_value);
    std::string sphere_name;
    while (ss >> sphere_name) {
        cfg.spheres.push_back(sphere_name);
    }

    return true;
}

std::ostream& operator<<(std::ostream& o, const CollisionSpheresModelConfig& c)
{
    o << "{ link_name: " << c.link_name << ", spheres: " << c.spheres << " }";
    return o;
}

bool CollisionVoxelModelConfig::Load(
    XmlRpc::XmlRpcValue& config,
    CollisionVoxelModelConfig& cfg)
{
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config.hasMember("link_name"))
    {
        ROS_ERROR("voxel model config is malformed");
        return false;
    }

    XmlRpc::XmlRpcValue& link_name_value = config["link_name"];

    if (link_name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("voxel model config 'link_name' element must be a string");
        return false;
    }

    cfg.link_name = (std::string)link_name_value;

    return true;
}

std::ostream& operator<<(std::ostream& o, const CollisionVoxelModelConfig& c)
{
    o << "{ link_name: " << c.link_name << " }";
    return o;
}

bool CollisionGroupConfig::Load(
    XmlRpc::XmlRpcValue& config,
    CollisionGroupConfig& cfg)
{
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config.hasMember("name") ||
        !config.hasMember("links"))
    {
        ROS_ERROR("group config is malformed");
        return false;
    }

    XmlRpc::XmlRpcValue& name_value = config["name"];
    XmlRpc::XmlRpcValue& links_value = config["links"];

    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("group config 'name' element must be a string");
        return false;
    }

    if (links_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("group config 'links' element must be an array");
        return false;
    }


    std::vector<std::string> group_links;
    group_links.reserve(links_value.size());
    for (int i = 0; i < links_value.size(); ++i) {
        XmlRpc::XmlRpcValue& link_value = links_value[i];
        if (link_value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !link_value.hasMember("name"))
        {
            ROS_ERROR("links config is malformed");
            return false;
        }

        XmlRpc::XmlRpcValue& link_name_value = link_value["name"];
        if (link_name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("links config 'name' element must be a string");
            return false;
        }

        group_links.push_back((std::string)link_name_value);
    }

    cfg.name = (std::string)name_value;
    cfg.links = std::move(group_links);

    return true;
}

std::ostream& operator<<(std::ostream& o, const CollisionGroupConfig& c)
{
    o << "{ name: " << c.name << ", links: " << c.links << " }";
    return o;
}

template <typename T>
bool LoadConfigArray(
    const ros::NodeHandle& nh,
    const std::string& param_name,
    std::vector<T>& configs)
{
    std::vector<T> out;
    if (nh.hasParam(param_name)) {
        XmlRpc::XmlRpcValue all_configs;
        nh.getParam(param_name, all_configs);

        if (all_configs.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("param '%s' must be an array", param_name.c_str());
            return false;
        }

        if (all_configs.size() == 0) {
            ROS_WARN("No spheres in %s", param_name.c_str());
        }

        out.reserve(all_configs.size());
        for (int i = 0; i < all_configs.size(); ++i) {
            XmlRpc::XmlRpcValue& config = all_configs[i];
            T collision_config;
            if (!T::Load(config, collision_config)) {
                ROS_ERROR("Failed to load collision sphere config");
                return false;
            }
            out.push_back(std::move(collision_config));
        }
    }
    else {
        ROS_WARN("No param '%s' on the param server", param_name.c_str());
    }

    configs = std::move(out);
    return true;
}

static
bool LoadCollisionSphereConfigs(
    const ros::NodeHandle& nh,
    std::vector<CollisionSphereConfig>& configs)
{
    const std::string spheres_param_name = "collision_spheres";
    return LoadConfigArray(nh, spheres_param_name, configs);
}

static
bool LoadCollisionSpheresModelConfigs(
    const ros::NodeHandle& nh,
    std::vector<CollisionSpheresModelConfig>& configs)
{
    const std::string spheres_models_param_name = "spheres_models";
    return LoadConfigArray(nh, spheres_models_param_name, configs);
}

static
bool LoadCollisionVoxelModelConfigs(
    const ros::NodeHandle& nh,
    std::vector<CollisionVoxelModelConfig>& configs)
{
    const std::string voxel_models_param_name = "voxel_models";
    return LoadConfigArray(nh, voxel_models_param_name, configs);
}

static
bool LoadCollisionGroupConfigs(
    const ros::NodeHandle& nh,
    std::vector<CollisionGroupConfig>& configs)
{
    const std::string groups_param_name = "collision_groups";
    return LoadConfigArray(nh, groups_param_name, configs);
}

static
bool LoadAllowedCollisionsConfig(
    const ros::NodeHandle& nh,
    collision_detection::AllowedCollisionMatrix& acm)
{
    const std::string acm_param_name = "allowed_collisions";
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
    else {
        ROS_WARN("No param '%s' found on the param server", acm_param_name.c_str());
    }

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
    collision_detection::AllowedCollisionMatrix acm;

    if (!LoadCollisionSphereConfigs(nh, spheres_config) ||
        !LoadCollisionSpheresModelConfigs(nh, spheres_models_config) ||
        !LoadCollisionVoxelModelConfigs(nh, voxel_models_config) ||
        !LoadCollisionGroupConfigs(nh, groups_config) ||
        !LoadAllowedCollisionsConfig(nh, acm))
    {
        // errors printed within Load* functions
        return false;
    }

    // TODO: check references to spheres in collision_groups?

    cfg.spheres = std::move(spheres_config);
    cfg.spheres_models = std::move(spheres_models_config);
    cfg.voxel_models = std::move(voxel_models_config);
    cfg.groups = std::move(groups_config);
    cfg.acm = acm;
    return true;
}

} // namespace collision
} // namespace sbpl
