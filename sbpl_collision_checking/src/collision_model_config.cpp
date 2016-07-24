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

static const char* XmlRpcValueTypeToString(const XmlRpc::XmlRpcValue::Type type)
{
    switch (type) {
    case XmlRpc::XmlRpcValue::Type::TypeInvalid:
        return "Invalid";
    case XmlRpc::XmlRpcValue::Type::TypeBoolean:
        return "Boolean";
    case XmlRpc::XmlRpcValue::Type::TypeInt:
        return "Int";
    case XmlRpc::XmlRpcValue::Type::TypeDouble:
        return "TypeDouble";
    case XmlRpc::XmlRpcValue::Type::TypeString:
        return "String";
    case XmlRpc::XmlRpcValue::Type::TypeDateTime:
        return "DateTime";
    case XmlRpc::XmlRpcValue::Type::TypeBase64:
        return "Base64";
    case XmlRpc::XmlRpcValue::Type::TypeArray:
        return "Array";
    case XmlRpc::XmlRpcValue::Type::TypeStruct:
        return "Struct";
    default:
        return "Unrecognized";
    }
}

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

template <typename T>
bool LoadConfigArray(
    XmlRpc::XmlRpcValue& all_configs,
    std::vector<T>& configs)
{
    std::vector<T> out;

    if (all_configs.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("param must be an array");
        return false;
    }

    if (all_configs.size() == 0) {
        ROS_WARN("No elements in array");
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

    configs = std::move(out);
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
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("spheres model config is malformed");
        return false;
    }

    if (!config.hasMember("link_name")) {
        ROS_ERROR("spheres model config must have have a 'link_name' element");
        return false;
    }

    XmlRpc::XmlRpcValue& link_name_value = config["link_name"];
    if (link_name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("spheres model config 'link_name' element must be a string");
        return false;
    }

    bool autogenerate = false;
    if (config.hasMember("auto")) {
        XmlRpc::XmlRpcValue& auto_value = config["auto"];
        if (auto_value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
            ROS_ERROR("spheres model config 'auto' element must be a boolean");
            return false;
        }

        autogenerate = (bool)auto_value;
    }

    double radius = 0.0;
    std::vector<CollisionSphereConfig> spheres;

    if (autogenerate) {
        // read in radius
        if (!config.hasMember("radius")) {
            ROS_ERROR("spheres model config with 'auto' element true must have a 'radius' element");
            return false;
        }
        XmlRpc::XmlRpcValue& radius_value = config["radius"];
        if (!IsNumeric(radius_value)) {
            ROS_ERROR("spheres model config 'radius' element must be numeric");
            return false;
        }
        radius = ToDouble(radius_value);
    }
    else {
        // read in spheres
        if (!config.hasMember("spheres")) {
            ROS_ERROR("spheres model config with [implicit] 'auto' element false must have a 'spheres' element");
            return false;
        }

        if (!LoadConfigArray(config["spheres"], spheres)) {
            ROS_ERROR("Failed to load Collision Sphere Model Config");
            return false;
        }
    }

    cfg.link_name = (std::string)link_name_value;
    cfg.autogenerate = autogenerate;
    cfg.radius = radius;
    cfg.spheres = std::move(spheres);
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
        ROS_ERROR("group config is malformed. expected { name: <name>, links: [...] }");
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
            ROS_ERROR("links config is malformed. expected { name: <name> }");
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

static
bool LoadAllowedCollisionsConfig(
    XmlRpc::XmlRpcValue& all_entries,
    collision_detection::AllowedCollisionMatrix& acm)
{
    if (all_entries.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("param is not an array");
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

    return true;
}

bool CollisionModelConfig::Load(
    const ros::NodeHandle& nh,
    CollisionModelConfig& cfg)
{
    XmlRpc::XmlRpcValue robot_collision_model_config;
    if (!nh.getParam("robot_collision_model", robot_collision_model_config)) {
        ROS_ERROR("Failed to retrieve 'robot_collision_model' from the param server");
        return false;
    }

    return Load(robot_collision_model_config, cfg);
}

bool CollisionModelConfig::Load(
    XmlRpc::XmlRpcValue& config,
    CollisionModelConfig& cfg)
{
    std::vector<CollisionSpheresModelConfig> spheres_models_config;
    std::vector<CollisionVoxelModelConfig> voxels_models_config;
    std::vector<CollisionGroupConfig> groups_config;
    collision_detection::AllowedCollisionMatrix acm;

    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("robot_collision_model config is malformed (Type: %s, Expected: %s)", XmlRpcValueTypeToString(config.getType()), XmlRpcValueTypeToString(XmlRpc::XmlRpcValue::Type::TypeString));
        return false;
    }

    const std::string spheres_models_param_name = "spheres_models";
    const std::string voxel_models_param_name = "voxel_models";
    const std::string groups_param_name = "collision_groups";
    const std::string acm_param_name = "allowed_collisions";

    if (config.hasMember("spheres_models")) {
        if (!LoadConfigArray(config["spheres_models"], spheres_models_config)) {
            ROS_ERROR("Failed to load Collision Spheres Model Configs");
            return false;
        }
    }
    else {
        ROS_WARN("No param 'spheres_models' found on the param server");
    }

    if (config.hasMember("voxels_models")) {
        if (!LoadConfigArray(config["voxels_models"], voxels_models_config)) {
            ROS_ERROR("Failed to load Collision Voxels Model Configs");
            return false;
        }
    }
    else {
        ROS_WARN("No param 'voxels_model' found on the param server");
    }

    if (config.hasMember("collision_groups")) {
        if (!LoadConfigArray(config["collision_groups"], groups_config)) {
            ROS_ERROR("Failed to load Collision Group Configs");
            return false;
        }
    }
    else {
        ROS_WARN("No param 'collision_groups' found on the param server");
    }

    if (config.hasMember("allowed_collisions")) {
        if (!LoadAllowedCollisionsConfig(config["allowed_collisions"], acm)) {
            ROS_ERROR("Failed to load Allowed Collision Matrix");
            return false;
        }
    }
    else {
        ROS_WARN("No param 'allowed_collisions' found on the param server");
    }

    // TODO: check references to spheres in collision_groups?

    cfg.spheres_models = std::move(spheres_models_config);
    cfg.voxel_models = std::move(voxels_models_config);
    cfg.groups = std::move(groups_config);
    cfg.acm = acm;
    return true;
}

} // namespace collision
} // namespace sbpl
