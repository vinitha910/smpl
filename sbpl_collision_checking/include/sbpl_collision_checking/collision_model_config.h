////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef sbpl_collision_CollisionModelConfig_h
#define sbpl_collision_CollisionModelConfig_h

// standard includes
#include <string>
#include <vector>

// system includes
#include <moveit/collision_detection/collision_matrix.h>
#include <ros/ros.h>

namespace sbpl {
namespace collision {

struct CollisionLinkConfig
{
    /// \brief name of the collision link
    std::string name;

    /// \brief name of the link this collision link is attached to
    std::string root;

    /// \brief list of sphere names that are part of this collision link
    std::vector<std::string> spheres;

    static bool Load(XmlRpc::XmlRpcValue& config, CollisionLinkConfig& cfg);
};

struct CollisionGroupConfig
{
    std::string name;
    std::string type; // "voxels" or "spheres"
    std::string root_name;
    std::string tip_name; // TODO: find out why this is necessary
    std::vector<CollisionLinkConfig> collision_links;

    static bool Load(XmlRpc::XmlRpcValue& config, CollisionGroupConfig& cfg);
};

struct CollisionSphereConfig
{
    std::string name;
    double x;
    double y;
    double z;
    double radius;
    int priority;

    static bool Load(XmlRpc::XmlRpcValue& config, CollisionSphereConfig& cfg);
};

struct CollisionModelConfig
{
    std::vector<CollisionGroupConfig> collision_groups;
    std::vector<CollisionSphereConfig> collision_spheres;
    collision_detection::AllowedCollisionMatrix acm;

    static bool Load(const ros::NodeHandle& nh, CollisionModelConfig& cfg);
};

} // namespace collision
} // namespace sbpl

#endif
