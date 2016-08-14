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

#ifndef sbpl_manip_detail_robot_model_h
#define sbpl_manip_detail_robot_model_h

#include <sbpl_arm_planner/robot_model.h>

#include <ros/console.h>

namespace sbpl {
namespace manip {

template <typename T>
bool RobotModel::registerExtension(T* e)
{
    return m_database->registerExtension<T>(e);
}

template <typename T>
bool RobotModel::unregisterExtension(T* e)
{
    return m_database->unregisterExtension<T>(e);
}

template <typename Extension>
Extension* RobotModel::getExtension()
{
    return m_database->getExtension<Extension>();
}

template <typename T>
bool ExtensionDatabase::registerExtension(T* e)
{
    size_t id = typeid(T).hash_code();
    auto it = m_extensions.find(id);
    if (it != m_extensions.end()) {
        ROS_DEBUG_NAMED("robot", "extension for %s already exists", typeid(T).name());
        return false;
    }

    RobotModel* iface = dynamic_cast<RobotModel*>(e);
    if (!iface) {
        ROS_ERROR_NAMED("robot", "extension %s is not a robot model", typeid(T).name());
        return false;
    }

    ROS_DEBUG_NAMED("robot", "register extension %s", typeid(T).name());
    m_extensions[id] = iface;
    return true;
}

template <typename T>
bool ExtensionDatabase::unregisterExtension(T* e)
{
    ROS_DEBUG_NAMED("robot", "unregister extension %s", typeid(T).name());
    size_t id = typeid(T).hash_code();
    return m_extensions.erase(id) == 1;
}

template <typename Extension>
Extension* ExtensionDatabase::getExtension()
{
    size_t id = typeid(Extension).hash_code();
    auto it = m_extensions.find(id);
    if (it == m_extensions.end()) {
        ROS_DEBUG_NAMED("robot", "extension %s was not found", typeid(Extension).name());
        return nullptr;
    }
    Extension* e = dynamic_cast<Extension*>(it->second);
    return e;
}

} // namespace manip
} // namespace sbpl

#endif

