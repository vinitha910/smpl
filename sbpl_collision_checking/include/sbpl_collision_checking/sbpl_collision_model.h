////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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

#ifndef sbpl_collision_SBPLCollisionModel_h
#define sbpl_collision_SBPLCollisionModel_h

#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <urdf/model.h>

#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/group.h>

namespace sbpl {
namespace collision {

class CollisionModelImpl;

/// \brief Represents the collision model of the robot used for planning.
class SBPLCollisionModel
{
public:

    SBPLCollisionModel();
    ~SBPLCollisionModel();

    /// \name Initialization
    ///@{

    bool init(
        const std::string& urdf_string,
        const CollisionModelConfig& config);

    bool setDefaultGroup(const std::string& group_name);

    void setOrderOfJointPositions(
        const std::vector<std::string>& joint_names,
        const std::string& group_name);

    ///@}

    std::string getReferenceFrame(const std::string& group_name) const;

    void getGroupNames(std::vector<std::string>& names) const;

    bool getJointLimits(
        const std::string& group_name,
        const std::string& joint_name,
        double& min_limit,
        double& max_limit,
        bool& continuous) const;

    bool getFrameInfo(
        const std::string& name,
        const std::string& group_name,
        int& chain,
        int& segment) const;

    bool doesLinkExist(const std::string& name, const std::string& group_name) const;

    const std::vector<const Sphere*>& getDefaultGroupSpheres() const;

    /// \name Modify State of Collision Robot
    /// @{

    void setWorldToModelTransform(const KDL::Frame& f);

    void setJointPosition(const std::string& name, double position);

    Group* getGroup(const std::string& name);

    void getVoxelGroups(std::vector<Group*>& vg);

    bool computeDefaultGroupFK(
        const std::vector<double>& angles,
        std::vector<std::vector<KDL::Frame>>& frames);

    bool computeGroupFK(
        const std::vector<double>& angles,
        Group* group,
        std::vector<std::vector<KDL::Frame>>& frames);

    /// @}

    void printGroups() const;

    void printDebugInfo(const std::string& group_name) const;

private:

    std::unique_ptr<CollisionModelImpl> impl_;
};

} // namespace collision
} // namespace sbpl

#endif
