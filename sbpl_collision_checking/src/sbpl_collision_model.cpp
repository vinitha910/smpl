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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

// project includes
#include <sbpl_collision_checking/sbpl_collision_model.h>
#include "collision_model_impl.h"

namespace sbpl {
namespace collision {

SBPLCollisionModel::SBPLCollisionModel() :
    impl_(new CollisionModelImpl)
{
}

SBPLCollisionModel::~SBPLCollisionModel()
{
}

bool SBPLCollisionModel::init(
    const std::string& urdf_string,
    const CollisionModelConfig& config)
{
    return impl_->init(urdf_string, config);
}

void SBPLCollisionModel::getGroupNames(std::vector<std::string> &names) const
{
    return impl_->getGroupNames(names);
}

bool SBPLCollisionModel::setDefaultGroup(const std::string &group_name)
{
    return impl_->setDefaultGroup(group_name);
}

void SBPLCollisionModel::printGroups() const
{
    return impl_->printGroups();
}

bool SBPLCollisionModel::getFrameInfo(
    const std::string &name,
    const std::string &group_name,
    int &chain,
    int &segment) const
{
    return impl_->getFrameInfo(name, group_name, chain, segment);
}

bool SBPLCollisionModel::computeDefaultGroupFK(
    const std::vector<double> &angles,
    std::vector<std::vector<KDL::Frame>> &frames)
{
    return impl_->computeDefaultGroupFK(angles, frames);
}

bool SBPLCollisionModel::computeGroupFK(
    const std::vector<double> &angles,
    Group *group,
    std::vector<std::vector<KDL::Frame>> &frames)
{
    return impl_->computeGroupFK(angles, group, frames);
}

void SBPLCollisionModel::setOrderOfJointPositions(
    const std::vector<std::string> &joint_names,
    const std::string &group_name)
{
    return impl_->setOrderOfJointPositions(joint_names, group_name);
}

void SBPLCollisionModel::setJointPosition(
    const std::string &name,
    double position)
{
    return impl_->setJointPosition(name, position);
}

void SBPLCollisionModel::printDebugInfo(const std::string &group_name) const
{
    return impl_->printDebugInfo(group_name);
}

const std::vector<const Sphere*>&
SBPLCollisionModel::getDefaultGroupSpheres() const
{
    return impl_->getDefaultGroupSpheres();
}

bool SBPLCollisionModel::getJointLimits(
    const std::string &group_name,
    const std::string &joint_name,
    double &min_limit,
    double &max_limit,
    bool &continuous) const
{
    return impl_->getJointLimits(
            group_name, joint_name, min_limit, max_limit, continuous);
}

std::string SBPLCollisionModel::getReferenceFrame(
    const std::string &group_name) const
{
    return impl_->getReferenceFrame(group_name);
}

Group *SBPLCollisionModel::getGroup(const std::string &name)
{
    return impl_->getGroup(name);
}

void SBPLCollisionModel::getVoxelGroups(std::vector<Group*>& vg)
{
    return impl_->getVoxelGroups(vg);
}

bool SBPLCollisionModel::doesLinkExist(
    const std::string& name,
    const std::string& group_name) const
{
    return impl_->doesLinkExist(name, group_name);
}

void SBPLCollisionModel::setWorldToModelTransform(const KDL::Frame& f)
{
    impl_->setWorldToModelTransform(f);
}

} // namespace collision
} // namespace sbpl
