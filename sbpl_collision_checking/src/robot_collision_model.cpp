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
#include <sbpl_collision_checking/robot_collision_model.h>
#include "collision_model_impl.h"

namespace sbpl {
namespace collision {

RobotCollisionModel::RobotCollisionModel() :
    m_impl(new CollisionModelImpl)
{
}

RobotCollisionModel::~RobotCollisionModel()
{
}

bool RobotCollisionModel::init(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    return m_impl->init(urdf, config);
}

const std::string& RobotCollisionModel::name() const
{
    return m_impl->name();
}

const std::string& RobotCollisionModel::modelFrame() const
{
    return m_impl->modelFrame();
}

size_t RobotCollisionModel::jointVarCount() const
{
    return m_impl->jointVarCount();
}

const std::vector<std::string>& RobotCollisionModel::jointVarNames() const
{
    return m_impl->jointVarNames();
}

bool RobotCollisionModel::hasJointVar(const std::string& joint_name) const
{
    return m_impl->hasJointVar(joint_name);
}

int RobotCollisionModel::jointVarIndex(const std::string& joint_name) const
{
    return m_impl->jointVarIndex(joint_name);
}

const std::string& RobotCollisionModel::jointVarName(int jidx) const
{
    return m_impl->jointVarName(jidx);
}

bool RobotCollisionModel::jointVarIsContinuous(
    const std::string& joint_name) const
{
    return m_impl->jointVarIsContinuous(joint_name);
}

bool RobotCollisionModel::jointVarHasPositionBounds(
    const std::string& joint_name) const
{
    return m_impl->jointVarHasPositionBounds(joint_name);
}

double RobotCollisionModel::jointVarMaxPosition(
    const std::string& joint_name) const
{
    return m_impl->jointVarMaxPosition(joint_name);
}

double RobotCollisionModel::jointVarMinPosition(
    const std::string& joint_name) const
{
    return m_impl->jointVarMinPosition(joint_name);
}

bool RobotCollisionModel::jointVarIsContinuous(int jidx) const
{
    return m_impl->jointVarIsContinuous(jidx);
}

bool RobotCollisionModel::jointVarHasPositionBounds(int jidx) const
{
    return m_impl->jointVarHasPositionBounds(jidx);
}

double RobotCollisionModel::jointVarMinPosition(int jidx) const
{
    return m_impl->jointVarMinPosition(jidx);
}

double RobotCollisionModel::jointVarMaxPosition(int jidx) const
{
    return m_impl->jointVarMaxPosition(jidx);
}

size_t RobotCollisionModel::linkCount() const
{
    return m_impl->linkCount();
}

const std::vector<std::string>& RobotCollisionModel::linkNames() const
{
    return m_impl->linkNames();
}

bool RobotCollisionModel::hasLink(const std::string& link_name) const
{
    return m_impl->hasLink(link_name);
}

int RobotCollisionModel::linkIndex(const std::string& link_name) const
{
    return m_impl->linkIndex(link_name);
}

const std::string& RobotCollisionModel::linkName(int lidx) const
{
    return m_impl->linkName(lidx);
}

size_t RobotCollisionModel::sphereModelCount() const
{
    return m_impl->sphereModelCount();
}

const CollisionSphereModel& RobotCollisionModel::sphereModel(int smidx) const
{
    return m_impl->sphereModel(smidx);
}

bool RobotCollisionModel::hasSpheresModel(const std::string& link_name) const
{
    return m_impl->hasSpheresModel(link_name);
}

bool RobotCollisionModel::hasSpheresModel(int lidx) const
{
    return m_impl->hasSpheresModel(lidx);
}

bool RobotCollisionModel::hasVoxelsModel(const std::string& link_name) const
{
    return m_impl->hasVoxelsModel(link_name);
}

bool RobotCollisionModel::hasVoxelsModel(int lidx) const
{
    return m_impl->hasVoxelsModel(lidx);
}

size_t RobotCollisionModel::voxelsModelCount() const
{
    return m_impl->voxelsModelCount();
}

const CollisionVoxelsModel& RobotCollisionModel::voxelsModel(int vmidx) const
{
    return m_impl->voxelsModel(vmidx);
}

size_t RobotCollisionModel::groupCount() const
{
    return m_impl->groupCount();
}

const std::vector<CollisionGroupModel>& RobotCollisionModel::groups() const
{
    return m_impl->groups();
}

bool RobotCollisionModel::hasGroup(const std::string& group_name) const
{
    return m_impl->hasGroup(group_name);
}

int RobotCollisionModel::groupIndex(const std::string& group_name) const
{
    return m_impl->groupIndex(group_name);
}

const std::string& RobotCollisionModel::groupName(int gidx) const
{
    return m_impl->groupName(gidx);
}

const std::vector<int>& RobotCollisionModel::groupLinkIndices(
    const std::string& group_name) const
{
    return m_impl->groupLinkIndices(group_name);
}

const std::vector<int>& RobotCollisionModel::groupLinkIndices(int gidx) const
{
    return m_impl->groupLinkIndices(gidx);
}

const std::vector<int>& RobotCollisionModel::groupSphereStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupSphereStateIndices(group_name);
}

const std::vector<int>& RobotCollisionModel::groupSphereStateIndices(
    int gidx) const
{
    return m_impl->groupSphereStateIndices(gidx);
}

const std::vector<int>& RobotCollisionModel::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupOutsideVoxelsStateIndices(group_name);
}

const std::vector<int>& RobotCollisionModel::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    return m_impl->groupOutsideVoxelsStateIndices(gidx);
}

const Eigen::Affine3d& RobotCollisionModel::worldToModelTransform() const
{
    return m_impl->worldToModelTransform();
}

bool RobotCollisionModel::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    return m_impl->setWorldToModelTransform(transform);
}

const std::vector<double>& RobotCollisionModel::jointPositions() const
{
    return m_impl->jointPositions();
}

const Affine3dVector& RobotCollisionModel::linkTransforms() const
{
    return m_impl->linkTransforms();
}

double RobotCollisionModel::jointPosition(const std::string& joint_name) const
{
    return m_impl->jointPosition(joint_name);
}

double RobotCollisionModel::jointPosition(int jidx) const
{
    return m_impl->jointPosition(jidx);
}

bool RobotCollisionModel::setJointPosition(
    const std::string& name,
    double position)
{
    return m_impl->setJointPosition(name, position);
}

bool RobotCollisionModel::setJointPosition(int jidx, double position)
{
    return m_impl->setJointPosition(jidx, position);
}

const Eigen::Affine3d& RobotCollisionModel::linkTransform(
    const std::string& link_name) const
{
    return m_impl->linkTransform(link_name);
}

const Eigen::Affine3d& RobotCollisionModel::linkTransform(int lidx) const
{
    return m_impl->linkTransform(lidx);
}

bool RobotCollisionModel::linkTransformDirty(const std::string& link_name) const
{
    return m_impl->linkTransformDirty(link_name);
}

bool RobotCollisionModel::linkTransformDirty(int lidx) const
{
    return m_impl->linkTransformDirty(lidx);
}

bool RobotCollisionModel::updateLinkTransforms()
{
    return m_impl->updateLinkTransforms();
}

bool RobotCollisionModel::updateLinkTransform(int lidx)
{
    return m_impl->updateLinkTransform(lidx);
}

bool RobotCollisionModel::updateLinkTransform(const std::string& link_name)
{
    return m_impl->updateLinkTransform(link_name);
}

const CollisionVoxelsState& RobotCollisionModel::voxelsState(int vsidx) const
{
    return m_impl->voxelsState(vsidx);
}

bool RobotCollisionModel::voxelsStateDirty(int vsidx) const
{
    return m_impl->voxelsStateDirty(vsidx);
}

bool RobotCollisionModel::updateVoxelsStates()
{
    return m_impl->updateVoxelsStates();
}

bool RobotCollisionModel::updateVoxelsState(int vsidx)
{
    return m_impl->updateVoxelsState(vsidx);
}

const CollisionSphereState& RobotCollisionModel::sphereState(int ssidx) const
{
    return m_impl->sphereState(ssidx);
}

bool RobotCollisionModel::sphereStateDirty(int ssidx) const
{
    return m_impl->sphereStateDirty(ssidx);
}

bool RobotCollisionModel::updateSphereStates()
{
    return m_impl->updateSphereStates();
}

bool RobotCollisionModel::updateSphereState(int ssidx)
{
    return m_impl->updateSphereState(ssidx);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getVisualization() const
{
    return m_impl->getVisualization();
}

visualization_msgs::MarkerArray
RobotCollisionModel::getVisualization(const std::string& group_name) const
{
    return m_impl->getVisualization(group_name);
}

visualization_msgs::MarkerArray
RobotCollisionModel::getVisualization(int gidx) const
{
    return m_impl->getVisualization(gidx);
}


} // namespace collision
} // namespace sbpl
