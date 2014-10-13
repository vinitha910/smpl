#include <sbpl_collision_checking/sbpl_collision_model.h>
#include "collision_model_impl.h"

namespace sbpl_arm_planner
{

SBPLCollisionModel::SBPLCollisionModel() :
    impl_(new sbpl::manipulation::CollisionModelImpl)
{
}

SBPLCollisionModel::~SBPLCollisionModel()
{
}

bool SBPLCollisionModel::init(const std::string& urdf_string)
{
    return impl_->init(urdf_string);
}

void SBPLCollisionModel::getGroupNames(std::vector<std::string> &names)
{
    return impl_->getGroupNames(names);
}

bool SBPLCollisionModel::setDefaultGroup(const std::string &group_name)
{
    return impl_->setDefaultGroup(group_name);
}

void SBPLCollisionModel::printGroups()
{
    return impl_->printGroups();
}

bool SBPLCollisionModel::getFrameInfo(const std::string &name, const std::string &group_name, int &chain, int &segment)
{
    return impl_->getFrameInfo(name, group_name, chain, segment);
}

bool SBPLCollisionModel::initAllGroups()
{
    return impl_->initAllGroups();
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

void SBPLCollisionModel::setJointPosition(const std::string &name, double position)
{
    return impl_->setJointPosition(name, position);
}

void SBPLCollisionModel::printDebugInfo(const std::string &group_name)
{
    return impl_->printDebugInfo(group_name);
}

void SBPLCollisionModel::getDefaultGroupSpheres(std::vector<Sphere*> &spheres)
{
    return impl_->getDefaultGroupSpheres(spheres);
}

bool SBPLCollisionModel::getJointLimits(
    const std::string &group_name,
    const std::string &joint_name,
    double &min_limit,
    double &max_limit,
    bool &continuous)
{
    return impl_->getJointLimits(group_name, joint_name, min_limit, max_limit, continuous);
}

std::string SBPLCollisionModel::getReferenceFrame(const std::string &group_name)
{
    return impl_->getReferenceFrame(group_name);
}

Group *SBPLCollisionModel::getGroup(const std::string &name)
{
    return impl_->getGroup(name);
}

void SBPLCollisionModel::getVoxelGroups(std::vector<Group*> &vg)
{
    return impl_->getVoxelGroups(vg);
}

bool SBPLCollisionModel::doesLinkExist(const std::string &name, const std::string &group_name)
{
    return impl_->doesLinkExist(name, group_name);
}

bool SBPLCollisionModel::setModelToWorldTransform(
    const moveit_msgs::RobotState &state,
    const std::string &world_frame)
{
    return impl_->setWorldToModelTransform(state, world_frame);
}

} // namespace sbpl_arm_planner
