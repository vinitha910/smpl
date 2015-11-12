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

#include "collision_model_impl.h"

#include <queue>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/tree.hpp>

#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

CollisionModelImpl::CollisionModelImpl() :
    nh_(),
    ph_("~"),
    rm_loader_(),
    robot_model_(),
    group_config_map_(),
    urdf_(),
    dgroup_(nullptr)
{
}

CollisionModelImpl::~CollisionModelImpl()
{
    for (auto iter = group_config_map_.begin();
        iter != group_config_map_.end();
        iter++)
    {
        if (iter->second != NULL) {
            delete iter->second;
        }
    }
}

bool CollisionModelImpl::init(
    const std::string& urdf_string,
    const CollisionModelConfig& config)
{
    return initURDF(urdf_string) &&
            initMoveItRobotModel(urdf_string) &&
            initAllGroups(config) &&
            initKdlRobotModel();
}

void CollisionModelImpl::getGroupNames(std::vector<std::string>& names) const
{
    for (auto iter = group_config_map_.begin();
        iter != group_config_map_.end();
        ++iter)
    {
        names.push_back(iter->first);
    }
}

bool CollisionModelImpl::setDefaultGroup(const std::string& group_name)
{
    if (group_config_map_.find(group_name) == group_config_map_.end()) {
        ROS_ERROR("Failed to find group '%s' in group_config_map_", group_name.c_str());
        ROS_ERROR("Expecting one of the following group names:");
        for (auto it = group_config_map_.cbegin();
            it != group_config_map_.cend();
            ++it)
        {
            ROS_ERROR("%s", it->first.c_str());
        }
        return false;
    }
    
    dgroup_ = group_config_map_[group_name];
    return true;
}

void CollisionModelImpl::printGroups() const
{
    if (group_config_map_.begin() == group_config_map_.end()) {
        ROS_ERROR("No groups found.");
        return;
    }

    for (auto iter = group_config_map_.begin();
        iter != group_config_map_.end();
        ++iter)
    {
        if (!iter->second->init_) {
            ROS_ERROR("Failed to print %s group information because has not yet been initialized.", iter->second->getName().c_str());
            continue;
        }
        iter->second->print();
        ROS_INFO("----------------------------------");
    }
}

bool CollisionModelImpl::getFrameInfo(
    const std::string& name,
    const std::string& group_name,
    int& chain,
    int& segment) const
{
    auto git = group_config_map_.find(group_name);
    if (git == group_config_map_.end()) {
        ROS_ERROR("No group named '%s'", name.c_str());
        return false;
    }
    else {
        return git->second->getFrameInfo(name, chain, segment);
    }
}

bool CollisionModelImpl::initAllGroups(const CollisionModelConfig& config)
{
    for (const CollisionGroupConfig& group_config : config.collision_groups) {
        const std::string& gname = group_config.name;

        if (group_config_map_.find(gname) != group_config_map_.end()) {
            ROS_WARN("Already have group name '%s'", gname.c_str());
            continue;
        }


        Group* gc = new Group;
        if (!gc->init(urdf_, group_config, config.collision_spheres)) {
            ROS_ERROR("Failed to get all params for %s", gname.c_str());
            delete gc;
            return false;
        }

        group_config_map_[gname] = gc;
    }

    ROS_INFO("Successfully initialized collision groups");
    return true;
}

bool CollisionModelImpl::computeDefaultGroupFK(
    const std::vector<double>& angles,
    std::vector<std::vector<KDL::Frame>>& frames)
{
    return computeGroupFK(angles, dgroup_, frames);
}

bool CollisionModelImpl::computeGroupFK(
    const std::vector<double>& angles,
    Group* group,
    std::vector<std::vector<KDL::Frame>>& frames)
{
    return group->computeFK(angles, frames);
}

void CollisionModelImpl::setOrderOfJointPositions(
    const std::vector<std::string>& joint_names,
    const std::string& group_name)
{
    group_config_map_[group_name]->setOrderOfJointPositions(joint_names);
}

void CollisionModelImpl::setJointPosition(
    const std::string& name,
    double position)
{
    auto kjmit = m_joint_map.find(name);
    if (kjmit == m_joint_map.end()) {
        ROS_ERROR("Collision Robot doesn't know about joint '%s'", name.c_str());
        return;
    }

    const KDLJointMapping& kjm = kjmit->second;
    for (size_t i = 0; kjm.chain_indices.size(): ++i) {
        int chidx = kjmit.chain_indices[i];
        int jidx = kjmit.joint_indices[i];
        m_joint_arrays[chidx][jidx] = position;
    }

    for (auto iter = group_config_map_.begin();
        iter != group_config_map_.end();
        iter++)
    {
        iter->second->setJointPosition(name, position);
    }
}

void CollisionModelImpl::printDebugInfo(const std::string& group_name) const
{
    auto git = group_config_map_.find(group_name);
    if (git == group_config_map_.end()) {
        ROS_INFO("No group '%s' found", group_name.c_str());
    }
    else {
        Group* group = git->second;
        group->printDebugInfo();
    }
}

const std::vector<const Sphere*>&
CollisionModelImpl::getDefaultGroupSpheres() const
{
    return dgroup_->getSpheres();
}

bool CollisionModelImpl::getJointLimits(
    const std::string& group_name,
    const std::string& joint_name,
    double& min_limit,
    double& max_limit,
    bool& continuous) const
{
    if (group_config_map_.find(group_name) == group_config_map_.end()) {
        ROS_ERROR("Collision Model does not contain group '%s'", group_name.c_str());
        return false;
    }

    auto git = group_config_map_.at(group_name);

    if (!git->init_) {
        ROS_ERROR("Collision Model Group '%s' is not initialized", group_name.c_str());
        return false;
    }

    const std::string& root_link_name = git->getReferenceFrame();
    const std::string& tip_link_name = git->tip_name_;
    if (!leatherman::getJointLimits(
            urdf_.get(),
            root_link_name,
            tip_link_name,
            joint_name,
            min_limit,
            max_limit,
            continuous))
    {
        ROS_ERROR("Failed to find joint limits for joint '%s' between links '%s' and '%s'", joint_name.c_str(), root_link_name.c_str(), tip_link_name.c_str());
        return false;
    }

    return true;
}

std::string CollisionModelImpl::getReferenceFrame(
    const std::string& group_name) const
{
    return m_model_frame;
}

Group* CollisionModelImpl::getGroup(const std::string& name)
{
    Group* r = NULL;
    if (group_config_map_.find(name) == group_config_map_.end()) {
        return r;
    }
    return group_config_map_[name];
}

void CollisionModelImpl::getVoxelGroups(std::vector<Group*>& vg)
{
    vg.clear();
    for (auto iter = group_config_map_.begin();
        iter != group_config_map_.end();
        ++iter)
    {
        if (iter->second->type_ == Group::VOXELS) {
            vg.push_back(iter->second);
        }
    }
}

bool CollisionModelImpl::doesLinkExist(
    const std::string& name,
    const std::string& group_name) const
{
    int chain, segment;
    return getFrameInfo(name, group_name, chain, segment);
}

bool CollisionModelImpl::setWorldToModelTransform(
    const moveit_msgs::RobotState& state,
    const std::string& world_frame)
{
    Eigen::Affine3d T_world_robot(Eigen::Affine3d::Identity());
    KDL::Frame f;

    // set all single-variable joints
    std::map<std::string, double> joint_value_map;
    for (size_t i = 0; i < state.joint_state.name.size(); ++i) {
        joint_value_map[state.joint_state.name[i]] = state.joint_state.position[i];
    }
    robot_state_->setVariablePositions(joint_value_map);
    robot_state_->updateLinkTransforms();

    // check for robot_pose joint variables
    const std::string& robot_pose_joint_name = "robot_pose";
    if (world_frame != robot_model_->getModelFrame()) {
        bool found_world_pose = false;
        for (size_t i = 0; i < state.multi_dof_joint_state.joint_names.size(); ++i) {
            if (state.multi_dof_joint_state.joint_names[i] == robot_pose_joint_name) {
                found_world_pose = true;
                tf::transformMsgToEigen(state.multi_dof_joint_state.transforms[i], T_world_robot);
            }
        }
    
        if (!found_world_pose) {
            ROS_ERROR("Failed to find 6-DoF joint state 'world_pose' from MultiDOFJointState");
            return false;
        }
    }

    // set the transform from the world frame to each group reference frame
    for (auto iter = group_config_map_.begin(); iter != group_config_map_.end(); ++iter) {
        const std::string& group_frame = iter->second->getReferenceFrame();
        if (!robot_state_->knowsFrameTransform(iter->second->getReferenceFrame())) {
            ROS_ERROR("Robot Model does not contain transform from robot frame '%s' to group frame '%s'", robot_model_->getModelFrame().c_str(), group_frame.c_str());
            return false;
        }
        else {
            Eigen::Affine3d T_world_group = T_world_robot * robot_state_->getFrameTransform(group_frame);
            tf::transformEigenToKDL(T_world_group, f);
            iter->second->setModelToGroupTransform(f);
            leatherman::printKDLFrame(f, "group-world");
        }
    }

    return true;
}

bool CollisionModelImpl::initURDF(const std::string& urdf_string)
{
    urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (!urdf_->initString(urdf_string)) {
        ROS_WARN("Failed to parse the URDF");
        return false;
    }

    return true;
}

bool CollisionModelImpl::initKdlRobotModel()
{
    assert((bool)urdf_);

    if (!kdl_parser::treeFromUrdfModel(*urdf_, m_tree)) {
        ROS_ERROR("Failed to parse URDF into KDL tree");
        return false;
    }

    m_model_frame = m_tree.getRootSegment()->first.getName();
    ROS_INFO("Collision Robot Frame: %s", m_model_frame.c_str());

    // figure out which chains we need to update the model->group transforms

    m_chains.reserve(group_config_map_.size());
    m_joint_arrays.reserve(group_config_map_.size());

    for (const auto& ent : group_config_map_) {
        const std::string& group_name = ent.first;
        Group* group = ent.second;

        // create kinematic chain
        KDL::Chain chain;
        if (!m_tree.getChain(
                getReferenceFrame(), group->getReferenceFrame(), chain))
        {
            ROS_ERROR("Failed to get kinematic chain from reference from '%s' to group reference frame '%s'", getReferenceFrame().c_str(), gc->getReferenceFrame().c_str());
            return false;
        }
        m_chains.push_back(chain);

        // fill joint array with 0's
        KDL::JntArray jarray;
        jarray.resize(m_chains.back().getNrOfJoints());
        m_joint_arrays.push_back(jarray);
        KDL::SetToZero(m_joint_arrays.back());
    }

    // for every joint variable in the robot model, we need to figure out
    // which kinematic chains it influences and at which segment indices
    auto root_link = urdf_->getRoot();
    std::queue<boost::shared_ptr<const urdf::Link>> links;
    links.push_back(root_link);
    while (!links.empty()) {
        boost::shared_ptr<const urdf::Link> link = links.front();
        links.pop();

        // for each joint
        for (const auto& joint : link.child_joints) {
            const std::string& joint_name = joint->name;

            // create an entry from joint name -> kdl mappings
            auto vit = m_joint_map.insert(std::make_pair(
                    joint_name, KDLJointMapping()));

            // for each chain
            for (size_t chidx = 0; chidx < m_chains.size(); ++chidx) {
                // find the joint index of this joint in the chain, if any
                const KDL::Chain& chain = m_chains[chidx];
                int jidx = 0;
                for (int sidx = 0; sidx < chain.getNrOfSegments(); ++sidx) {
                    const KDL::Segment& seg = chain.getSegment(sidx);
                    if (seg.getJoint().getName() == joint_name) {
                        vit->second.chain_indices.push_back(chidx);
                        vit->second.joint_indices.push_back(jidx);
                        break;
                    }

                    if (seg.getJoint().getName() != "None") {
                        ++jidx;
                    }
                }
            }
        }

        // add all child links
        for (const auto& child_link : link.child_links) {
            links.push(child_link);
        }
    }


    return true;
}

bool CollisionModelImpl::initMoveItRobotModel(const std::string& urdf_string)
{
    std::string srdf_string;
    if (!nh_.getParam("robot_description_semantic", srdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description_semantic' from the param server");
        return false;
    }

    robot_model_loader::RobotModelLoader::Options ops;
    ops.robot_description_ = "";
    ops.urdf_string_ = urdf_string;
    ops.srdf_string_ = srdf_string;
    ops.urdf_doc_ = nullptr;
    ops.srdf_doc_ = nullptr;
    ops.load_kinematics_solvers_ = false;

    rm_loader_.reset(new robot_model_loader::RobotModelLoader(ops));
    if (!rm_loader_) {
        ROS_ERROR("Failed to instantiate Robot Model Loader");
        return false;
    }

    robot_model_ = rm_loader_->getModel();
    if (!robot_model_) {
        ROS_ERROR("Failed to retrieve valid Robot Model");
        return false;
    }

    robot_state_.reset(new robot_state::RobotState(robot_model_));
    if (!robot_state_) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    return true;
}

void CollisionModelImpl::decomposeRobotModel()
{
    // std::map<std::string, TreeElement>::const_iterator
    typedef KDL::SegmentMap::const_iterator segment_iterator;

    std::queue<segment_iterator> q;
    q.push(m_tree.getRootSegment());

    ROS_INFO("Root segment: %s", q.front()->first.c_str());

    // gather leaf nodes
    std::vector<segment_iterator> leaves;
    while (!q.empty()) {
        segment_iterator sit = q.front();
        q.pop();

        const KDL::TreeElement& e = sit->second;

        ROS_INFO("Segment %s: Joint %s, Type: %s",
                e.segment.getName().c_str(),
                e.segment.getJoint().getName().c_str(),
                e.segment.getJoint().getTypeName().c_str());

        if (e.children.empty()) {
            leaves.push_back(sit);
        }
        else {
            for (auto child : e.children) {
                q.push(child);
            }
        }
    }

    ROS_INFO("Found %zu leaves", leaves.size());
    for (auto leaf : leaves) {
        const std::string& name = leaf->first;
        ROS_INFO("  %s", name.c_str());
    }

    std::set<std::string> chain_roots;

    std::vector<KDL::Chain> chains;

    while (!leaves.empty()) {
        auto liit = leaves.end(); // last leaf so we can pop_back later
        --liit;
        segment_iterator tit = *liit;

        leaves.pop_back();

        const KDL::TreeElement& e = tit->second;

        auto rit = tit;
        while (rit != m_tree.getRootSegment() &&
            rit->second.parent->second.children.size() == 1)
        {
            rit = rit->second.parent;
        }

        // either we'll end at the root or we'll end at a node whose parent has
        // children

        if (rit == m_tree.getRootSegment()) {
            if (tit == m_tree.getRootSegment()) {

            }
            else {
                KDL::Chain chain;
                if (!m_tree.getChain(rit->first, tit->first, chain)) {
                    ROS_ERROR("Something went wrong getting chain");
                    return false;
                }

                chains.push_back(chain);
                ROS_INFO("Created chain from %s to %s with %u joints",
                        rit->first.c_str(),
                        tit->first.c_str(),
                        chains.back().getNrOfJoints());
            }
        }
        else {
            auto pit = rit;
            pit = pit->second.parent; // may be root

            KDL::Chain chain;
            if (!m_tree.getChain(pit->first, tit->first, chain)) {
                ROS_ERROR("Something went wrong getting chain");
                return false;
            }

            chains.push_back(chain);
            ROS_INFO("Created chain from %s to %s with %u joints",
                    pit->first.c_str(),
                    tit->first.c_str(),
                    chains.back().getNrOfJoints());

            if (chain_roots.find(pit->first) == chain_roots.end()) {
                leaves.push_back(pit);
                chain_roots.insert(pit->first);
            }
        }
    }
}

} // namespace collision
} // namespace sbpl
