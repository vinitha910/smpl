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
#include <leatherman/print.h>

#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

CollisionModelImpl::CollisionModelImpl() :
    nh_(),
    ph_("~"),
    urdf_(),
    m_model_frame(),
    m_tree(),
    m_chains(),
    m_solvers(),
    m_joint_arrays(),
    m_joint_map(),
    group_config_map_(),
    dgroup_(nullptr),
    m_T_world_model(KDL::Frame::Identity())
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
        const std::string& group_name = group_config.name;

        if (group_config_map_.find(group_name) != group_config_map_.end()) {
            ROS_WARN("Already have group name '%s'", group_name.c_str());
            continue;
        }


        Group* gc = new Group;
        if (!gc->init(urdf_, group_config, config.collision_spheres)) {
            ROS_ERROR("Failed to get all params for %s", group_name.c_str());
            delete gc;
            return false;
        }

        group_config_map_[group_name] = gc;
    }

    ROS_DEBUG("Successfully initialized collision groups");
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
    if (group->computeFK(angles, frames)) {
        for (size_t i = 0; i < frames.size(); ++i) {
            for (size_t j = 0; j < frames[i].size(); ++j) {
                frames[i][j] = m_T_world_model * frames[i][j];
            }
        }
        return true;
    }
    else {
        return false;
    }
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
    std::vector<bool> dirty_chains(m_chains.size(), false);

    auto kjmit = m_joint_map.find(name);
    if (kjmit == m_joint_map.end()) {
        ROS_ERROR("Collision Robot doesn't know about joint '%s'", name.c_str());
        return;
    }

    // set the joint position in any corresponding chains to groups
    const KDLJointMapping& kjm = kjmit->second;
    for (size_t i = 0; i < kjm.chain_indices.size(); ++i) {
        int chidx = kjm.chain_indices[i];
        int jidx = kjm.joint_indices[i];
        m_joint_arrays[chidx](jidx) = position;
        dirty_chains[chidx] = true;
    }

    // update any model->group transforms
    for (size_t chidx = 0; chidx < m_chains.size(); ++chidx) {
        const bool dirty = dirty_chains[chidx];
        if (dirty) {
            // update this chain and set the model frame
            KDL::Frame f(KDL::Frame::Identity());
            if (m_solvers[chidx]->JntToCart(m_joint_arrays[chidx], f) < 0) {
                ROS_WARN("Solver failed to compute forward kinematics for chain %zu", chidx);
            }

            ROS_INFO("Updating model-to-group transform for group %s", m_chain_index_to_group[chidx]->getName().c_str());
            m_chain_index_to_group[chidx]->setModelToGroupTransform(f);
        }
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

void CollisionModelImpl::setReferenceFrame(const std::string& frame)
{
    m_model_frame = frame;
}

bool CollisionModelImpl::initURDF(const std::string& urdf_string)
{
    urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (!urdf_->initString(urdf_string)) {
        ROS_WARN("Failed to parse the URDF");
        return false;
    }

    auto root_link = urdf_->getRoot();
    std::queue<boost::shared_ptr<const urdf::Link>> links;
    links.push(root_link);
    while (!links.empty()) {
        boost::shared_ptr<const urdf::Link> link = links.front();
        links.pop();

        // for each joint
        for (const auto& joint : link->child_joints) {
            const std::string& joint_name = joint->name;
            m_joint_names.push_back(joint_name);
            m_joint_positions.push_back(0.0);
            m_joint_to_index[m_joint_names.back()] = m_joint_names.size() - 1;
        }

        // add all child links
        for (const auto& child_link : link->child_links) {
            links.push(child_link);
        }
    }

    setReferenceFrame(urdf_->getRoot()->name);
    ROS_DEBUG("Collision Robot Frame: %s", getReferenceFrame().c_str());

    return true;
}

bool CollisionModelImpl::initKdlRobotModel()
{
    assert((bool)urdf_);

    if (!kdl_parser::treeFromUrdfModel(*urdf_, m_tree)) {
        ROS_ERROR("Failed to parse URDF into KDL tree");
        return false;
    }

    std::string kdl_root = m_tree.getRootSegment()->second.segment.getName();
    if (kdl_root != getReferenceFrame()) {
        ROS_ERROR("Root frame of KDL tree is not the reference frame of the robot");
        return false;
    }

    // initialize kinematic chain and joint array for each group to get the
    // transform from the model root to the group root
    m_chains.reserve(group_config_map_.size());
    m_chain_index_to_group.reserve(group_config_map_.size());
    m_solvers.reserve(group_config_map_.size());
    m_joint_arrays.reserve(group_config_map_.size());

    for (const auto& ent : group_config_map_) {
        const std::string& group_name = ent.first;
        Group* group = ent.second;

        // create kinematic chain
        KDL::Chain chain;
        if (!m_tree.getChain(
                getReferenceFrame(), group->getReferenceFrame(), chain))
        {
            ROS_ERROR("Failed to get kinematic chain from reference from '%s' to group reference frame '%s'",
                    getReferenceFrame().c_str(), group->getReferenceFrame().c_str());
            return false;
        }
        m_chains.push_back(chain);
        m_chain_index_to_group.push_back(group);
        m_solvers.push_back(
                std::unique_ptr<KDL::ChainFkSolverPos_recursive>(
                        new KDL::ChainFkSolverPos_recursive(m_chains.back())));

        // fill joint array with 0's
        KDL::JntArray jarray;
        jarray.resize(m_chains.back().getNrOfJoints());
        m_joint_arrays.push_back(jarray);
        KDL::SetToZero(m_joint_arrays.back());
    }

    ROS_DEBUG("Chains from robot root to group root");
    for (size_t chidx = 0; chidx < m_chains.size(); ++chidx) {
        ROS_DEBUG("Chain %zu", chidx);
        ROS_DEBUG("  Segment Count: %u", m_chains[chidx].getNrOfSegments());
        ROS_DEBUG("  Joint Count: %u", m_chains[chidx].getNrOfJoints());
        ROS_DEBUG("  Group Tip: %s", m_chain_index_to_group[chidx]->getName().c_str());
    }

    // map every joint to its chain and index within that chain; a joint may be
    // part of multiple chains (these chains are used to link between the root
    // and the root of a group)
    for (const auto& joint_name : m_joint_names) {
        // create an entry from joint name -> kdl mappings
        auto vit = m_joint_map.insert(std::make_pair(
                joint_name, KDLJointMapping())).first;

        // for each chain
        for (size_t chidx = 0; chidx < m_chains.size(); ++chidx) {
            // find the joint index of this joint in the chain, if any
            const KDL::Chain& chain = m_chains[chidx];
            int jidx = 0;
            for (int sidx = 0; sidx < chain.getNrOfSegments(); ++sidx) {
                const KDL::Segment& seg = chain.getSegment(sidx);

                if (seg.getJoint().getTypeName() == "None") {
                    // skip these joints; they don't contribute to state and
                    // won't have values in the joint array
                    continue;
                }

                if (seg.getJoint().getName() == joint_name) {
                    ROS_DEBUG("Found joint contributor '%s' of type '%s' to chain", joint_name.c_str(), seg.getJoint().getTypeName().c_str());
                    vit->second.chain_indices.push_back(chidx);
                    vit->second.joint_indices.push_back(jidx);
                    break;
                }

                ++jidx;
            }
        }
    }

    ROS_DEBUG("Joint -> KDL Map");
    for (const auto& ent : m_joint_map) {
        ROS_DEBUG("  %s -> (chains: %s, joints: %s)", ent.first.c_str(), to_string(ent.second.chain_indices).c_str(), to_string(ent.second.joint_indices).c_str());
    }

    return true;
}

// this might be a useful decomposition for the group class instead of the way
// it currently works
bool CollisionModelImpl::decomposeRobotModel()
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

    return true;
}

bool CollisionModelImpl::isDescendantOf(
    const std::string& link_a_name,
    const std::string& link_b_name) const
{
    auto link = urdf_->getLink(link_a_name);
    if (!link) {
        ROS_ERROR("Failed to find link '%s'", link_a_name.c_str());
        return false;
    }

    std::queue<boost::shared_ptr<const urdf::Link>> links;
    links.push(link);

    while (!links.empty()) {
        boost::shared_ptr<const urdf::Link> l = links.front();
        links.pop();

        if (l->name == link_b_name) {
            return true;
        }
        else {
            for (const auto& child : l->child_links) {
                links.push(child);
            }
        }
    }

    return false;
}

bool CollisionModelImpl::jointInfluencesLink(
    const std::string& joint_name,
    const std::string& link_name) const
{
    auto joint = urdf_->getJoint(joint_name);
    if (!joint) {
        ROS_ERROR("Failed to find joint '%s'", joint_name.c_str());
        return false;
    }

    const std::string& parent_link_name = joint->parent_link_name;
    return isDescendantOf(parent_link_name, link_name);
}

} // namespace collision
} // namespace sbpl
