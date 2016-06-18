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

#include "collision_model_impl.h"

// standard includes
#include <queue>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

CollisionModelImpl::CollisionModelImpl() :
    m_urdf(),
    m_name(),
    m_model_frame(),
    m_joint_names(),
    m_joint_continuous(),
    m_joint_has_position_bounds(),
    m_joint_min_positions(),
    m_joint_max_positions(),
    m_link_names(),
    m_joint_name_to_index(),
    m_link_name_to_index(),
    m_sphere_models(),
    m_spheres_models(),
    m_voxels_models(),
    m_group_models(),
    m_group_name_to_index(),
    m_link_spheres_models(),
    m_link_voxels_models(),
    m_T_world_model(),
    m_joint_positions(),
    m_dirty_link_transforms(),
    m_link_transforms(),
    m_dirty_voxels_states(),
    m_voxels_states(),
    m_dirty_sphere_states(),
    m_sphere_states(),
    m_link_voxels_states(),
    m_link_sphere_states(),
    m_group_states()
{
}

CollisionModelImpl::~CollisionModelImpl()
{
}

bool CollisionModelImpl::init(
    const std::string& urdf_string,
    const CollisionModelConfig& config)
{
    return initRobotModel(urdf_string) &&
            initRobotState() &&
            initCollisionModel(config) &&
            initCollisionState();
}

bool CollisionModelImpl::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    // TODO: implement: dirty the transforms of all affected links, sphere models,
    // and voxels models
    return false;
}

bool CollisionModelImpl::setJointPosition(int jidx, double position)
{
    ASSERT_VECTOR_RANGE(m_joint_positions, jidx);

    // TODO: implement: set the joint position and dirty the transforms of all
    // affected links, sphere models, and voxels models

    return false;
}

bool CollisionModelImpl::updateLinkTransforms()
{
    // TODO: implement
    return false;
}

bool CollisionModelImpl::updateLinkTransform(int lidx)
{
    // TODO: implement
    return false;
}

bool CollisionModelImpl::updateVoxelsStates()
{
    // TODO: implement
    return false;
}

bool CollisionModelImpl::updateVoxelsState(int vsidx)
{
    // TODO: implement
    return false;
}

bool CollisionModelImpl::updateSpherePositions()
{
    // TODO: implement
    return false;
}

bool CollisionModelImpl::updateSpherePosition(int sidx)
{
    // TODO: implement
    return false;
}

bool CollisionModelImpl::initRobotModel(const std::string& urdf_string)
{
    m_urdf = boost::make_shared<urdf::Model>();
    if (!m_urdf->initString(urdf_string)) {
        ROS_WARN("Failed to parse the URDF");
        return false;
    }

    m_name = m_urdf->getName();

    auto root_link = m_urdf->getRoot();
    m_model_frame = root_link->name;

    // TODO: depth-first or post-traversal reordering to keep dependent
    // joints/links next to one another

    // breadth-first traversal of all links in the robot
    std::queue<boost::shared_ptr<const urdf::Link>> links;
    links.push(root_link);
    while (!links.empty()) {
        boost::shared_ptr<const urdf::Link> link = links.front();
        links.pop();

        m_link_names.push_back(link->name);
        m_link_name_to_index[m_link_names.back()] = m_link_names.size() - 1;

        // for each joint
        for (const auto& joint : link->child_joints) {
            switch (joint->type) {
            case urdf::Joint::REVOLUTE:
            {

            }   break;
            case urdf::Joint::CONTINUOUS:
            {

            }   break;
            case urdf::Joint::PRISMATIC:
            {

            }   break;
            case urdf::Joint::FLOATING:
            {

            }   break;
            case urdf::Joint::PLANAR:
            {

            }   break;
            case urdf::Joint::FIXED:
            {

            }   break;
            default:
            {

            }   break;
            }

            if (joint->type != urdf::Joint::REVOLUTE &&
                joint->type != urdf::Joint::PRISMATIC &&
                joint->type != urdf::Joint::CONTINUOUS)
            {
                ROS_ERROR("Collision Model currently only supports single-dof joint types");
                return false;
            }

            const std::string& joint_name = joint->name;
            double min_position_limit = std::numeric_limits<double>::lowest();
            double max_position_limit = std::numeric_limits<double>::max();
            bool has_position_limit = false;
            bool continuous = (joint->type == urdf::Joint::CONTINUOUS);

            auto limits = joint->limits;
            if (limits) {
                has_position_limit = true;
                min_position_limit = limits->lower;
                max_position_limit = limits->upper;
            }

            m_joint_names.push_back(joint_name);
            m_joint_continuous.push_back(continuous);
            m_joint_has_position_bounds.push_back(has_position_limit);
            m_joint_min_positions.push_back(min_position_limit);
            m_joint_max_positions.push_back(max_position_limit);
            m_joint_name_to_index[joint_name] = m_joint_names.size() - 1;
        }

        // add all child links
        for (const auto& child_link : link->child_links) {
            links.push(child_link);
        }
    }

    ROS_DEBUG("Robot Model Frame: %s", m_model_frame.c_str());

    return true;
}

bool CollisionModelImpl::initRobotState()
{
    m_T_world_model = Eigen::Affine3d::Identity();
    m_joint_positions.resize(m_joint_names.size(), 0.0);
    m_dirty_link_transforms.resize(m_link_names.size(), true);
    m_link_transforms.resize(m_link_names.size(), Eigen::Affine3d::Identity());
    return true;
}

bool CollisionModelImpl::initCollisionModel(const CollisionModelConfig& config)
{
    // initialize sphere models
    m_sphere_models.resize(config.spheres.size());
    for (size_t i = 0; i < m_sphere_models.size(); ++i) {
        const CollisionSphereConfig& sphere_config = config.spheres[i];
        CollisionSphereModel& sphere_model = m_sphere_models[i];
        sphere_model.name = sphere_config.name;
        sphere_model.center = Eigen::Vector3d(sphere_config.x, sphere_config.y, sphere_config.z);
        sphere_model.radius = sphere_config.radius;
        sphere_model.priority = sphere_config.priority;
    }

    // initialize spheres models
    m_spheres_models.resize(config.spheres_models.size());
    for (size_t i = 0; i < m_spheres_models.size(); ++i) {
        const CollisionSpheresModelConfig& spheres_config = config.spheres_models[i];
        CollisionSpheresModel& spheres_model = m_spheres_models[i];
        for (const std::string& sphere_name : spheres_config.spheres) {
            // find the sphere model with this name
            auto sit = std::find_if(m_sphere_models.begin(), m_sphere_models.end(),
                    [&sphere_name](const CollisionSphereModel& sphere)
                    {
                        return sphere.name == sphere_name;
                    });
            if (sit == m_sphere_models.end()) {
                ROS_ERROR("Failed to find sphere '%s' specified in spheres model for link '%s'", sphere_name.c_str(), spheres_config.link_name.c_str());
                return false;
            }
            const CollisionSphereModel* sphere_model =
                    m_sphere_models.data() +
                    std::distance(m_sphere_models.begin(), sit);
            spheres_model.spheres.push_back(sphere_model);
        }
    }

    m_voxels_models.resize(config.voxel_models.size());
    for (size_t i = 0; i < m_voxels_models.size(); ++i) {
        CollisionVoxelsModel& voxel_model = m_voxels_models[i];
        const std::string& link_name = config.voxel_models[i].link_name;
        voxel_model.link_index = linkIndex(link_name);
    }
}

bool CollisionModelImpl::initCollisionState()
{
    return false;
}

bool CollisionModelImpl::isDescendantOf(
    const std::string& link_a_name,
    const std::string& link_b_name) const
{
    auto link = m_urdf->getLink(link_a_name);
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
    auto joint = m_urdf->getJoint(joint_name);
    if (!joint) {
        ROS_ERROR("Failed to find joint '%s'", joint_name.c_str());
        return false;
    }

    const std::string& parent_link_name = joint->parent_link_name;
    return isDescendantOf(parent_link_name, link_name);
}

void CollisionModelImpl::clear()
{
    m_urdf.reset();
    m_model_frame = "";
    m_joint_names.clear();
    m_link_names.clear();
    m_joint_name_to_index.clear();
    m_link_name_to_index.clear();
}

} // namespace collision
} // namespace sbpl
