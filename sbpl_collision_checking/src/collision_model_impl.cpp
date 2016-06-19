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

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <sbpl_geometry_utils/Voxelizer.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

Eigen::Affine3d ComputeRevoluteJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin * Eigen::AngleAxisd(jvals[0], axis);
}

Eigen::Affine3d ComputeContinuousJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin * Eigen::AngleAxisd(angles::normalize_angle(jvals[0]), axis);
}

Eigen::Affine3d ComputePrismaticJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin * Eigen::Translation3d(Eigen::Vector3d(
            0.0, 0.0, jvals[0]));
}

Eigen::Affine3d ComputeFloatingJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin *
            Eigen::Translation3d(Eigen::Vector3d(
                    jvals[0], jvals[1], jvals[2])) *
            Eigen::AngleAxisd(jvals[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(jvals[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(jvals[3], Eigen::Vector3d::UnitZ());
}

Eigen::Affine3d ComputePlanarJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin *
            Eigen::Translation3d(Eigen::Vector3d(jvals[0], jvals[1], 0.0)) *
            Eigen::AngleAxisd(
                    angles::normalize_angle(jvals[2]),
                    Eigen::Vector3d::UnitZ());
}

Eigen::Affine3d ComputeFixedJointTransform(
    const Eigen::Affine3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return origin;
}

CollisionModelImpl::CollisionModelImpl() :
    m_urdf(),
    m_name(),
    m_model_frame(),
    m_jvar_names(),
    m_jvar_continuous(),
    m_jvar_has_position_bounds(),
    m_jvar_min_positions(),
    m_jvar_max_positions(),
    m_jvar_name_to_index(),
    m_joint_origins(),
    m_joint_axes(),
    m_joint_transforms(),
    m_link_names(),
    m_link_name_to_index(),
    m_T_world_model(),
    m_jvar_positions(),
    m_dirty_link_transforms(),
    m_link_transforms(),
    m_sphere_models(),
    m_spheres_models(),
    m_voxels_models(),
    m_group_models(),
    m_group_name_to_index(),
    m_link_spheres_models(),
    m_link_voxels_models(),
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
    ASSERT_VECTOR_RANGE(m_jvar_positions, jidx);

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
            const std::string& joint_name = joint->name;
            auto limits = joint->limits;

            double min_position_limit = std::numeric_limits<double>::lowest();
            double max_position_limit = std::numeric_limits<double>::max();
            bool has_position_limit = false;
            bool continuous = (joint->type == urdf::Joint::CONTINUOUS);

            urdf::Pose origin = joint->parent_to_joint_origin_transform;
            urdf::Vector3 axis = joint->axis;

            if (limits) {
                has_position_limit = true;
                min_position_limit = limits->lower;
                max_position_limit = limits->upper;
            }

            switch (joint->type) {
            case urdf::Joint::FIXED:
            {
                m_jvar_names.push_back(joint_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(std::numeric_limits<double>::quiet_NaN());
                m_jvar_max_positions.push_back(std::numeric_limits<double>::quiet_NaN());

                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeFixedJointTransform);
            }   break;
            case urdf::Joint::REVOLUTE:
            {
                m_jvar_names.push_back(joint_name);
                m_jvar_continuous.push_back(continuous);
                m_jvar_has_position_bounds.push_back(has_position_limit);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);

                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeRevoluteJointTransform);
            }   break;
            case urdf::Joint::PRISMATIC:
            {
                m_jvar_names.push_back(joint_name);
                m_jvar_continuous.push_back(continuous);
                m_jvar_has_position_bounds.push_back(has_position_limit);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);

                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputePrismaticJointTransform);
            }   break;
            case urdf::Joint::CONTINUOUS:
            {
                m_jvar_names.push_back(joint_name);
                m_jvar_continuous.push_back(continuous);
                m_jvar_has_position_bounds.push_back(has_position_limit);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);

                m_jvar_name_to_index[joint_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeContinuousJointTransform);
            }   break;
            case urdf::Joint::PLANAR:
            {
                std::string var_name;

                var_name = joint_name + "/x";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/y";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/yaw";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputePlanarJointTransform);
            }   break;
            case urdf::Joint::FLOATING:
            {
                std::string var_name;

                var_name = joint_name + "/x";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/y";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/z";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/roll";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/pitch";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(false);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                var_name = joint_name + "/yaw";
                m_jvar_names.push_back(var_name);
                m_jvar_continuous.push_back(true);
                m_jvar_has_position_bounds.push_back(false);
                m_jvar_min_positions.push_back(min_position_limit);
                m_jvar_max_positions.push_back(max_position_limit);
                m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

                m_joint_transforms.push_back(ComputeFloatingJointTransform);
            }   break;
            default:
            {
                ROS_ERROR("Unknown joint type encountered");
                return false;
            }   break;
            }

            m_joint_origins.push_back(poseUrdfToEigen(origin));
            m_joint_axes.push_back(Eigen::Vector3d(axis.x, axis.y, axis.z));
        }

        // add all child links
        for (const auto& child_link : link->child_links) {
            links.push(child_link);
        }
    }

    m_joint_var_offset.resize(m_joint_transforms.size());
    double* d = m_jvar_positions.data();
    for (size_t i = 0; i < m_joint_transforms.size(); ++i) {
        m_joint_var_offset[i] = d;
        JointTransformFunction f = m_joint_transforms[i];
        if (f == ComputeFixedJointTransform) {
            d += 0;
        }
        else if (f == ComputeRevoluteJointTransform ||
                f == ComputeContinuousJointTransform ||
                f == ComputePrismaticJointTransform)
        {
            d += 1;
        }
        else if (f == ComputePlanarJointTransform) {
            d += 3;
        }
        else if (f == ComputeFloatingJointTransform) {
            d += 6;
        }
        else {
            ROS_ERROR("Unrecognized JointTransformFunction");
            return false;
        }
    }

    ROS_INFO("Robot Model:");
    ROS_INFO("  Name: %s", m_name.c_str());
    ROS_INFO("  Model Frame: %s", m_model_frame.c_str());
    ROS_INFO("  Joint Variable Names: %s", to_string(m_jvar_names).c_str());
    ROS_INFO("  Joint Variable Continuous: %s", to_string(m_jvar_continuous).c_str());
    ROS_INFO("  Joint Variable Has Position Bounds: %s", to_string(m_jvar_has_position_bounds).c_str());
    ROS_INFO("  Joint Variable Min Positions: %s", to_string(m_jvar_min_positions).c_str());
    ROS_INFO("  Joint Variable Max Positions: %s", to_string(m_jvar_max_positions).c_str());
    ROS_INFO("  Joint Variable Offsets: %s", to_string(m_joint_var_offset).c_str());
    ROS_INFO("  Joint Vairable Transform Functions: %s", to_string(m_joint_transforms).c_str());
    ROS_INFO("  Link Names: %s", to_string(m_link_names).c_str());

    return true;
}

bool CollisionModelImpl::initRobotState()
{
    m_T_world_model = Eigen::Affine3d::Identity();
    m_jvar_positions.assign(m_jvar_names.size(), 0.0);
    m_dirty_link_transforms.assign(m_link_names.size(), true);
    m_link_transforms.assign(m_link_names.size(), Eigen::Affine3d::Identity());
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

        // attach to the link
        spheres_model.link_index = linkIndex(spheres_config.link_name);

        // add references to all spheres
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

    // initialize voxels models
    m_voxels_models.resize(config.voxel_models.size());
    for (size_t i = 0; i < m_voxels_models.size(); ++i) {
        CollisionVoxelsModel& voxels_model = m_voxels_models[i];
        const std::string& link_name = config.voxel_models[i].link_name;
        voxels_model.link_index = linkIndex(link_name);
        const double LINK_VOXEL_RESOLUTION = 0.01;
        voxels_model.voxel_res = LINK_VOXEL_RESOLUTION;
        if (!voxelizeLink(link_name, voxels_model)) {
            ROS_ERROR("Failed to voxelize link '%s'", link_name.c_str());
        }
    }

    // initialize groups
    m_group_models.resize(config.groups.size());
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        CollisionGroupModel& group_model = m_group_models[i];
        const CollisionGroupConfig& group_config = config.groups[i];
        const std::string& group_name = group_config.name;
        group_model.name = group_name;

        for (size_t j = 0; j < group_config.links.size(); ++j) {
            const std::string& link_name = group_config.links[j];
            group_model.link_indices.push_back(linkIndex(link_name));
        }

        m_group_name_to_index[group_name] = i;
    }

    // initialize link spheres models
    m_link_spheres_models.assign(m_link_names.size(), nullptr);
    for (const CollisionSpheresModel& spheres_model : m_spheres_models) {
        m_link_spheres_models[spheres_model.link_index] = &spheres_model;
    }

    // initialize link voxels models
    m_link_voxels_models.assign(m_link_names.size(), nullptr);
    for (const CollisionVoxelsModel& voxels_model : m_voxels_models) {
        m_link_voxels_models[voxels_model.link_index] = &voxels_model;
    }

    return true;
}

bool CollisionModelImpl::initCollisionState()
{
    m_dirty_voxels_states.assign(m_voxels_models.size(), true);
    m_voxels_states.assign(m_voxels_models.size(), CollisionVoxelsState());

    // duplicate voxels from voxels model
    for (size_t i = 0; i < m_voxels_models.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_voxels_models[i];
        CollisionVoxelsState& voxels_state = m_voxels_states[i];
        voxels_state.voxels = voxels_model.voxels;
    }

    // create a sphere state for each unique sphere
    for (CollisionSpheresModel* spheres_model : m_link_spheres_models) {
        if (!spheres_model) {
            continue;
        }

        for (auto sphere_model : spheres_model->spheres)
        {
            m_sphere_states.push_back(CollisionSphereState());
        }
    }

    m_dirty_sphere_states.assign(m_sphere_states.size(), true);

    // initialize link voxels states
    m_link_voxels_states.assign(m_link_models.size(), nullptr);
    for (int i = 0; i < m_voxels_states.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_voxels_models[i];
        CollisionVoxelsState* voxels_state = &m_voxels_states[i];
        m_link_voxels_states[voxels_model.link_index] = voxels_state;
    }

    // initialize link spheres states

    return true;
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
    m_jvar_names.clear();
    m_link_names.clear();
    m_jvar_name_to_index.clear();
    m_link_name_to_index.clear();
}

Eigen::Affine3d CollisionModelImpl::poseUrdfToEigen(const urdf::Pose& p) const
{
    return Eigen::Translation3d(p.position.x, p.position.y, p.position.y) *
            Eigen::Quaterniond(
                    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
}

bool CollisionModelImpl::voxelizeLink(
    const std::string& link_name,
    CollisionVoxelsModel& model) const
{
    auto link = m_urdf->getLink(link_name);

    if (!link) {
        ROS_ERROR("Failed to find link '%s' in the URDF", link_name.c_str());
        return false;
    }

    if (!link->collision && link->collision_array.empty()) {
        ROS_ERROR("Failed to find collision elements of link '%s'", link->name.c_str());
        return false;
    }

    if (link->collision) {
        if (!voxelizeCollisionElement(
            *link->collision, model.voxel_res, model.voxels))
        {
            ROS_ERROR("Failed to voxelize collision element for link '%s'", link_name.c_str());
            return false;
        }
    }
    else if (!link->collision_array.empty()) {
        for (auto collision : link->collision_array) {
            if (!voxelizeCollisionElement(
                    *collision, model.voxel_res, model.voxels))
            {
                ROS_ERROR("Failed to voxelize collision element for link '%s'", link_name.c_str());
                return false;
            }
        }
    }
    else {
        ROS_ERROR("Hmm");
        return false;
    }

    if (model.voxels.empty()) {
        ROS_WARN("Voxelizing collision elements for link '%s' produced 0 voxels", link_name.c_str());
    }

    return true;
}

bool CollisionModelImpl::voxelizeCollisionElement(
    const urdf::Collision& collision,
    double res,
    std::vector<Eigen::Vector3d>& voxels) const
{
    auto geom = collision.geometry;

    if (!geom) {
        ROS_ERROR("Failed to find geometry for collision element");
        return false;
    }

    Eigen::Affine3d pose;

    geometry_msgs::Pose p;
    p.position.x = collision.origin.position.x;
    p.position.y = collision.origin.position.y;
    p.position.z = collision.origin.position.z;
    collision.origin.rotation.getQuaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    tf::poseMsgToEigen(p, pose);

    return voxelizeGeometry(*geom, pose, res, voxels);
}

bool CollisionModelImpl::voxelizeGeometry(
    const urdf::Geometry& geom,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels) const
{
    if (geom.type == urdf::Geometry::MESH) {
        geometry_msgs::Vector3 scale;
        scale.x = 1.0f; scale.y = 1.0f; scale.z = 1.0f;
        std::vector<geometry_msgs::Point> mesh_vertices;
        std::vector<int> triangles;
        urdf::Mesh* mesh = (urdf::Mesh*)&geom;
        if (!leatherman::getMeshComponentsFromResource(
                mesh->filename, scale, triangles, mesh_vertices))
        {
            ROS_ERROR("Failed to get mesh from file. (%s)", mesh->filename.c_str());
            return false;
        }

        ROS_DEBUG("mesh: %s  triangles: %zu  vertices: %zu", mesh->filename.c_str(), triangles.size(), mesh_vertices.size());

        std::vector<Eigen::Vector3d> vertices(mesh_vertices.size());
        for (size_t vidx = 0; vidx < mesh_vertices.size(); ++vidx) {
            const geometry_msgs::Point& vertex = mesh_vertices[vidx];
            vertices[vidx] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
        }

        sbpl::VoxelizeMesh(vertices, triangles, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::BOX) {
        urdf::Box* box = (urdf::Box*)&geom;
        ROS_DEBUG("box: { dims: %0.3f, %0.3f, %0.3f }", box->dim.x, box->dim.y, box->dim.z);
        sbpl::VoxelizeBox(box->dim.x, box->dim.y, box->dim.z, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::CYLINDER) {
        urdf::Cylinder* cyl = (urdf::Cylinder*)&geom;
        ROS_DEBUG("cylinder: { radius: %0.3f, length: %0.3f }", cyl->radius, cyl->length);
        sbpl::VoxelizeCylinder(cyl->radius, cyl->length, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::SPHERE) {
        urdf::Sphere* sph = (urdf::Sphere*)&geom;
        ROS_DEBUG("sphere: { radius: %0.3f }", sph->radius);
        sbpl::VoxelizeSphere(sph->radius, pose, res, voxels, false);
        ROS_DEBUG(" -> voxels: %zu", voxels.size());
    }
    else {
        ROS_ERROR("Unrecognized geometry type for voxelization");
        return false;
    }

    return true;
}

} // namespace collision
} // namespace sbpl
