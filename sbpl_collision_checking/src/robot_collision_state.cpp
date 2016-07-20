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

#include <sbpl_collision_checking/robot_collision_state.h>

// standard includes
#include <assert.h>
#include <stdlib.h>
#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// system includes
#include <angles/angles.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <leatherman/viz.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <urdf/model.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include "debug.h"
#include "transform_functions.h"
#include "voxel_operations.h"

namespace sbpl {
namespace collision {

static const char* RCM_LOGGER = "robot";

/////////////////////////////////////////
// RobotCollisionStateImpl Declaration //
/////////////////////////////////////////

class RobotCollisionStateImpl
{
public:

    RobotCollisionStateImpl(const RobotCollisionModel* impl);
    ~RobotCollisionStateImpl();

    const RobotCollisionModel* model() const;

    auto   worldToModelTransform() const -> const Eigen::Affine3d&;
    bool   setWorldToModelTransform(const Eigen::Affine3d& transform);

    auto   jointPositions() const -> const std::vector<double>&;
    auto   linkTransforms() const -> const Affine3dVector&;

    double jointPosition(const std::string& joint_name) const;
    double jointPosition(int jidx) const;

    bool   setJointPosition(const std::string& joint_name, double position);
    bool   setJointPosition(int jidx, double position);

    auto   linkTransform(const std::string& link_name) const ->
            const Eigen::Affine3d&;
    auto   linkTransform(int lidx) const -> const Eigen::Affine3d&;

    bool   linkTransformDirty(const std::string& link_name) const;
    bool   linkTransformDirty(int lidx) const;

    bool   updateLinkTransforms();
    bool   updateLinkTransform(int lidx);
    bool   updateLinkTransform(const std::string& link_name);

    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto spheresState(int ssidx) const -> const CollisionSpheresState&;

    auto sphereState(const SphereIndex& sidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(const SphereIndex& sidx) const;
    bool updateSphereStates();
    bool updateSphereState(const SphereIndex& sidx);

    auto   groupSpheresStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupSpheresStateIndices(int gidx) const ->
            const std::vector<int>&;
    auto  groupOutsideVoxelsStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;

    auto getVisualization() const ->
        visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const ->
        visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const ->
        visualization_msgs::MarkerArray;
    auto getStaticModelVisualization() const ->
            visualization_msgs::MarkerArray;
    auto getStaticModelVisualization(const std::string& group_name) const ->
            visualization_msgs::MarkerArray;
    auto getStaticModelVisualization(int gidx) const ->
            visualization_msgs::MarkerArray;
    auto getDynamicModelVisualization() const ->
            visualization_msgs::MarkerArray;
    auto getDynamicModelVisualization(const std::string& group_name) const ->
            visualization_msgs::MarkerArray;
    auto getDynamicModelVisualization(int gidx) const ->
            visualization_msgs::MarkerArray;

private:

    const RobotCollisionModel* m_model;

    /// \name Robot State
    ///@{
    std::vector<double>     m_jvar_positions;           // per variable
    std::vector<double*>    m_joint_var_offsets;        // per joint
    std::vector<bool>       m_dirty_link_transforms;    // per link
    Affine3dVector          m_link_transforms;          // per link
    ///@}

    /// \name Collision State
    ///@{

    std::vector<bool>                       m_dirty_sphere_states; // per-sphere
    std::vector<int>                        m_sphere_offsets;
    std::vector<CollisionSpheresState>      m_spheres_states;

    // one entry for each voxels model
    std::vector<bool>                       m_dirty_voxels_states;
    std::vector<CollisionVoxelsState>       m_voxels_states;

    // one entry for each corresponding group model
    std::vector<CollisionGroupState>        m_group_states;

    // per-link references to corresponding spheres and voxels states
    std::vector<CollisionVoxelsState*>      m_link_voxels_states;
    std::vector<CollisionSpheresState*>     m_link_spheres_states;

    ///@}

    void initRobotState();
    void initCollisionState();

    bool checkCollisionStateReferences() const;

    bool updateSpheresStateToSphereStatesReferences();
    bool updateVoxelsStateToModelReferences();
    bool updateLinkBodyToSpheresStateReferences();
    bool updateLinkBodyToVoxelsStateReferences();

    int sphereIndex(const SphereIndex& sidx) const;
};

////////////////////////////////////////
// RobotCollisionStateImpl Definition //
////////////////////////////////////////

RobotCollisionStateImpl::RobotCollisionStateImpl(
    const RobotCollisionModel* model)
:
    m_model(model),
    m_jvar_positions(),
    m_dirty_link_transforms(),
    m_link_transforms(),
    m_dirty_sphere_states(),
    m_dirty_voxels_states(),
    m_voxels_states(),
    m_group_states(),
    m_link_voxels_states(),
    m_link_spheres_states()
{
    initRobotState();
    initCollisionState();
}

RobotCollisionStateImpl::~RobotCollisionStateImpl()
{
}

inline
const std::vector<int>& RobotCollisionStateImpl::groupSpheresStateIndices(
    const std::string& group_name) const
{
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>& RobotCollisionStateImpl::groupSpheresStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>& RobotCollisionStateImpl::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].voxels_indices;
}

inline
const std::vector<int>& RobotCollisionStateImpl::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].voxels_indices;
}

inline
const RobotCollisionModel* RobotCollisionStateImpl::model() const
{
    return m_model;
}

inline
const Eigen::Affine3d& RobotCollisionStateImpl::worldToModelTransform() const
{
    return m_link_transforms[0];
}

bool RobotCollisionStateImpl::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    if (!transform.isApprox(m_link_transforms[0], 0.0)) {
        m_link_transforms[0] = transform;
        std::fill(m_dirty_link_transforms.begin(), m_dirty_link_transforms.end(), true);
        std::fill(m_dirty_voxels_states.begin(), m_dirty_voxels_states.end(), true);
        std::fill(m_dirty_sphere_states.begin(), m_dirty_sphere_states.end(), true);
        return true;
    }
    else {
        return false;
    }
}

inline
const std::vector<double>& RobotCollisionStateImpl::jointPositions() const
{
    return m_jvar_positions;
}

inline
const Affine3dVector& RobotCollisionStateImpl::linkTransforms() const
{
    return m_link_transforms;
}

inline
double RobotCollisionStateImpl::jointPosition(const std::string& joint_name) const
{
    const int jidx = m_model->jointVarIndex(joint_name);
    return m_jvar_positions[jidx];
}

inline
double RobotCollisionStateImpl::jointPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, jidx);
    return m_jvar_positions[jidx];
}

inline
bool RobotCollisionStateImpl::setJointPosition(
    const std::string& joint_name,
    double position)
{
    const int jidx = m_model->jointVarIndex(joint_name);
    return setJointPosition(jidx, position);
}

bool RobotCollisionStateImpl::setJointPosition(int jidx, double position)
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, jidx);

    if (m_jvar_positions[jidx] != position) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "Setting joint position of joint %d to %0.3f", jidx, position);

        m_jvar_positions[jidx] = position;

        // TODO: cache affected link transforms in a per-joint array?

        std::queue<int> q;
        q.push(m_model->jointChildLinkIndex(jidx));
        while (!q.empty()) {
            int lidx = q.front();
            q.pop();

            ROS_DEBUG_NAMED(RCM_LOGGER, "Dirtying transform to link '%s'", m_model->linkName(lidx).c_str());

            // dirty the transform of the affected link
            m_dirty_link_transforms[lidx] = true;

            // dirty the voxels states of any attached voxels model
            if (m_link_voxels_states[lidx]) {
                int dvsidx = std::distance(m_voxels_states.data(), m_link_voxels_states[lidx]);
                m_dirty_voxels_states[dvsidx] = true;
            }

            // dirty the sphere states of any attached sphere models
            if (m_link_spheres_states[lidx]) {
                CollisionSpheresState* spheres_state;
                int ssidx = std::distance(m_spheres_states.data(), sphere_state);
                int off = m_sphere_offsets[ssidx];
                std::assign(
                        &m_dirty_sphere_states[off],
                        &m_dirty_sphere_states[off] + spheres_state->spheres.size(),
                        true);
            }

            // add child links to the queue
            for (int cjidx : m_model->linkChildJointIndices(lidx)) {
                q.push(m_model->jointChildLinkIndex(cjidx));
            }
        }

        return true;
    }
    else {
        return false;
    }
}

inline
const Eigen::Affine3d& RobotCollisionStateImpl::linkTransform(
    const std::string& link_name) const
{
    const int lidx = m_model->linkIndex(link_name);
    return m_link_transforms[lidx];
}

inline
const Eigen::Affine3d& RobotCollisionStateImpl::linkTransform(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_transforms, lidx);
    return m_link_transforms[lidx];
}

inline
bool RobotCollisionStateImpl::linkTransformDirty(const std::string& link_name) const
{
    const int lidx = m_model->linkIndex(link_name);
    return m_dirty_link_transforms[lidx];
}

inline
bool RobotCollisionStateImpl::linkTransformDirty(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    return m_dirty_link_transforms[lidx];
}

bool RobotCollisionStateImpl::updateLinkTransforms()
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating all link transforms");
    bool updated = false;
    for (size_t lidx = 0; lidx < m_model->linkCount(); ++lidx) {
        updated |= updateLinkTransform(lidx);
    }
    return updated;
}

bool RobotCollisionStateImpl::updateLinkTransform(int lidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_link_transforms, lidx);
    if (!m_dirty_link_transforms[lidx]) {
        return false;
    }

    int pjidx = m_model->linkParentJointIndex(lidx);
    int plidx = m_model->jointParentLinkIndex(pjidx);

    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating transform for link '%s'. parent joint = %d, parent link = %d", m_model->linkName(lidx).c_str(), pjidx, plidx);

    if (plidx >= 0) {
        // recursively update the kinematic tree
        // TODO: optimize out recursion
        updateLinkTransform(plidx);
    }

    if (plidx >= 0) {
        const Eigen::Affine3d& T_world_parent = m_link_transforms[plidx];
        JointTransformFunction fn = m_model->jointTransformFn(pjidx);
        const Eigen::Affine3d& joint_origin = m_model->jointOrigin(pjidx);
        const Eigen::Vector3d& joint_axis = m_model->jointAxis(pjidx);
        double* variables = m_joint_var_offsets[pjidx];
        const Eigen::Affine3d& T_parent_link = fn(joint_origin, joint_axis, variables);
        m_link_transforms[lidx] = T_world_parent * T_parent_link;
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, " -> %s", AffineToString(m_link_transforms[lidx]).c_str());

    m_dirty_link_transforms[lidx] = false;
    return true;
}

inline
bool RobotCollisionStateImpl::updateLinkTransform(const std::string& link_name)
{
    const int lidx = m_model->linkIndex(link_name);
    return updateLinkTransform(lidx);
}

inline
const CollisionVoxelsState& RobotCollisionStateImpl::voxelsState(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    return m_voxels_states[vsidx];
}

inline
bool RobotCollisionStateImpl::voxelsStateDirty(int vsidx) const
{
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);
    return m_dirty_voxels_states[vsidx];
}

bool RobotCollisionStateImpl::updateVoxelsStates()
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating all voxels states");
    bool updated = false;
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        updated |= updateVoxelsState(vsidx);
    }
    return updated;
}

bool RobotCollisionStateImpl::updateVoxelsState(int vsidx)
{
    ASSERT_VECTOR_RANGE(m_dirty_voxels_states, vsidx);

    if (!m_dirty_voxels_states[vsidx]) {
        return false;
    }

    CollisionVoxelsState& state = m_voxels_states[vsidx];

    std::vector<Eigen::Vector3d> new_voxels;
    const int lidx = state.model->link_index;
    updateLinkTransform(lidx);

    const Eigen::Affine3d& T_model_link = m_link_transforms[lidx];

    // transform voxels into the model frame
    new_voxels.resize(state.model->voxels.size());
    for (size_t i = 0; i < state.model->voxels.size(); ++i) {
        new_voxels[i] = T_model_link * state.model->voxels[i];
    }

    state.voxels = std::move(new_voxels);
    m_dirty_voxels_states[vsidx] = false;
    return true;
}

const CollisionSpheresState& RobotCollisionStateImpl::spheresState(
    int ssidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_states, ssidx);
    return m_spheres_states[ssidx];
}

inline
const CollisionSphereState& RobotCollisionStateImpl::sphereState(
    const SphereIndex& sidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_states, sidx.ss);
    ASSERT_VECTOR_RANGE(m_spheres_states[sidx.ss], sidx.s);
    return m_spheres_states[sidx.ss].spheres[sidx.s];
}

inline
bool RobotCollisionStateImpl::sphereStateDirty(const SphereIndex& sidx) const
{
    const int idx = sphereIndex(sidx);
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, idx);
    return m_dirty_sphere_states[idx];
}

bool RobotCollisionStateImpl::updateSphereStates()
{
    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating all sphere positions");
    bool updated = false;
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        for (size_t sidx = 0; sidx < spheres_state.spheres.size(); ++sidx) {
            updated |= updateSphereState(SphereIndex(ssidx, sidx));
        }
    }
    return updated;
}

bool RobotCollisionStateImpl::updateSphereState(const SphereIndex& sidx)
{
    const int idx = sphereIndex(sidx);
    ASSERT_VECTOR_RANGE(m_dirty_sphere_states, idx);

    if (!m_dirty_sphere_states[idx]) {
        return false;
    }

    CollisionSphereState& sphere_state = m_spheres_states[sidx.ss].spheres[sidx.s];

    const int lidx = sphere_state.parent_state->model->link_index;
    updateLinkTransform(lidx);

    ROS_DEBUG_NAMED(RCM_LOGGER, "Updating position of sphere '%s'", sphere_state.model->name.c_str());
    const Eigen::Affine3d& T_model_link = m_link_transforms[lidx];
    sphere_state.pos = T_model_link * sphere_state.model->center;

    m_dirty_sphere_states[idx] = false;
    return true;
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getVisualization() const
{
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < m_model->groupCount(); ++i) {
        auto marr = getVisualization(i);
        ma.markers.insert(ma.markers.end(), marr.markers.begin(), marr.markers.end());
    }
    return ma;
}

inline
visualization_msgs::MarkerArray RobotCollisionStateImpl::getVisualization(
    const std::string& group_name) const
{
    const int gidx = m_model->groupIndex(group_name);
    return getVisualization(gidx);
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getVisualization(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    const CollisionGroupState& group_state = m_group_states[gidx];

    std::vector<std::vector<double>> spheres;
    std::vector<double> rad;

    size_t sphere_count = 0;
    for (int ssidx : group_state.spheres_indices) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        for (const CollisionSphereState& sphere_state : spheres_state.spheres) {
            ++sphere_count;
        }
    }

    spheres.reserve(sphere_count);
    rad.reserve(sphere_count);

    for (int ssidx : group_state.spheres_indices) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        for (const CollisionSphereState& sphere_state : spheres_state.spheres) {
            std::vector<double> sphere(4, 0.0);
            sphere[0] = sphere_state.pos.x();
            sphere[1] = sphere_state.pos.y();
            sphere[2] = sphere_state.pos.z();
            sphere[3] = sphere_state.model->radius;
            spheres.push_back(std::move(sphere));
            rad.push_back(sphere_state.model->radius);
        }
    }

    const int hue = 90;
    return viz::getSpheresMarkerArray(
            spheres, rad, hue, "", "collision_model", 0);
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getStaticModelVisualization() const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getStaticModelVisualization(
    const std::string& group_name) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getStaticModelVisualization(int gidx) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getDynamicModelVisualization() const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getDynamicModelVisualization(
    const std::string& group_name) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
RobotCollisionStateImpl::getDynamicModelVisualization(int gidx) const
{
    throw std::runtime_error("unimplemented");
    return visualization_msgs::MarkerArray();
}

void RobotCollisionStateImpl::initRobotState()
{
    // initialize joint variable offsets
    // NOTE: need to initialize this before initRobotState to set up these
    // references
    m_jvar_positions.assign(m_model->jointVarCount(), 0.0);

    m_joint_var_offsets.resize(m_model->jointCount());
    double* d = m_jvar_positions.data();
    for (size_t i = 0; i < m_model->jointCount(); ++i) {
        m_joint_var_offsets[i] = d;
        JointTransformFunction f = m_model->jointTransformFn(i);
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
            d += 7;
        }
        else {
            ROS_ERROR_NAMED(RCM_LOGGER, "Unrecognized JointTransformFunction");
            throw std::runtime_error("Unrecognized JointTransformFunction");
        }
    }

    m_dirty_link_transforms.assign(m_model->linkCount(), true);
    m_link_transforms.assign(m_model->linkCount(), Eigen::Affine3d::Identity());

    ROS_DEBUG_NAMED(RCM_LOGGER, "Robot State:");
    ROS_DEBUG_NAMED(RCM_LOGGER, "  %zu Joint Positions", m_jvar_positions.size());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  %zu Dirty Link Transforms", m_dirty_link_transforms.size());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  %zu Link Transforms", m_link_transforms.size());
}

void RobotCollisionStateImpl::initCollisionState()
{
    // initialize sphere and spheres states
    m_spheres_states.assign(m_model->spheresModelCount(), CollisionSpheresState());
    int offset = 0;
    for (size_t i = 0; i < m_model->spheresModelCount(); ++i) {
        m_sphere_offsets[i] = offset;
        const CollisionSpheresModel& spheres_model = m_model->spheresModel(i);
              CollisionSpheresState& spheres_state = m_spheres_states[i];

        // map spheres state -> spheres model
        spheres_state.model = &spheres_model;

        // map sphere state -> sphere model
        // map sphere state -> spheres state
        spheres_state.spheres.resize(spheres_model.spheres.size());
        for (size_t j = 0; j < spheres_model.spheres.size(); ++j) {
            spheres_state.spheres[j].model = &spheres_model.spheres[j];
            spheres_state.spheres[j].parent_state = &spheres_state;
            ++offset;
        }
    }

    m_dirty_sphere_states.assign(sphereModelCount(), true);

    // initialize voxels states
    m_dirty_voxels_states.assign(m_model->voxelsModelCount(), true);
    m_voxels_states.assign(m_model->voxelsModelCount(), CollisionVoxelsState());

    for (size_t i = 0; i < m_model->voxelsModelCount(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
              CollisionVoxelsState& voxels_state = m_voxels_states[i];
        // duplicate voxels from voxels model
        voxels_state.model = &voxels_model;
        voxels_state.voxels = voxels_model.voxels;
    }

    // initialize group states
    m_group_states.assign(m_model->groupCount(), CollisionGroupState());
    for (size_t i = 0; i < m_model->groupCount(); ++i) {
        const CollisionGroupModel& group_model = m_model->group(i);
              CollisionGroupState& group_state = m_group_states[i];
        // map group state -> group model
        group_state.model = &group_model;
    }

    // map group state -> spheres states
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        // get the link this spheres state is attached to
        const int lidx = spheres_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) !=
                group_model.link_indices.end())
            {
                m_group_states[gidx].spheres_indices.push_back(ssidx);
            }
        }
    }

    // map group state -> (outside) voxels states
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        const CollisionVoxelsState& voxels_state = m_voxels_states[vsidx];
        // get the link this spheres state is attached to
        const int lidx = voxels_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) ==
                group_model.link_indices.end())
            {
                m_group_states[gidx].voxels_indices.push_back(vsidx);
            }
        }
    }

    // initialize link voxels states
    m_link_voxels_states.assign(m_model->linkCount(), nullptr);
    for (int i = 0; i < m_voxels_states.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
        CollisionVoxelsState* voxels_state = &m_voxels_states[i];
        m_link_voxels_states[voxels_model.link_index] = voxels_state;
    }

    // initialize link spheres states
    m_link_spheres_states.assign(m_model->linkCount(), nullptr);
    for (int i = 0; i < m_spheres_states.size(); ++i) {
        const CollisionSpheresModel& spheres_model = m_model->spheresModel(i);
        CollisionSpheresState* spheres_state = &m_spheres_states[i];
        m_link_spheres_states[spheres_model.link_index] = spheres_state;
    }

    assert(checkCollisionStateReferences());

    ROS_DEBUG_NAMED(RCM_LOGGER, "Collision State:");
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Dirty Sphere States: %zu", m_dirty_sphere_states.size());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres States: [%p, %p]", m_spheres_states.data(), m_spheres_states.data() + m_spheres_states.size());
    for (const auto& spheres_state : m_spheres_states) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    model: %p, spheres: %s", spheres_state.model, to_string(spheres_state.spheres).c_str());
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Dirty Voxels States: %zu", m_dirty_voxels_states.size());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Voxels States: [%p, %p]", m_voxels_states.data(), m_voxels_states.data() + m_voxels_states.size());
    for (const auto& voxels_state : m_voxels_states) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    model: %p, voxels: %zu", voxels_state.model, voxels_state.voxels.size());
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Group States: [%p, %p]", m_group_states.data(), m_group_states.data() + m_group_states.size());
    for (const auto& group_state : m_group_states) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    model: %p, spheres_indices: %s, voxels_indices: %s", group_state.model, to_string(group_state.spheres_indices).c_str(), to_string(group_state.voxels_indices).c_str());
    }
}

bool RobotCollisionStateImpl::checkCollisionStateReferences() const
{
    // c++14 would make me happier here...wtb generic lambdas :(
    auto within = [](const void* ptr, const void* start, const void* end) {
        return ptr >= start && ptr < end;
    };

    // check valid sphere state -> spheres state references
    for (const auto& spheres_state : m_spheres_states) {
        for (const auto& sphere_state : spheres_state.spheres) {
            if (!within(sphere_state.parent_state,
                    m_spheres_states.data(),
                    m_spheres_states.data() + m_spheres_states.size()))
            {
                return false;
            }
        }
    }

    for (const auto& group_state : m_group_states) {
        // check valid group state -> spheres states references
        if (std::any_of(
                group_state.spheres_indices.begin(),
                group_state.spheres_indices.end(),
                [&](int ssidx)
                {
                    return ssidx < 0 || ssidx >= m_spheres_states.size();
                }))
        {
            return false;
        }

        // check valid group state -> voxels states references
        if (std::any_of(
                group_state.voxels_indices.begin(),
                group_state.voxels_indices.end(),
                [&](int vsidx)
                {
                    return vsidx < 0 || vsidx >= m_voxels_states.size();
                }))
        {
            return false;
        }
    }

    return true;
}

{
    return true;
}

int RobotCollisionStateImpl::sphereIndex(const SphereIndex& sidx) const
{
    return m_sphere_offsets[sidx.ss] + sidx.s;
}

////////////////////////////////////////
// RobotCollisionModel Implementation //
////////////////////////////////////////

RobotCollisionState::RobotCollisionState(const RobotCollisionModel* model) :
    m_impl(new RobotCollisionStateImpl(model))
{
}

const RobotCollisionModel* RobotCollisionState::model() const
{
    return m_impl->model();
}

RobotCollisionState::~RobotCollisionState()
{
}

const Eigen::Affine3d& RobotCollisionState::worldToModelTransform() const
{
    return m_impl->worldToModelTransform();
}

bool RobotCollisionState::setWorldToModelTransform(
    const Eigen::Affine3d& transform)
{
    return m_impl->setWorldToModelTransform(transform);
}

const std::vector<double>& RobotCollisionState::jointPositions() const
{
    return m_impl->jointPositions();
}

const Affine3dVector& RobotCollisionState::linkTransforms() const
{
    return m_impl->linkTransforms();
}

double RobotCollisionState::jointPosition(const std::string& joint_name) const
{
    return m_impl->jointPosition(joint_name);
}

double RobotCollisionState::jointPosition(int jidx) const
{
    return m_impl->jointPosition(jidx);
}

bool RobotCollisionState::setJointPosition(
    const std::string& name,
    double position)
{
    return m_impl->setJointPosition(name, position);
}

bool RobotCollisionState::setJointPosition(int jidx, double position)
{
    return m_impl->setJointPosition(jidx, position);
}

const Eigen::Affine3d& RobotCollisionState::linkTransform(
    const std::string& link_name) const
{
    return m_impl->linkTransform(link_name);
}

const Eigen::Affine3d& RobotCollisionState::linkTransform(int lidx) const
{
    return m_impl->linkTransform(lidx);
}

bool RobotCollisionState::linkTransformDirty(const std::string& link_name) const
{
    return m_impl->linkTransformDirty(link_name);
}

bool RobotCollisionState::linkTransformDirty(int lidx) const
{
    return m_impl->linkTransformDirty(lidx);
}

bool RobotCollisionState::updateLinkTransforms()
{
    return m_impl->updateLinkTransforms();
}

bool RobotCollisionState::updateLinkTransform(int lidx)
{
    return m_impl->updateLinkTransform(lidx);
}

bool RobotCollisionState::updateLinkTransform(const std::string& link_name)
{
    return m_impl->updateLinkTransform(link_name);
}

const CollisionVoxelsState& RobotCollisionState::voxelsState(int vsidx) const
{
    return m_impl->voxelsState(vsidx);
}

bool RobotCollisionState::voxelsStateDirty(int vsidx) const
{
    return m_impl->voxelsStateDirty(vsidx);
}

bool RobotCollisionState::updateVoxelsStates()
{
    return m_impl->updateVoxelsStates();
}

bool RobotCollisionState::updateVoxelsState(int vsidx)
{
    return m_impl->updateVoxelsState(vsidx);
}

const CollisionSphereState& RobotCollisionState::sphereState(
    const SphereIndex& sidx) const
{
    return m_impl->sphereState(sidx);
}

bool RobotCollisionState::sphereStateDirty(const SphereIndex& sidx) const
{
    return m_impl->sphereStateDirty(sidx);
}

bool RobotCollisionState::updateSphereStates()
{
    return m_impl->updateSphereStates();
}

bool RobotCollisionState::updateSphereState(const SphereIndex& sidx)
{
    return m_impl->updateSphereState(sidx);
}

const std::vector<int>& RobotCollisionState::groupSpheresStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupSpheresStateIndices(group_name);
}

const std::vector<int>& RobotCollisionState::groupSpheresStateIndices(
    int gidx) const
{
    return m_impl->groupSpheresStateIndices(gidx);
}

const std::vector<int>& RobotCollisionState::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    return m_impl->groupOutsideVoxelsStateIndices(group_name);
}

const std::vector<int>& RobotCollisionState::groupOutsideVoxelsStateIndices(
    int gidx) const
{
    return m_impl->groupOutsideVoxelsStateIndices(gidx);
}

visualization_msgs::MarkerArray
RobotCollisionState::getVisualization() const
{
    return m_impl->getVisualization();
}

visualization_msgs::MarkerArray
RobotCollisionState::getVisualization(const std::string& group_name) const
{
    return m_impl->getVisualization(group_name);
}

visualization_msgs::MarkerArray
RobotCollisionState::getVisualization(int gidx) const
{
    return m_impl->getVisualization(gidx);
}

visualization_msgs::MarkerArray
RobotCollisionState::getStaticModelVisualization() const
{
    return m_impl->getStaticModelVisualization();
}

visualization_msgs::MarkerArray
RobotCollisionState::getStaticModelVisualization(
    const std::string& group_name) const
{
    return m_impl->getStaticModelVisualization(group_name);
}

visualization_msgs::MarkerArray
RobotCollisionState::getStaticModelVisualization(int gidx) const
{
    return m_impl->getStaticModelVisualization(gidx);
}

visualization_msgs::MarkerArray
RobotCollisionState::getDynamicModelVisualization() const
{
    return m_impl->getDynamicModelVisualization();
}

visualization_msgs::MarkerArray
RobotCollisionState::getDynamicModelVisualization(
    const std::string& group_name) const
{
    return m_impl->getDynamicModelVisualization(group_name);
}

visualization_msgs::MarkerArray
RobotCollisionState::getDynamicModelVisualization(int gidx) const
{
    return m_impl->getDynamicModelVisualization(gidx);
}

} // namespace collision
} // namespace sbpl
