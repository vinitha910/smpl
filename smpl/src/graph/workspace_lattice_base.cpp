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

#include <smpl/graph/workspace_lattice_base.h>

// system includes
#include <leatherman/print.h>

// project includes
#include <smpl/angles.h>

namespace sbpl {
namespace motion {

WorkspaceLatticeBase::WorkspaceLatticeBase(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams* params,
    OccupancyGrid* grid)
:
    RobotPlanningSpace(robot, checker, params),
    m_grid(grid),
    m_fk_iface(nullptr),
    m_ik_iface(nullptr),
    m_rm_iface(nullptr),
    m_res(),
    m_val_count(),
    m_dof_count(0),
    m_fangle_indices()
{
}

bool WorkspaceLatticeBase::init(const Params& _params)
{
    m_fk_iface = robot()->getExtension<ForwardKinematicsInterface>();
    if (!m_fk_iface) {
        ROS_WARN("Workspace Lattice requires Forward Kinematics Interface extension");
        return false;
    }

    m_ik_iface = robot()->getExtension<InverseKinematicsInterface>();
    if (!m_ik_iface) {
        ROS_WARN("Workspace Lattice requires Inverse Kinematics Interface extension");
        return false;
    }

    m_rm_iface = robot()->getExtension<RedundantManipulatorInterface>();
    if (!m_rm_iface) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return false;
    }

    m_fangle_indices.resize(m_rm_iface->redundantVariableCount());
    for (size_t i = 0; i < m_fangle_indices.size(); ++i) {
        m_fangle_indices[i] = m_rm_iface->redundantVariableIndex(i);
    }
    m_dof_count = 6 + m_fangle_indices.size();

    m_res.resize(m_dof_count);
    m_val_count.resize(m_dof_count);

    m_res[0] = m_grid->getResolution();
    m_res[1] = m_grid->getResolution();
    m_res[2] = m_grid->getResolution();
    // TODO: limit these ranges and handle discretization appropriately
    m_res[3] = 2.0 * M_PI / _params.R_count;
    m_res[4] = M_PI       / _params.P_count;
    m_res[5] = 2.0 * M_PI / _params.Y_count;

    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        m_res[6 + i] = (2.0 * M_PI) / _params.free_angle_res[i];
    }

    m_val_count[0] = std::numeric_limits<int>::max();
    m_val_count[1] = std::numeric_limits<int>::max();
    m_val_count[2] = std::numeric_limits<int>::max();
    m_val_count[3] = _params.R_count;
    m_val_count[4] = _params.P_count;
    m_val_count[5] = _params.Y_count;
    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        m_val_count[6 + i] = (2.0 * M_PI) / _params.free_angle_res[i];
    }

    ROS_INFO("discretization of workspace lattice:");
    ROS_INFO("  x: { res: %0.3f, count: %d }", m_res[0], m_val_count[0]);
    ROS_INFO("  y: { res: %0.3f, count: %d }", m_res[1], m_val_count[1]);
    ROS_INFO("  z: { res: %0.3f, count: %d }", m_res[2], m_val_count[2]);
    ROS_INFO("  R: { res: %0.3f, count: %d }", m_res[3], m_val_count[3]);
    ROS_INFO("  P: { res: %0.3f, count: %d }", m_res[4], m_val_count[4]);
    ROS_INFO("  Y: { res: %0.3f, count: %d }", m_res[5], m_val_count[5]);
    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        ROS_INFO("  J%d: { res: %0.3f, count: %d }", i, m_res[6 + i], m_val_count[6 + i]);
    }

    return true;
}

bool WorkspaceLatticeBase::initialized() const
{
    return (bool)m_fk_iface;
}

void WorkspaceLatticeBase::stateRobotToWorkspace(
    const RobotState& state,
    WorkspaceState& ostate)
{
    SixPose pose;
    bool res = m_fk_iface->computePlanningLinkFK(state, pose);
    assert(res); // forward kinematics shouldn't fail

    ostate.resize(m_dof_count);
    std::copy(pose.begin(), pose.end(), ostate.begin());
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        ostate[6 + fai] = state[m_fangle_indices[fai]];
    }
}

void WorkspaceLatticeBase::stateRobotToCoord(
    const RobotState& state,
    WorkspaceCoord& coord)
{
    WorkspaceState ws_state;
    stateRobotToWorkspace(state, ws_state);
    stateWorkspaceToCoord(ws_state, coord);
}

bool WorkspaceLatticeBase::stateWorkspaceToRobot(
    const WorkspaceState& state,
    RobotState& ostate)
{
    SixPose pose(state.begin(), state.begin() + 6);

    RobotState seed(robot()->jointVariableCount(), 0);
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        seed[m_fangle_indices[fai]] = state[6 + fai];
    }

    ROS_DEBUG_STREAM_NAMED(params()->expands_log, "pose: " << pose << ", seed: " << seed);

    return m_rm_iface->computeFastIK(pose, seed, ostate);
}

void WorkspaceLatticeBase::stateWorkspaceToCoord(
    const WorkspaceState& state,
    WorkspaceCoord& coord)
{
    coord.resize(m_dof_count);
    posWorkspaceToCoord(&state[0], &coord[0]);
    rotWorkspaceToCoord(&state[3], &coord[3]);
    favWorkspaceToCoord(&state[6], &coord[6]);
}

bool WorkspaceLatticeBase::stateCoordToRobot(
    const WorkspaceCoord& coord,
    RobotState& state)
{
    return false;
}

void WorkspaceLatticeBase::stateCoordToWorkspace(
    const WorkspaceCoord& coord,
    WorkspaceState& state)
{
    state.resize(m_dof_count);
    posCoordToWorkspace(&coord[0], &state[0]);
    rotCoordToWorkspace(&coord[3], &state[3]);
    favCoordToWorkspace(&coord[6], &state[6]);
}

bool WorkspaceLatticeBase::stateWorkspaceToRobot(
    const WorkspaceState& state,
    const RobotState& seed,
    RobotState& ostate)
{
    SixPose pose(state.begin(), state.begin() + 6);

    ROS_DEBUG_STREAM_NAMED(params()->expands_log, "pose: " << pose << ", seed: " << seed);

    // TODO: unrestricted variant?
    return m_rm_iface->computeFastIK(pose, seed, ostate);
}

void WorkspaceLatticeBase::posWorkspaceToCoord(const double* wp, int* gp)
{
    m_grid->worldToGrid(wp[0], wp[1], wp[2], gp[0], gp[1], gp[2]);
}

void WorkspaceLatticeBase::posCoordToWorkspace(const int* gp, double* wp)
{
    m_grid->gridToWorld(gp[0], gp[1], gp[2], wp[0], wp[1], wp[2]);
}

void WorkspaceLatticeBase::rotWorkspaceToCoord(const double* wr, int* gr)
{
    gr[0] = (int)((angles::normalize_angle_positive(wr[0]) + m_res[3] * 0.5) / m_res[3]) % m_val_count[3];
    gr[1] = (int)((angles::normalize_angle_positive(wr[1]) + m_res[4] * 0.5) / m_res[4]) % m_val_count[4];
    gr[2] = (int)((angles::normalize_angle_positive(wr[2]) + m_res[5] * 0.5) / m_res[5]) % m_val_count[5];
}

void WorkspaceLatticeBase::rotCoordToWorkspace(const int* gr, double* wr)
{
    wr[0] = angles::normalize_angle((double)gr[0] * m_res[3]);
    wr[1] = angles::normalize_angle((double)gr[1] * m_res[4]);
    wr[2] = angles::normalize_angle((double)gr[2] * m_res[5]);
}

void WorkspaceLatticeBase::poseWorkspaceToCoord(const double* wp, int* gp)
{
    posWorkspaceToCoord(wp, gp);
    rotWorkspaceToCoord(wp + 3, gp + 3);
}

void WorkspaceLatticeBase::poseCoordToWorkspace(const int* gp, double* wp)
{
    posCoordToWorkspace(gp, wp);
    rotCoordToWorkspace(gp + 3, wp + 3);
}

void WorkspaceLatticeBase::favWorkspaceToCoord(const double* wa, int* ga)
{
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        ga[fai] = (int)((angles::normalize_angle_positive(wa[fai]) + m_res[6 + fai] * 0.5) / m_res[6 + fai]) % m_val_count[6 + fai];
    }
}

void WorkspaceLatticeBase::favCoordToWorkspace(const int* ga, double* wa)
{
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        wa[fai] = angles::normalize_angle((double)ga[fai] * m_res[6 + fai]);
    }
}

} // namespace motion
} // namespace sbpl
