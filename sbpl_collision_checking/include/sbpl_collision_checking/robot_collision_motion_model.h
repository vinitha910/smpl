////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#ifndef SBPL_COLLISION_ROBOT_COLLISION_MOTION_MODEL_H
#define SBPL_COLLISION_ROBOT_COLLISION_MOTION_MODEL_H

// standard includes
#include <vector>

// system includes
#include <Eigen/Dense>
#include <smpl/angles.h>
#include <smpl/forward.h>
#include <smpl/types.h>

// project includes
#include <sbpl_collision_checking/robot_collision_model.h>

namespace sbpl {
namespace collision {

SBPL_CLASS_FORWARD(RobotCollisionMotionModel);

/// This class is responsible for on-the-fly linear interpolation of motions,
/// given a desired number of evenly-spaced (in joint space) waypoints along the
/// path.
///
/// Queries may be made to generate the n'th waypoint along the path. The
/// application may decide to generate the interpolation samples in any order
struct MotionInterpolation
{
    motion::RobotState start;
    motion::RobotState diffs;
    int waypoint_count;
    double waypoint_count_inv;

    int waypointCount() const { return waypoint_count; }

    void interpolate(
        int n,
        const std::vector<int>& collision_model_joint_indices,
        motion::RobotState& state)
    {
        state.resize(collision_model_joint_indices.size());
        const double alpha = (double)n * waypoint_count_inv;
        for (size_t vidx = 0; vidx < collision_model_joint_indices.size(); ++vidx) {
            state[vidx] = start[vidx] + alpha * diffs[vidx];
        }
    }
};

/// This class is responsible for determining the maximum distance any sphere on
/// the robot might travel for a given motion between two states. It's primary
/// focus is preparing an interpolation of the path such that no sphere moves
/// by more than a given threshold between two waypoints on the path. This
/// threshold can be derived by application safety margins and be used to
/// determine any additional buffers required to guarantee safe motion between
/// the two states. The robot is assumed to follow a linear motion between the
/// initial and final states of the path.
class RobotCollisionMotionModel
{
public:

    RobotCollisionMotionModel(const RobotCollisionModel* rcm);

    const Eigen::Vector3d& motionCenter(int jidx) const;
    double motionRadius(int jidx) const;

    double getMaxSphereMotion(
        const motion::RobotState& start,
        const motion::RobotState& finish,
        const std::vector<int>& collision_model_joint_indices) const;

    void fillMotionInterpolation(
        const motion::RobotState& start,
        const motion::RobotState& finish,
        const std::vector<int>& collision_model_joint_indices,
        double res,
        MotionInterpolation& motion) const
    {
        motion.start = start;

        motion.diffs.resize(start.size());

        // compute distance traveled by each joint
        std::vector<double> diffs(collision_model_joint_indices.size(), 0.0);
        for (size_t vidx = 0; vidx < collision_model_joint_indices.size(); ++vidx) {
            if (m_rcm->jointVarIsContinuous(vidx)) {
                motion.diffs[vidx] = angles::shortest_angle_diff(finish[vidx], start[vidx]);
            }
            else {
                motion.diffs[vidx] = finish[vidx] - start[vidx];
            }
        }

        double maxMotion =
                getMaxSphereMotion(start, finish, collision_model_joint_indices);
        int waypoint_count = 2;
        if (maxMotion == 0.0) {
            motion.waypoint_count = 0;
            // waypoint_count_inv irrelevant
        } else {
            double maxMotionInv = 1.0 / maxMotion;
            waypoint_count = std::max(waypoint_count, (int)std::ceil(maxMotion / res) + 1);
            motion.waypoint_count = waypoint_count;
            motion.waypoint_count_inv = 1.0 / (double)(waypoint_count - 1);
        }
    }

private:

    const RobotCollisionModel* m_rcm;
    std::vector<Eigen::Vector3d> m_motion_centers;
    std::vector<double> m_motion_radii;
};

inline
const Eigen::Vector3d& RobotCollisionMotionModel::motionCenter(int jidx) const
{
    return m_motion_centers[jidx];
}

inline
double RobotCollisionMotionModel::motionRadius(int jidx) const
{
    return m_motion_radii[jidx];
}

} // namespace collision
} // namespace sbpl

#endif
