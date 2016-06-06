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

#ifndef sbpl_manip_post_processing_h
#define sbpl_manip_post_processing_h

// standard includes
#include <vector>

// system includes
#include <trajectory_msgs/JointTrajectoryPoint.h>

// project includes
#include <sbpl_arm_planner/collision_checker.h>

namespace sbpl {
namespace manip {

void shortcutPath(
    CollisionChecker* cc,
    std::vector<std::vector<double>>& pin,
    std::vector<std::vector<double>>& pout);

void shortcutTrajectory(
    CollisionChecker* cc,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_in,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out);

bool interpolateTrajectory(
    CollisionChecker* cc,
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& traj,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out);

} // namespace manip
} // namespace sbpl

#endif
