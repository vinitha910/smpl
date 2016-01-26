////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the University of Pennsylvania nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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

#include <sbpl_manipulation_components/post_processing.h>

#include <leatherman/print.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/interpolation.h>

namespace sbpl_arm_planner {

void shortcutPath(
    sbpl_arm_planner::CollisionChecker* cc,
    std::vector<std::vector<double>>& pin,
    std::vector<std::vector<double>>& pout)
{
    int i = 0;
    int num_checks = 0, path_length = 0;
    unsigned int current = 1;
    double dist = 100;

    if (pin.size() <= 2) {
        ROS_INFO("Path has 2 waypoints or less. Can't shortcut.");
        pout = pin;
        return;
    }

    pout.clear();
    pout.push_back(pin[0]);
    ROS_DEBUG("[keeping %d] %s", 0, to_string(pin[0]).c_str());
    while (current != pin.size()) {
        if (!cc->isStateToStateValid(pin[i], pin[current], path_length, num_checks, dist)) {
            i = current;
            ROS_DEBUG("[keeping %d] %s (num_checks: %d  interp_length: %d)", current, to_string(pin[i]).c_str(), num_checks, path_length);
            pout.push_back(pin[i]);
        }
        else {
            if (current < pin.size()-1) {
                ROS_DEBUG("[cutting %d] %s (num_checks: %d  interp_length: %d)", current, to_string(pin[current]).c_str(), num_checks, path_length);
            }
            else {
                ROS_DEBUG("[keeping %d] %s (num_checks: %d  interp_length: %d)",int(pin.size())-1, to_string(pin[pin.size() - 1]).c_str(), num_checks, path_length);
                pout.push_back(pin[current]);
            }
        }
        current++;
    }
    if (pout.size() == 2) {
        ROS_ERROR("Does it make sense that the shortcutted path length is 2? Is it freespace?");
    }

    ROS_INFO("Original path length: %zu", pin.size());
    ROS_INFO("Shortcutted path length: %zu", pout.size());
}

void shortcutTrajectory(
    sbpl_arm_planner::CollisionChecker* cc,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_in,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out)
{
    std::vector<std::vector<double>> pin(traj_in.size());
    std::vector<std::vector<double>> pout;

    // convert JointTrajectoryPoint vector to vector<vector<double>> repr
    for (size_t j = 0; j < traj_in.size(); ++j) {
        pin[j].resize(traj_in[j].positions.size(),0);
        for (size_t k = 0; k < traj_in[j].positions.size(); ++k) {
            pin[j][k] = traj_in[j].positions[k];
        }
    }

    if (pin.size() > 2) {
        sbpl_arm_planner::shortcutPath(cc, pin, pout);
    }
    else {
        ROS_WARN("Path is too short for shortcutting.");
        pout = pin;
    }

    traj_out.resize(pout.size());
    for (size_t j = 0; j < pout.size(); ++j) {
        for (size_t k = 0; k < pout[j].size(); ++k) {
            traj_out[j].positions.resize(pout[j].size());
            traj_out[j].positions[k] = pout[j][k];
        }
    }
}

bool interpolateTrajectory(
    sbpl_arm_planner::CollisionChecker* cc,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out)
{
    if (traj.empty()) {
        return false;
    }

    const size_t num_joints = traj.front().positions.size();
    for (const auto& pt : traj) {
        if (pt.positions.size() != num_joints) {
            ROS_ERROR("Failed to interpolate trajectory. Input trajectory is malformed");
            return false;
        }
    }

    std::vector<std::vector<double>> path;
    std::vector<double> start(num_joints, 0);
    std::vector<double> end(num_joints, 0);
    std::vector<double> inc(num_joints, 0.069);

    // tack on the first point of the trajectory
    path.push_back(traj.front().positions);

    for (size_t i = 0; i < traj.size() - 1; ++i) {
        start = traj[i].positions;
        end = traj[i + 1].positions;

        ROS_DEBUG_STREAM("Interpolating between " << start << " and " << end);

        std::vector<std::vector<double>> ipath;
        if (!cc->interpolatePath(start, end, inc, ipath)) {
            ROS_ERROR("Failed to interpolate between waypoint %zu and %zu because it's infeasible given the limits.", i, i + 1);
            return false;
        }

        if (!ipath.empty()) {
            // concatenate current path and the intermediate path (we already
            // have the first waypoint in the path from last iteration)
            path.insert(path.end(), ipath.begin() + 1, ipath.end());
        }

        ROS_DEBUG("[%zu] path length: %zu", i, path.size());
    }

    traj_out.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        traj_out[i].positions.resize(path[i].size());
        for (size_t j = 0; j < path[i].size(); ++j) {
            traj_out[i].positions[j] = path[i][j];
        }
        traj_out[i].time_from_start.fromSec(double(i+1) * (traj.back().time_from_start.toSec()/double(path.size())));
    }
    ROS_INFO("Original path length: %d   Interpolated path length: %d", int(traj.size()), int(traj_out.size()));
    return true;
}

} // namespace sbpl_arm_planner
