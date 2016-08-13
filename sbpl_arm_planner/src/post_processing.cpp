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

#include <sbpl_arm_planner/post_processing.h>

// standard includes
#include <chrono>

// system includes
#include <Eigen/Dense>
#include <leatherman/print.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>
#include <sbpl_geometry_utils/shortcut.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace sbpl {
namespace manip {

class ShortcutPathGenerator
{
public:

    ShortcutPathGenerator(CollisionChecker* cc) :
        m_cc(cc)
    { }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& end,
        OutputIt ofirst, int& cost) const
    {
        int path_length;
        int num_checks;
        double dist;
        if (m_cc->isStateToStateValid(
                start, end, path_length, num_checks, dist))
        {
            *ofirst++ = start;
            *ofirst++ = end;
            cost = 0;
            return true;
        }
        else {
            return false;
        }
    }

private:

    CollisionChecker* m_cc;
};

class EuclidShortcutPathGenerator
{
public:

    EuclidShortcutPathGenerator(RobotModel* rm, CollisionChecker* cc) :
        m_rm(rm),
        m_cc(cc)
    { }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& end,
        OutputIt ofirst, int& cost) const
    {
        // compute forward kinematics for the start an end configurations
        std::vector<double> from_pose, to_pose;
        if (!m_rm->computePlanningLinkFK(start, from_pose) ||
            !m_rm->computePlanningLinkFK(end, to_pose))
        {
            return false;
        }

        Eigen::Vector3d pstart(from_pose[0], from_pose[1], from_pose[2]);
        Eigen::Vector3d pend(to_pose[0], to_pose[1], to_pose[2]);

        Eigen::Quaterniond qstart(
                Eigen::AngleAxisd(from_pose[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(from_pose[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(from_pose[3], Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond qend(
                Eigen::AngleAxisd(to_pose[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(to_pose[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(to_pose[3], Eigen::Vector3d::UnitX()));

        // compute the number of path waypoints
        double posdiff = (pend - pstart).norm();
        double rotdiff = Eigen::AngleAxisd(qstart.inverse() * qend).angle();

        const double interp_pres = 0.01;
        const double interp_rres = sbpl::utils::ToRadians(5.0);

        int num_points = 2;
        num_points = std::max(num_points, (int)ceil(posdiff / interp_pres));
        num_points = std::max(num_points, (int)ceil(rotdiff / interp_rres));

        std::vector<RobotState> cpath;

        cpath.push_back(start);
        for (int i = 1; i < num_points; ++i) {
            // compute the intermediate pose
            double alpha = (double)i / (double)(num_points - 1);
            Eigen::Vector3d ppos = (1.0 - alpha) * pstart + alpha * pend;
            Eigen::Quaterniond prot = qstart.slerp(alpha, qend);

            const Eigen::Affine3d ptrans = prot * Eigen::Translation3d(ppos);

            tf::Matrix3x3 mat;
            mat.setValue(
                ptrans(0, 0), ptrans(0, 1), ptrans(0, 2),
                ptrans(1, 0), ptrans(1, 1), ptrans(1, 2),
                ptrans(2, 0), ptrans(2, 1), ptrans(2, 2));
            double R, P, Y;
            mat.getEulerYPR(Y, P, R);

            std::vector<double> ipose(6, 0.0);
            ipose[0] = ppos.x();
            ipose[1] = ppos.y();
            ipose[2] = ppos.z();
            ipose[3] = R;
            ipose[4] = P;
            ipose[5] = Y;

            // run inverse kinematics with the previous pose as the seed state
            const RobotState& prev_wp = cpath.back();
            RobotState wp(m_rm->getPlanningJoints().size(), 0.0);
            if (!m_rm->computeIK(ipose, prev_wp, wp)) {
                return false;
            }

            // check the path segment for collisions
            int path_length, num_checks;
            double dist;
            if (!m_cc->isStateToStateValid(
                    prev_wp, wp, path_length, num_checks, dist))
            {
                return false;
            }

            cpath.push_back(wp);
        }

        for (auto& point : cpath) {
            *ofirst++ = std::move(point);
        }
        cost = 0;
        return true;
    }

private:

    RobotModel* m_rm;
    CollisionChecker* m_cc;
};

void ShortcutPath(
    RobotModel* rm,
    CollisionChecker* cc,
    std::vector<RobotState>& pin,
    std::vector<RobotState>& pout,
    ShortcutType type)
{
    if (pin.size() < 2) {
        pout = pin;
        return;
    }

    std::vector<int> costs(pin.size() - 1, 1);

    switch (type) {
    case ShortcutType::JOINT_SPACE:
    {
        ShortcutPathGenerator generators[] = { ShortcutPathGenerator(cc) };
        shortcut::ShortcutPath(
                pin.begin(), pin.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(pout));
    }   break;
    case ShortcutType::EUCLID_SPACE:
    {
        EuclidShortcutPathGenerator generators[] =
                { EuclidShortcutPathGenerator(rm, cc) };

        auto then = std::chrono::high_resolution_clock::now();
        shortcut::ShortcutPath(
                pin.begin(), pin.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(pout));
        auto now = std::chrono::high_resolution_clock::now();
        ROS_INFO("Path shortcutting took %0.3f seconds", std::chrono::duration<double>(now - then).count());
    }   break;
    default:
        break;
    }

    ROS_INFO("Original path length: %zu", pin.size());
    ROS_INFO("Shortcutted path length: %zu", pout.size());
}

void ShortcutTrajectory(
    RobotModel* rm,
    CollisionChecker* cc,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_in,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out,
    ShortcutType type)
{
    std::vector<RobotState> pin(traj_in.size());
    std::vector<RobotState> pout;

    // convert JointTrajectoryPoint vector to vector<vector<double>> repr
    for (size_t j = 0; j < traj_in.size(); ++j) {
        pin[j].resize(traj_in[j].positions.size(),0);
        for (size_t k = 0; k < traj_in[j].positions.size(); ++k) {
            pin[j][k] = traj_in[j].positions[k];
        }
    }

    if (pin.size() > 2) {
        ShortcutPath(rm, cc, pin, pout, type);
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

bool InterpolateTrajectory(
    CollisionChecker* cc,
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& traj,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_out)
{
    if (traj.empty()) {
        return true;
    }

    const size_t num_joints = traj.front().positions.size();
    for (const auto& pt : traj) {
        if (pt.positions.size() != num_joints) {
            ROS_ERROR("Failed to interpolate trajectory. Input trajectory is malformed");
            return false;
        }
    }

    std::vector<RobotState> path;
    RobotState start(num_joints, 0);
    RobotState end(num_joints, 0);

    // tack on the first point of the trajectory
    path.push_back(traj.front().positions);

    // iterate over path segments
    for (size_t i = 0; i < traj.size() - 1; ++i) {
        start = traj[i].positions;
        end = traj[i + 1].positions;

        ROS_DEBUG_STREAM("Interpolating between " << start << " and " << end);

        std::vector<RobotState> ipath;
        if (!cc->interpolatePath(start, end, ipath)) {
            ROS_ERROR("Failed to interpolate between waypoint %zu and %zu because it's infeasible given the limits.", i, i + 1);
            return false;
        }

        // check the interpolated path for collisions, as the interpolator may
        // take a slightly different
        bool collision = false;
        for (const auto& point : ipath) {
            double dist;
            if (!cc->isStateValid(point, false, false, dist)) {
                collision = true;
                break;
            }
        }

        if (collision) {
            ROS_ERROR("Interpolated path collides. Resorting to original waypoints");
            path.push_back(end);
            continue;
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
    }
    ROS_INFO("Original path length: %d   Interpolated path length: %d", int(traj.size()), int(traj_out.size()));
    return true;
}

} // namespace manip
} // namespace sbpl
