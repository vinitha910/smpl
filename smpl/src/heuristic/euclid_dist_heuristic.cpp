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

#include <smpl/heuristic/euclid_dist_heuristic.h>

// standard includes
#include <math.h>

// project includes
#include <smpl/angles.h>

namespace sbpl {
namespace motion {

static inline
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

EuclidDistHeuristic::EuclidDistHeuristic(
    const RobotPlanningSpacePtr& pspace,
    const OccupancyGrid* grid)
:
    RobotHeuristic(pspace, grid)
{
    m_point_ext = pspace->getExtension<PointProjectionExtension>();
    if (m_point_ext) {
        ROS_INFO("Got Point Projection Extension!");
    }
    m_pose_ext = pspace->getExtension<PoseProjectionExtension>();
    if (m_pose_ext) {
        ROS_INFO("Got Pose Projection Extension!");
    }
    if (!m_pose_ext && !m_point_ext) {
        ROS_WARN("EuclidDistHeuristic recommends PointProjectionExtension or PoseProjectionExtension");
    }
}

double EuclidDistHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    const std::vector<double>& goal_pose = planningSpace()->goal().pose;
    return EuclideanDistance(x, y, z, goal_pose[0], goal_pose[1], goal_pose[2]);
}

double EuclidDistHeuristic::getMetricStartDistance(double x, double y, double z)
{
    // TODO: implement
    return 0.0;
}

Extension* EuclidDistHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int EuclidDistHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }

    if (m_pose_ext) {
        Eigen::Affine3d p;
        if (!m_pose_ext->projectToPose(state_id, p)) {
            return 0;
        }

        const std::vector<double>& goal_pose = planningSpace()->goal().pose;
        const Eigen::Vector3d gp(goal_pose[0], goal_pose[1], goal_pose[2]);

        assert(goal_pose.size() >= 6);
        Eigen::Quaterniond qgoal(
                Eigen::AngleAxisd(goal_pose[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(goal_pose[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(goal_pose[3], Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond qstate(p.rotation());

        if (qstate.dot(qgoal) < 0.0) {
            qgoal = Eigen::Quaterniond(-qgoal.w(), -qgoal.x(), -qgoal.y(), -qgoal.z());
        }

        double dp = (p.translation() - gp).norm();
        double dr = angles::normalize_angle(2.0 * acos(qstate.dot(qgoal)));
        dr = std::fabs(dr);

        double Y, P, R;
        angles::get_euler_zyx(p.rotation(), Y, P, R);
        ROS_DEBUG("h(%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f) = dp: %0.3f + dr: %0.3f", p.translation()[0], p.translation()[1], p.translation()[2], Y, P, R, dr, dp);

        return 50 * params()->cost_per_meter * (/*dp +*/ dr);
    } else if (m_point_ext) {
        Eigen::Vector3d p;
        if (!m_point_ext->projectToPoint(state_id, p)) {
            return 0;
        }

        const std::vector<double>& goal_pose = planningSpace()->goal().pose;
        Eigen::Vector3d gp(goal_pose[0], goal_pose[1], goal_pose[2]);

        int h = 50 * params()->cost_per_meter * (gp - p).norm();
        ROS_DEBUG_NAMED(params()->heuristic_log, "h(%d) = %d", state_id, h);
        return h;
    } else {
        return 0;
    }
}

int EuclidDistHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int EuclidDistHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (!m_point_ext) {
        return 0;
    }

    Eigen::Vector3d fp, tp;
    if (!m_point_ext->projectToPoint(from_id, fp) ||
        !m_point_ext->projectToPoint(to_id, tp))
    {
        return 0;
    }

    int h;
    h = (tp - fp).norm();
    h *= params()->cost_per_meter;
    h *= 500;
    h /= grid()->getResolution();
    return h;
}

} // namespace motion
} // namespace sbpl
