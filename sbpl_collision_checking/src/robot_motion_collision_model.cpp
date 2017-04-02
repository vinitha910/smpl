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

#include <ros/console.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <smpl/angles.h>

namespace sbpl {
namespace collision {

RobotMotionCollisionModel::RobotMotionCollisionModel(
    const RobotCollisionModel* rcm)
:
    m_rcm(rcm),
    m_motion_centers(),
    m_motion_radii()
{
    ROS_DEBUG("Compute motion spheres");

    // queue of joints to be processed
    std::vector<int> q_joint(rcm->jointCount(), -1);
    int q_head = 0;
    int q_tail = 0;

    // number of child joints processed for determining topological ordering
    std::vector<int> p_joint(rcm->jointCount(), 0);

    // initialize the queue with the parent joint indices of each leaf link
    for (int lidx = 0; lidx < rcm->linkCount(); ++lidx) {
        if (rcm->linkChildJointIndices(lidx).empty()) {
            const int jidx = rcm->linkParentJointIndex(lidx);
            q_joint[q_tail++] = jidx;
            ROS_DEBUG("Insert parent joint %d of link %s", jidx, rcm->linkName(lidx).c_str());
        }
    }

    // M(n), n = 1...num_joints
    std::vector<Eigen::Vector3d> motion_centers(rcm->jointCount());
    std::vector<double> motion_radii(rcm->jointCount(), 0.0);

    // the sample spheres (MR spheres of child links + R of current link) used
    // to construct the (motion) M sphere for each joint
    std::vector<std::vector<Eigen::Vector3d>> sample_spheres(rcm->jointCount());

    // the sample sphere radii
    std::vector<double> sample_radii(rcm->jointCount(), 0.0);

    while (q_head != q_tail) {
        int jidx = q_joint[q_head++];

        ROS_DEBUG("Compute motion sphere for joint %d (link %s)", jidx, rcm->linkName(rcm->jointChildLinkIndex(jidx)).c_str());

        // construct MR(n) = R(n) + M_samples(n+1)

        // M_samples(n+1) for each child joint, transformed into the parent
        // joint frame
        std::vector<Eigen::Vector3d> joint_frame_sample_centers;

        // radius of each sample sphere, generally different for each child link
        // TODO: compress this
        std::vector<double> joint_frame_sample_radii;

        // R(n)
        const int clidx = rcm->jointChildLinkIndex(jidx);
        if (rcm->hasSpheresModel(clidx)) {
            const CollisionSpheresModel& spheres = rcm->spheresModel(rcm->linkSpheresModelIndex(clidx));
            const CollisionSphereModel* root = spheres.spheres.root();

            if (!root) {
                ROS_WARN("Root is null?");
            }

            joint_frame_sample_centers.push_back(root->center);
            joint_frame_sample_radii.push_back(root->radius);

            ROS_DEBUG("  R(%d) = { center: (%0.3f, %0.3f, %0.3f), radius: %0.3f }", jidx, root->center.x(), root->center.y(), root->center.z(), root->radius);
        }

        // M_samples(n+1)
        for (int cjidx : rcm->linkChildJointIndices(clidx)){
            if (sample_radii[cjidx] != 0.0) {
                // transform motion samples from child joint into parent joint frame
                const Eigen::Affine3d& T_joint_child = rcm->jointOrigin(cjidx);

                size_t child_mr_samples = 0;
                for (const Eigen::Vector3d& sphere_pos : sample_spheres[cjidx]) {
                    joint_frame_sample_centers.push_back(T_joint_child * sphere_pos);
                    joint_frame_sample_radii.push_back(sample_radii[cjidx]);
                    ++child_mr_samples;
                }
                ROS_DEBUG("  samples from MR(%d): %zu", cjidx, child_mr_samples);
            } else {
                ROS_DEBUG("  skip null motion sphere samples from child joint %d", cjidx);
            }
        }

        // MR(n)
        Eigen::Vector3d mr_center = Eigen::Vector3d::Zero();
        double mr_radius = 0.0;
        if (!joint_frame_sample_centers.empty()) {
            for (size_t i = 0; i < joint_frame_sample_centers.size(); ++i) {
                mr_center += joint_frame_sample_centers[i];
            }
            mr_center /= joint_frame_sample_centers.size();

            for (size_t i = 0; i < joint_frame_sample_centers.size(); ++i) {
                double radius = (joint_frame_sample_centers[i] - mr_center).norm() + joint_frame_sample_radii[i];
                mr_radius = std::max(mr_radius, radius);
            }
        }


        ROS_DEBUG("  MR(%d) = { center: (%0.3f, %0.3f, %0.3f), radius: %0.3f }", jidx, mr_center.x(), mr_center.y(), mr_center.z(), mr_radius);

        // M(n) = sample joint variable values, update MR, and compute M
        std::vector<Eigen::Vector3d> sample_motion_sphere_centers;
        if (mr_radius != 0.0) {
            double res = 2.0 * M_PI / 180.0; // desired resolution

            // sample MR(n) positions by uniform sampling the joint variable values
            if (rcm->jointType(jidx) == JointType::REVOLUTE) {
                int vidx = rcm->jointVarIndexFirst(jidx);
                double span = rcm->jointVarMaxPosition(vidx) - rcm->jointVarMinPosition(vidx);
                int sample_count = (int)std::round(span / res) + 1;
                ROS_DEBUG("  sample revolute joint [%0.3f, %0.3f] with %d samples", rcm->jointVarMinPosition(vidx), rcm->jointVarMaxPosition(vidx), sample_count);
                res = span / (sample_count - 1);
                for (int i = 0; i < sample_count; ++i) {
                    // interpolate between min and max position
                    double alpha = (double)i / (double)(sample_count - 1);
                    double val = (1.0 - alpha) * rcm->jointVarMinPosition(vidx) +
                            alpha * rcm->jointVarMaxPosition(vidx);

                    // construct rotation about joint axis at this variable position
                    const Eigen::Affine3d T_joint_link(
                            Eigen::AngleAxisd(val, rcm->jointAxis(jidx)));

                    sample_motion_sphere_centers.push_back(T_joint_link * mr_center);
                }
            } else if (rcm->jointType(jidx) == JointType::CONTINUOUS) {
                int sample_count = (int)std::round(2.0 * M_PI / res);
                ROS_DEBUG("  sample continuous joint with %d samples", sample_count);
                res = 2.0 * M_PI / sample_count;
                for (int i = 0; i < sample_count; ++i) {
                    double val = i * res;

                    int vidx = rcm->jointVarIndexFirst(jidx);

                    const Eigen::Affine3d T_joint_link(
                            Eigen::AngleAxisd(val, rcm->jointAxis(jidx)));

                    sample_motion_sphere_centers.push_back(T_joint_link * mr_center);
                }
            } else if (rcm->jointType(jidx) == JointType::PRISMATIC) {
                int vidx = rcm->jointVarIndexFirst(jidx);
                double span = rcm->jointVarMaxPosition(vidx) -
                        rcm->jointVarMinPosition(vidx);
                int sample_count = (int)std::round(span / res) + 1;
                ROS_DEBUG("  sample prismatic joint with %d samples", sample_count);
                res = span / (sample_count - 1);
                for (int i = 0; i < sample_count; ++i) {
                    // interpolate between min and max position
                    double alpha = (double)i / (double)(sample_count - 1);
                    double val = (1.0 - alpha) * rcm->jointVarMinPosition(vidx) +
                            alpha * rcm->jointVarMaxPosition(vidx);

                    // construct rotation about joint axis at this variable position
                    const Eigen::Affine3d T_joint_link(
                            Eigen::Translation3d(val * rcm->jointAxis(jidx)));

                    sample_motion_sphere_centers.push_back(T_joint_link * mr_center);
                }
            } else if (rcm->jointType(jidx) == JointType::FLOATING) {
                ROS_ERROR("TODO: Cannot sample floating joint transform");
            } else if (rcm->jointType(jidx) == JointType::PLANAR) {
                ROS_ERROR("TODO: Cannot sample planar joint transform");
            } else if (rcm->jointType(jidx) == JointType::FIXED) {
                ROS_DEBUG("  sample fixed joint with 1 sample");
                const Eigen::Affine3d T_joint_link(rcm->jointOrigin(jidx));
                sample_motion_sphere_centers.push_back(T_joint_link * mr_center);
            }
        }

        ROS_DEBUG("  MR(%d) samples: %zu", jidx, sample_motion_sphere_centers.size());

        // store MR(n) samples and MR(n) radius for constructing MR(n-1)
        sample_spheres[jidx] = sample_motion_sphere_centers;
        sample_radii[jidx] = mr_radius;

        // M(n)
        Eigen::Vector3d m_center(Eigen::Vector3d::Zero());
        double m_radius = 0.0;
        ROS_DEBUG("  Compute M(%d) from %zu samples", jidx, sample_motion_sphere_centers.size());
        if (!sample_motion_sphere_centers.empty()) {
            for (const Eigen::Vector3d& center : sample_motion_sphere_centers) {
                m_center += center;
            }
            m_center /= sample_motion_sphere_centers.size();
            for (const Eigen::Vector3d& center : sample_motion_sphere_centers) {
                m_radius = std::max(m_radius, (center - m_center).norm() + mr_radius);
            }
        }

        motion_centers[jidx] = m_center;
        motion_radii[jidx] = m_radius;
        ROS_DEBUG("  M(%d) = { center: (%0.3f, %0.3f, %0.3f), radius: %0.3f }", jidx, m_center.x(), m_center.y(), m_center.z(), m_radius);

        // update the position of the spheres on the current link
        // update the position of the bounding sphere of the child link

        // construct the motion spheres for the parents in topological fashion
        int plidx = rcm->jointParentLinkIndex(jidx);
        int pjidx = rcm->linkParentJointIndex(plidx);
        if (pjidx) {
            ++p_joint[pjidx];

            if (p_joint[pjidx] == rcm->linkChildJointIndices(plidx).size()) {
                q_joint[q_tail++] = pjidx;
            }
        }
    }

    m_motion_centers = std::move(motion_centers);
    m_motion_radii = std::move(motion_radii);
}

double RobotMotionCollisionModel::getMaxSphereMotion(
    const motion::RobotState& start,
    const motion::RobotState& finish) const
{
    assert(start.size() == m_rcm->jointVarCount());
    assert(finish.size() == m_rcm->jointVarCount());

    double motion = 0.0;
    for (size_t jidx; jidx < m_rcm->jointCount(); ++jidx) {
        size_t fvidx = m_rcm->jointVarIndexFirst(jidx);

        double dist = 0.0;
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::CONTINUOUS:
            dist = angles::shortest_angle_dist(finish[fvidx], start[fvidx]);
            motion += (m_motion_centers[jidx].norm() + m_motion_radii[jidx]) * dist;
            break;
        case JointType::REVOLUTE:
            dist = std::fabs(finish[fvidx] - start[fvidx]);
            motion += (m_motion_centers[jidx].norm() + m_motion_radii[jidx]) * dist;
            break;
        case JointType::PRISMATIC:
            dist = std::fabs(finish[fvidx] - start[fvidx]);
            motion += dist;
            break;
        case JointType::PLANAR: {
            const double dx = finish[fvidx + 0] - start[fvidx + 0];
            const double dy = finish[fvidx + 1] - start[fvidx + 1];
            const double dth = angles::shortest_angle_dist(
                    finish[fvidx + 2], start[fvidx + 2]);
            dist = std::sqrt(dx * dx + dy * dy);
            // TODO: HACK! hallucinated motion sphere radius to force waypoints
            motion += dist + dth * 1.0;
        }   break;
        case JointType::FLOATING: {
            const double dx = finish[fvidx + 0] - start[fvidx + 0];
            const double dy = finish[fvidx + 1] - start[fvidx + 1];
            const double dz = finish[fvidx + 2] - start[fvidx + 2];
            dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            motion += dist;
        }   break;
        }
    }

    return motion;
}

double RobotMotionCollisionModel::getMaxSphereMotion(
    const motion::RobotState& diff) const
{
    assert(diff.size() == m_rcm->jointVarCount());

    double motion = 0.0;
    for (size_t jidx = 0; jidx < m_rcm->jointVarCount(); ++jidx) {
        const int fvidx = m_rcm->jointVarIndexFirst(jidx);

        double dist = 0.0;
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::CONTINUOUS:
        case JointType::REVOLUTE:
            dist = std::fabs(diff[fvidx]);
            motion += (m_motion_centers[jidx].norm() + m_motion_radii[jidx]) * dist;
            break;
        case JointType::PRISMATIC:
            dist = std::fabs(diff[fvidx]);
            motion += dist;
            break;
        case JointType::PLANAR: {
            const double dx = diff[fvidx + 0];
            const double dy = diff[fvidx + 1];
            const double dth = diff[fvidx + 2];
            dist = std::sqrt(dx * dx + dy * dy);
            // TODO: HACK! hallucinated motion sphere radius to force waypoints
            motion += dist + dth * 1.0;
        }   break;
        case JointType::FLOATING: {
            const double dx = diff[fvidx + 0];
            const double dy = diff[fvidx + 1];
            const double dz = diff[fvidx + 2];
            dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            motion += dist;
        }   break;
        }
    }

    return motion;
}

/// Return an upper bound on the distance any sphere might travel given the
/// motion of a subset of joints.
double RobotMotionCollisionModel::getMaxSphereMotion(
    const motion::RobotState& start,
    const motion::RobotState& finish,
    const std::vector<int>& variables) const
{
    assert(start.size() == finish.size());
    assert(start.size() == variables.size());

    double motion = 0.0;
    for (size_t i = 0; i < start.size(); ++i) {
        const int vidx = variables[i];
        const int jidx = m_rcm->jointVarJointIndex(vidx);

        double dist = 0.0;
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::CONTINUOUS:
            dist = angles::shortest_angle_dist(finish[i], start[i]);
            motion += (m_motion_centers[jidx].norm() + m_motion_radii[jidx]) * dist;
            break;
        case JointType::REVOLUTE:
            dist = std::fabs(finish[i] - start[i]);
            motion += (m_motion_centers[jidx].norm() + m_motion_radii[jidx]) * dist;
            break;
        case JointType::PRISMATIC:
            dist = std::fabs(finish[i] - start[i]);
            motion += dist;
            break;
        case JointType::PLANAR:
        case JointType::FLOATING:
            break;
        }
    }

    return motion;
}

double RobotMotionCollisionModel::getMaxSphereMotion(
    const motion::RobotState& diff,
    const std::vector<int>& variables) const
{
    assert(diff.size() == variables.size());

    double motion = 0.0;
    for (size_t i = 0; i < diff.size(); ++i) {
        const int vidx = variables[i];
        const int jidx = m_rcm->jointVarJointIndex(vidx);

        double dist = 0.0;
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::CONTINUOUS:
        case JointType::REVOLUTE:
            dist = std::fabs(diff[i]);
            motion += (m_motion_centers[jidx].norm() + m_motion_radii[jidx]) * dist;
            break;
        case JointType::PRISMATIC:
            dist = std::fabs(diff[i]);
            motion += dist;
            break;
        case JointType::PLANAR:
        case JointType::FLOATING:
            break;
        }
    }

    return motion;
}

} // namespace collision
} // namespace sbpl
