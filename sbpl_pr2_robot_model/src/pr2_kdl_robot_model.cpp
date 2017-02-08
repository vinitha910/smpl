////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen
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

#include <sbpl_pr2_robot_model/pr2_kdl_robot_model.h>

// system includes
#include <kdl/tree.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/ros.h>
#include <smpl/angles.h>

using namespace std;

namespace sbpl {
namespace motion {

PR2KDLRobotModel::PR2KDLRobotModel() :
    KDLRobotModel(),
    pr2_ik_solver_(),
    rpy_solver_()
{
    forearm_roll_link_name_ = "r_forearm_roll_link";
    wrist_pitch_joint_name_ = "r_wrist_flex_joint";
    end_effector_link_name_ = "r_gripper_palm_link";
}

PR2KDLRobotModel::~PR2KDLRobotModel()
{
}

bool PR2KDLRobotModel::init(
    const std::string& robot_description,
    const std::vector<std::string>& planning_joints,
    const std::string& chain_root_link,
    const std::string& chain_tip_link,
    int free_angle)
{
    ROS_INFO("Initialize PR2 KDL Robot Model");

    if (!KDLRobotModel::init(
            robot_description,
            planning_joints,
            chain_root_link,
            chain_tip_link,
            free_angle))
    {
        return false;
    }

    // TODO: Take in as params instead.
    if (planning_joints[0].substr(0,1).compare("l") == 0) {
        forearm_roll_link_name_.replace(0,1,"l");
        wrist_pitch_joint_name_.replace(0,1,"l");
        end_effector_link_name_.replace(0,1,"l");
    }

    // PR2 Specific IK Solver
    pr2_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(
            *urdf_, chain_root_name_, chain_tip_name_, 0.02, 2));
    if (!pr2_ik_solver_->active_) {
        ROS_ERROR("The pr2 IK solver is NOT active. Exiting.");
        initialized_ = false;
        return false;
    }

    // initialize rpy solver
    double wrist_min_limit, wrist_max_limit;
    bool wrist_continuous;
    double wrist_vel_limit, wrist_eff_limit;
    if (!getJointLimits(
            wrist_pitch_joint_name_,
            wrist_min_limit,
            wrist_max_limit,
            wrist_continuous,
            wrist_vel_limit,
            wrist_eff_limit))
    {
        initialized_ = false;
        return false;
    }

    rpy_solver_.reset(new RPYSolver(wrist_min_limit, wrist_max_limit));

    initialized_ = true;
    return true;
}

bool PR2KDLRobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution,
    ik_option::IkOption option)
{
    //pose: {x,y,z,r,p,y} or {x,y,z,qx,qy,qz,qw}
    KDL::Frame frame_des;
    frame_des.p.x(pose[0]);
    frame_des.p.y(pose[1]);
    frame_des.p.z(pose[2]);

    if (pose.size() == 6) {
        // RPY
        frame_des.M = KDL::Rotation::RPY(pose[3], pose[4], pose[5]);
    } else {
        // quaternion
        frame_des.M = KDL::Rotation::Quaternion(pose[3], pose[4], pose[5], pose[6]);
    }

    // transform into kinematics frame
    frame_des = T_planning_to_kinematics_ * frame_des;

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        // must be normalized for CartToJntSearch
        jnt_pos_in_(i) = angles::normalize_angle(start[i]);
    }

    solution.resize(start.size());

    // choose solver
    if (option == ik_option::RESTRICT_XYZ) {
        std::vector<double> rpy(3, 0);
        std::vector<double> fpose(6, 0);
        std::vector<double> epose(6, 0);
        frame_des.M.GetRPY(rpy[0], rpy[1], rpy[2]);
        const std::vector<double> rpy2(rpy);

        // get pose of forearm link
        if (!computeFK(start, forearm_roll_link_name_, fpose)) {
            ROS_ERROR("[rm] computeFK failed on forearm pose.");
            return false;
        }

        // get pose of end-effector link
        if (!computeFK(start, end_effector_link_name_, epose)) {
            ROS_ERROR("[rm] computeFK failed on end_eff pose.");
            return false;
        }

        return rpy_solver_->computeRPYOnly(rpy2, start, fpose, epose, 1, solution);
    } else {
        const double timeout = 0.2;
        const double consistency_limit = 2.0 * M_PI;
        if (pr2_ik_solver_->CartToJntSearch(
                jnt_pos_in_,
                frame_des,
                jnt_pos_out_,
                timeout,
                consistency_limit) < 0)
        {
            return false;
        }

        for (size_t i = 0; i < solution.size(); ++i) {
            solution[i] = jnt_pos_out_(i);
        }
    }

    return true;
}

bool PR2KDLRobotModel::computeFastIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution)
{
    //pose: {x,y,z,r,p,y} or {x,y,z,qx,qy,qz,qw}
    KDL::Frame frame_des;
    frame_des.p.x(pose[0]);
    frame_des.p.y(pose[1]);
    frame_des.p.z(pose[2]);

    if (pose.size() == 6) {
        // RPY
        frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
    } else {
        // quaternion
        frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);
    }

    // transform into kinematics frame
    frame_des = T_planning_to_kinematics_ * frame_des;

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        // must be normalized for CartToJntSearch
        jnt_pos_in_(i) = angles::normalize_angle(start[i]);
    }

    if (pr2_ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) < 0) {
        return false;
    }

    solution.resize(start.size());
    for (size_t i = 0; i < solution.size(); ++i) {
        solution[i] = jnt_pos_out_(i);
    }

    return true;
}

} // namespace motion
} // namespace sbpl
