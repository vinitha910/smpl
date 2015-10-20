////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Maxim Likhachev
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

#include <sbpl_manipulation_components/robot_model.h>

#include <ros/console.h>

namespace sbpl_arm_planner {

RobotModel::RobotModel()
{
    logger_ = "kinematic_model";
}

void RobotModel::setPlanningJoints(const std::vector<std::string>& joints)
{
    planning_joints_ = joints;
}

const std::vector<std::string>& RobotModel::getPlanningJoints() const
{
    return planning_joints_;
}

void RobotModel::setPlanningLink(const std::string& name)
{
    planning_link_ = name;
}

const std::string& RobotModel::getPlanningLink() const
{
    return planning_link_;
}

void RobotModel::setPlanningFrame(const std::string& name)
{
    planning_frame_ = name;
}

const std::string& RobotModel::getPlanningFrame() const
{
    return planning_frame_;
}

const std::string& RobotModel::getKinematicsFrame() const
{
    return kinematics_frame_;
}

bool RobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    KDL::Frame& f)
{
    ROS_ERROR("Function not filled in.");  
    return false;
}

bool RobotModel::computeFK(
    const std::vector<double>& angles,
    const std::string& name,
    std::vector<double>& pose)
{
    ROS_ERROR("Function not filled in.");  
    return false;
}

bool RobotModel::computePlanningLinkFK(
    const std::vector<double>& angles,
    std::vector<double>& pose)
{
    ROS_ERROR("Function not filled in.");  
    return false;
}

bool RobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double> &solution,
    int option)
{
    ROS_ERROR("Function not filled in."); 
    return false;
}

bool RobotModel::computeIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector< std::vector<double> >& solutions,
    int option)
{
    ROS_ERROR("Function not filled in."); 
    return false;
}

bool RobotModel::computeFastIK(
    const std::vector<double>& pose,
    const std::vector<double>& start,
    std::vector<double>& solution)
{
    ROS_ERROR("Function not filled in.");  
    return false;
}

void RobotModel::printRobotModelInformation()
{
    ROS_ERROR("Function not filled in.");  
}

void RobotModel::setLoggerName(const std::string& name)
{
    logger_ = name;
}

bool RobotModel::checkJointLimits(const std::vector<double>& angles)
{
    ROS_ERROR("Function not filled in.");  
    return false;
}

void RobotModel::setKinematicsToPlanningTransform(
    const KDL::Frame& f,
    const std::string& name)
{
    T_kinematics_to_planning_ = f;
    T_planning_to_kinematics_ = f.Inverse();
    planning_frame_ = name;
}

} // namespace sbpl_arm_planner
