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

#ifndef _ROBOT_MODEL_
#define _ROBOT_MODEL_

#include <string>
#include <vector>

#include <angles/angles.h>
#include <kdl/frames.hpp>

namespace sbpl {
namespace manip {

namespace ik_option {

enum IkOption
{
    UNRESTRICTED = 0,
    RESTRICT_XYZ_JOINTS = 1
};

std::ostream& operator<<(std::ostream& o, IkOption option);
std::string to_string(IkOption option);

} // namespace ik_option

class RobotModel
{
public:

    RobotModel();

    virtual ~RobotModel() { };

    /// \name Configuration
    /// @{

    void setPlanningJoints(const std::vector<std::string>& joints);
    const std::vector<std::string>& getPlanningJoints() const;

    virtual double minVarLimit(int jidx) const = 0;
    virtual double maxVarLimit(int jidx) const = 0;
    virtual bool   hasVarLimit(int jidx) const = 0;

    void setPlanningLink(const std::string& name);
    const std::string& getPlanningLink() const;

    void setPlanningFrame(const std::string& name);
    const std::string& getPlanningFrame() const;

    const std::string& getKinematicsFrame() const;

    ///@}

    /// \brief Joint Limits
    virtual bool checkJointLimits(
        const std::vector<double>& angles,
        bool verbose = false);

    /// \name Forward Kinematics
    ///@{

    /// \brief Compute the forward kinematics pose of a link in the robot model.
    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        KDL::Frame& f);

    /// \brief Compute the forward kinematics pose of a link in the robot model.
    virtual bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        std::vector<double>& pose);

    /// \brief Compute forward kinematics of the planning link.
    ///
    /// The output pose, stored in \p pose, should be of the format
    /// { x, y, z, R, P, Y } of the planning link
    ///
    /// \return true if forward kinematics were computed; false otherwise
    virtual bool computePlanningLinkFK(
        const std::vector<double>& angles,
        std::vector<double>& pose);

    ///@}

    /// \name Inverse Kinematics
    ///@{

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED);

    virtual bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<std::vector<double> >& solutions,
        ik_option::IkOption option = ik_option::UNRESTRICTED);

    virtual bool computeFastIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution);

    /// @}

    /// \name Debug Output
    /// @{

    virtual void printRobotModelInformation();

    void setLoggerName(const std::string& name);

    ///@}

    /// \brief Transform between Kinematics frame <-> Planning frame
    void setKinematicsToPlanningTransform(
        const KDL::Frame& f,
        const std::string& name);

protected:

    /** \brief frame that the planning is done in (i.e. map) */
    std::string planning_frame_;

    /** \brief frame that the kinematics is computed in (i.e. robot base) */
    std::string kinematics_frame_;

    /** \brief the link that is being planned for (i.e. wrist) */
    std::string planning_link_;

    std::vector<std::string> planning_joints_;

    KDL::Frame T_kinematics_to_planning_;

    KDL::Frame T_planning_to_kinematics_;

    /** \brief ROS logger stream name */
    std::string logger_;
};

} // namespace manip
} // namespace sbpl

#endif
