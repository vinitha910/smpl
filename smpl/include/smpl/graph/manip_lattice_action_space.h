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

#ifndef SMPL_MANIP_LATTICE_ACTION_SPACE_H
#define SMPL_MANIP_LATTICE_ACTION_SPACE_H

// standard includes
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// system includes
#include <boost/algorithm/string.hpp>

// project includes
#include <smpl/forward.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/graph/action_space.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/robot_planning_space_observer.h>
#include <smpl/graph/manip_lattice.h>

namespace sbpl {
namespace motion {

class RobotPlanningSpace;

SBPL_CLASS_FORWARD(RobotPlanningSpace);
SBPL_CLASS_FORWARD(ManipLatticeActionSpace);

class ManipLatticeActionSpace : public ActionSpace
{
public:

    using const_iterator = std::vector<MotionPrimitive>::const_iterator;

    bool init(ManipLattice* space);

    bool load(const std::string& action_filename);

    void addMotionPrim(
        const std::vector<double>& mprim,
        bool short_dist_mprim,
        bool add_converse = true);

    void clear();

    const_iterator begin() const { return m_mprims.begin(); }
    const_iterator end() const { return m_mprims.end(); }

    int longDistCount() const;
    int shortDistCount() const;

    bool useAmp(MotionPrimitive::Type type) const;
    bool useMultipleIkSolutions() const;
    bool useLongAndShortPrims() const;
    double ampThresh(MotionPrimitive::Type type) const;

    void useAmp(MotionPrimitive::Type type, bool enable);
    void useMultipleIkSolutions(bool enable);
    void useLongAndShortPrims(bool enable);
    void ampThresh(MotionPrimitive::Type type, double thresh);

    /// \name Required Public Functions from ActionSpace
    ///@{
    bool apply(const RobotState& parent, std::vector<Action>& actions) override;
    ///@}

    /// \name Reimplemented Public Functions from RobotPlanningSpaceObserver
    ///@{
    virtual void updateStart(const RobotState& start) override;
    virtual void updateGoal(const GoalConstraint& goal) override;
    ///@}

protected:

    std::vector<MotionPrimitive> m_mprims;

    ForwardKinematicsInterface* m_fk_iface = nullptr;
    InverseKinematicsInterface* m_ik_iface = nullptr;

    bool m_mprim_enabled[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];
    double m_mprim_thresh[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];

    bool m_use_multiple_ik_solutions        = false;
    bool m_use_long_and_short_dist_mprims   = false;

    bool applyMotionPrimitive(
        const RobotState& state,
        const MotionPrimitive& mp,
        Action& action);

    bool computeIkAction(
        const RobotState& state,
        const std::vector<double>& goal,
        double dist_to_goal,
        ik_option::IkOption option,
        std::vector<Action>& actions);

    virtual bool getAction(
        const RobotState& parent,
        double goal_dist,
        double start_dist,
        const MotionPrimitive& mp,
        std::vector<Action>& actions);

    bool mprimActive(
        double start_dist,
        double goal_dist,
        MotionPrimitive::Type type) const;
};

} // namespace motion
} // namespace sbpl

#endif

