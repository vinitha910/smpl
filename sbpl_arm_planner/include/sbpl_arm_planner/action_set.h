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

#ifndef sbpl_arm_planner_ActionSet_h
#define sbpl_arm_planner_ActionSet_h

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <iterator>
#include <ros/ros.h>
#include <angles/angles.h>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <sbpl_manipulation_components/motion_primitive.h>

namespace sbpl_arm_planner {

class EnvironmentROBARM3D;

class ActionSet;
typedef std::shared_ptr<ActionSet> ActionSetPtr;
typedef std::shared_ptr<ActionSet> ActionSetConstPtr;

class ActionSet
{
public:

    typedef std::vector<MotionPrimitive>::const_iterator const_iterator;

    static bool Load(const std::string& action_file, ActionSet& action_set);

    static const double DefaultAmpThreshold;

    ActionSet();

    bool init(EnvironmentROBARM3D* env, bool use_multiple_ik_solutions = false);

    /// \brief Add a long or short distance motion primitive to the action set
    /// \param mprim The angle delta for each joint, in radians
    /// \param short_dist true = short distance; false = long distance
    /// \param add_converse Whether to add the negative of this motion primitive
    ///     to the action set
    void addMotionPrim(
        const std::vector<double>& mprim,
        bool short_dist_mprim,
        bool add_converse = true);

    /// \brief Remove all motion primitives and disable all adaptive motions
    void clear();

    const_iterator begin() const { return mp_.begin(); }
    const_iterator end() const { return mp_.end(); }

    int longDistCount() const;
    int shortDistCount() const;

    bool useAmp(MotionPrimitive::Type type) const;
    bool useMultipleIkSolutions() const;
    double ampThresh(MotionPrimitive::Type type) const;

    void useAmp(MotionPrimitive::Type type, bool enable);
    void useMultipleIkSolutions(bool enable);
    void ampThresh(MotionPrimitive::Type type, double thresh);

    bool getActionSet(const RobotState& parent, std::vector<Action>& actions);

    void print() const;

protected:

    std::vector<MotionPrimitive> mp_;

    // NOTE: these arrays will NOT include entries for LONG_DISTANCE motion
    // primitives
    bool m_mprim_enabled[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];
    double m_mprim_thresh[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];

    bool use_multiple_ik_solutions_;

    EnvironmentROBARM3D *env_;

    bool applyMotionPrimitive(
        const RobotState &state,
        const MotionPrimitive &mp,
        Action &action);

    bool getAction(
        const RobotState& parent,
        double dist_to_goal,
        const MotionPrimitive& mp,
        std::vector<Action>& actions);

    bool mprimActive(double dist_to_goal, MotionPrimitive::Type type) const;
};

} // namespace sbpl_arm_planner

#endif

