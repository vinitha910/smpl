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

#ifndef sbpl_manip_types_h
#define sbpl_manip_types_h

// standard includes
#include <vector>

namespace sbpl {
namespace manip {

typedef std::vector<double> RobotState;

typedef std::vector<RobotState> Action;

enum GoalType
{
    INVALID_GOAL_TYPE = -1,
    XYZ_GOAL,
    XYZ_RPY_GOAL,
    JOINT_STATE_GOAL,
    NUMBER_OF_GOAL_TYPES
};

struct GoalConstraint
{
    // Relevant for joint state goals
    std::vector<double> angles;
    std::vector<double> angle_tolerances;

    // Relevant for workspace goals
    std::vector<double> pose;           // goal pose of the planning link as (x, y, z, R, P, Y)
    std::vector<double> tgt_off_pose;   // goal pose offset from planning link
    double xyz_offset[3];               // offset from the planning link
    double xyz_tolerance[3];            // (x, y, z) tolerance
    double rpy_tolerance[3];            // (R, P, Y) tolerance

    int xyz[3];                         // planning frame cell (x, y, z)

    GoalType type;                      // type of goal constraint
};

} // namespace manip
} // namespace sbpl

#endif
