////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/planning_params.h>

// system includes
#include <leatherman/utils.h>
#include <leatherman/print.h>

namespace sbpl {
namespace motion {

const std::string PlanningParams::DefaultRobotModelLog = "robot";
const std::string PlanningParams::DefaultGraphLog = "graph";
const std::string PlanningParams::DefaultHeuristicLog = "heuristic";
const std::string PlanningParams::DefaultExpandsLog = "expands";
const std::string PlanningParams::DefaultPostProcessingLog = "post_process";
const std::string PlanningParams::DefaultSolutionLog = "solution";

std::string to_string(ShortcutType type)
{
    switch (type) {
    case ShortcutType::INVALID_SHORTCUT_TYPE:
        return "INVALID_SHORTCUT_TYPE";
    case ShortcutType::JOINT_SPACE:
        return "JOINT_SPACE";
    case ShortcutType::JOINT_POSITION_VELOCITY_SPACE:
        return "JOINT_POSITION_VELOCITY_SPACE";
    case ShortcutType::EUCLID_SPACE:
        return "EUCLID_SPACE";
    default:
        return "UNRECOGNIZED_SHORTCUT_TYPE";
    }
}

PlanningParams::PlanningParams() :
    planning_frame(),

    action_filename(),
    use_multiple_ik_solutions(DefaultUseMultipleIkSolutions),
    use_xyz_snap_mprim(DefaultUseMotionPrimitiveSnapXYZ),
    use_rpy_snap_mprim(DefaultUseMotionPrimitiveSnapRPY),
    use_xyzrpy_snap_mprim(DefaultUseMotionPrimitiveSnapXYZRPY),
    use_short_dist_mprims(DefaultUseMotionPrimitiveShortDistance),
    xyz_snap_thresh(DefaultThreshSnapXYZ),
    rpy_snap_thresh(DefaultThreshSnapRPY),
    xyzrpy_snap_thresh(DefaultThreshSnapXYZRPY),
    short_dist_mprims_thresh(DefaultThreshShortDistance),

    cost_multiplier(DefaultCostMultiplier),
    cost_per_cell(DefaultCostPerCell),
    cost_per_meter(DefaultCostPerMeter),
    cost_per_second(DefaultCostPerSecond),
    time_per_cell(DefaultTimePerCell),
    max_mprim_offset(DefaultMaxMprimOffset),

    use_bfs_heuristic(DefaultUseBfsHeuristic),
    planning_link_sphere_radius(DefaultPlanningLinkSphereRadius),

    epsilon(DefaultEpsilon),
    allowed_time(DefaultAllowedTime),
    search_mode(DefaultSearchMode),

    shortcut_path(DefaultShortcutPath),
    interpolate_path(DefaultInterpolatePath),
    waypoint_time(DefaultWaypointTime),
    shortcut_type(DefaultShortcutType),

    print_path(true),
    verbose(false),
    verbose_heuristics(false),
    verbose_collisions(false),
    robot_log(DefaultRobotModelLog),
    graph_log(DefaultGraphLog),
    heuristic_log(DefaultHeuristicLog),
    expands_log(DefaultExpandsLog),
    post_processing_log(DefaultPostProcessingLog),
    solution_log(DefaultSolutionLog)
{
}

} // namespace motion
} // namespace sbpl
