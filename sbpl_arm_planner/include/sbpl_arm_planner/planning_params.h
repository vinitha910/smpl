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

#ifndef sbpl_manip_planning_params_h
#define sbpl_manip_planning_params_h

// standard includes
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// system includes
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>

namespace sbpl {
namespace manip {

enum ShortcutType
{
    INVALID_SHORTCUT_TYPE = -1,
    JOINT_SPACE,
    JOINT_POSITION_VELOCITY_SPACE,
    EUCLID_SPACE,
    NUM_SHORTCUT_TYPES
};

std::string to_string(ShortcutType type);

class PlanningParams
{
public:

    // manipulation lattice parameters
    static const int DefaultCostMultiplier = 1000;
    static const int DefaultCostPerCell = 100;
    static const int DefaultCostPerMeter = 50;
    static const int DefaultCostPerSecond = DefaultCostMultiplier;
    static constexpr double DefaultTimePerCell = 0.05;
    static constexpr double DefaultMaxMprimOffset = 0.0;
    static const bool DefaultUseMultipleIkSolutions = false;

    // heuristic parameters
    static const bool DefaultUseBfsHeuristic = true;
    static constexpr double DefaultPlanningLinkSphereRadius = 0.08;

    // search parameters
    static const bool DefaultSearchMode = false;
    static constexpr double DefaultAllowedTime = 10.0;
    static constexpr double DefaultEpsilon = 10.0;

    // profiling parameters
    static constexpr double DefaultWaypointTime = 0.35;

    // post processing parameters
    static const bool DefaultShortcutPath = false;
    static const bool DefaultInterpolatePath = false;
    static const ShortcutType DefaultShortcutType = ShortcutType::JOINT_SPACE;

    // logging parameters
    static const std::string DefaultRobotModelLog;
    static const std::string DefaultGraphLog;
    static const std::string DefaultHeuristicLog;
    static const std::string DefaultExpandsLog;
    static const std::string DefaultPostProcessingLog;
    static const std::string DefaultSolutionLog;

    // TODO: visualization parameters

    PlanningParams();

    bool init(const std::string& ns = "~");

    void printParams(const std::string& stream) const;

    /// \name Environment
    ///@{
    std::string planning_frame;
    int num_joints;
    std::vector<std::string> planning_joints;
    std::vector<int> coord_vals;
    std::vector<double> coord_delta;
    ///@}

    /// \name Actions
    ///@{
    std::string action_file;
    bool use_multiple_ik_solutions;
    ///@}

    /// \name Costs
    ///@{
    int cost_multiplier;           ///< uniform cost of actions
    int cost_per_cell;             ///< uniform cost of cells in heuristic
    int cost_per_meter;            ///< euclidean distance heuristic cost
    int cost_per_second;
    double time_per_cell;
    double max_mprim_offset;
    ///@}

    /// \name Heuristic
    ///@{
    bool use_bfs_heuristic;
    double planning_link_sphere_radius;
    ///@}

    /// \name Search
    ///@{
    std::string planner_name;
    double epsilon;
    double allowed_time;
    bool search_mode; // true => stop after first solution
    ///@}

    /// \name Post-Processing
    ///@{
    bool shortcut_path;
    bool interpolate_path;
    double waypoint_time;
    ShortcutType shortcut_type;
    ///@}

    /// \name Logging
    ///@{
    bool print_path;
    bool verbose;
    bool verbose_heuristics;
    bool verbose_collisions;

    std::string graph_log;
    std::string heuristic_log;
    std::string expands_log;
    std::string robot_log;
    std::string post_processing_log;
    std::string solution_log;
    ///@}
};

} // namespace manip
} // namespace sbpl

#endif

