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
#include <angles/angles.h>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>

namespace sbpl {
namespace manip {

enum ShortcutType
{
    INVALID_SHORTCUT_TYPE = -1,
    JOINT_SPACE,
    EUCLID_SPACE,
    NUM_SHORTCUT_TYPES
};

std::string to_string(ShortcutType type);

class PlanningParams
{
public:

    // manipulation lattice parameters
    static const int DefaultCostMultiplier = 1000;
    static const int DefaultCostPerCell = 1;
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
    std::string planning_frame_;
    int num_joints_;
    std::vector<std::string> planning_joints_;
    std::vector<int> coord_vals_;
    std::vector<double> coord_delta_;
    ///@}

    /// \name Actions
    ///@{
    bool use_multiple_ik_solutions_;
    ///@}

    /// \name Costs
    ///@{
    int cost_multiplier_;           ///< uniform cost of actions
    int cost_per_cell_;             ///< uniform cost of cells in heuristic
    int cost_per_meter_;            ///< euclidean distance heuristic cost
    int cost_per_second_;
    double time_per_cell_;
    double max_mprim_offset_;
    ///@}

    /// \name Heuristic
    ///@{
    bool use_bfs_heuristic_;
    double planning_link_sphere_radius_;
    ///@}

    /// \name Search
    ///@{
    std::string planner_name_;
    double epsilon_;
    double allowed_time_;
    bool search_mode_; // true => stop after first solution
    ///@}

    /// \name Post-Processing
    ///@{
    bool shortcut_path_;
    bool interpolate_path_;
    double waypoint_time_;
    ShortcutType shortcut_type;
    ///@}

    /// \name Logging
    ///@{
    bool print_path_;
    bool verbose_;
    bool verbose_heuristics_;
    bool verbose_collisions_;

    std::string graph_log_;
    std::string heuristic_log_;
    std::string expands_log_;
    std::string rmodel_log_;
    std::string post_processing_log_;
    std::string solution_log_;
    ///@}
};

} // namespace manip
} // namespace sbpl

#endif

