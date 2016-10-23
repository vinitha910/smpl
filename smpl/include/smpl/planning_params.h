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
namespace motion {

enum ShortcutType
{
    INVALID_SHORTCUT_TYPE = -1,
    JOINT_SPACE,
    JOINT_POSITION_VELOCITY_SPACE,
    EUCLID_SPACE,
    NUM_SHORTCUT_TYPES
};

std::string to_string(ShortcutType type);

#define PARAMETER_EXPERIMENTAL 0
#if PARAMETER_EXPERIMENTAL
struct PlanningParameter
{
    typedef std::vector<PlanningParameter>              ParameterArray;
    typedef std::map<std::string, PlanningParameter>    ParameterMap;
    typedef ParameterMap::iterator                      iterator;

    enum Type
    {
        TypeInvalid,
        TypeBoolean,
        TypeInt,
        TypeDouble,
        TypeString,
        TypeArray,
        TypeMap,
    }

    Type type() const { return m_type; }

    PlanningParameter() : m_type(TypeInvalid), m_value() { }
    PlanningParameter(bool value) : m_type(TypeBoolean) { m_value.b = value; }
    PlanningParameter(int value) : m_type(TypeInt) { m_value.i = value; }
    PlanningParameter(double value) : m_type(TypeDouble) { m_value.d = value; }
    PlanningParameter(const std::string& value) : m_type(TypeString) {
        m_value.s = new std::string(value);
    }
    PlanningParameter(const char* value) : m_type(TypeString) {
        m_value.s = new std::string(value);
    }

    ~PlanningParameter();

    operator bool&();
    operator int&();
    operator double&();
    operator std::string&();
    operator std::map<std::string, PlanningParameter>&();
    operator std::vector<PlanningParameter>&();

    size_t size() const;

    PlanningParameter& operator[](size_t i);
    const PlanningParameter& operator[](size_t i) const;

    bool hasMember(const std::string& name) const;

    PlanningParameter& operator[](const std::string& key);
    const PlanningParameter& operator[](const std::string& key);

    iterator begin() { return m_value.m->begin(); }
    iterator end() { return m_value.m->end(); }

private:

    Type m_type;
    union
    {
        bool            b;
        int             i;
        double          d;
        std::string*    s;
        ParameterArray* a;
        ParameterMap*   m;
    } m_value;
};
#endif

class PlanningParams
{
public:

    // manipulation lattice parameters
    static const int DefaultCostMultiplier = 1000;
    static const int DefaultCostPerCell = 100;
    static const int DefaultCostPerMeter = 1000;
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

    void printParams(const std::string& stream) const;

    /// \name Environment
    ///@{
    std::string planning_frame;
    std::vector<int> coord_vals;
    std::vector<double> coord_delta;
    ///@}

    /// \name Actions
    ///@{
    std::string action_filename;
    bool use_multiple_ik_solutions;

    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_thresh;
    double rpy_snap_thresh;
    double xyzrpy_snap_thresh;
    double short_dist_mprims_thresh;
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

#if PARAMETER_EXPERIMENTAL
inline
PlanningParameter::~PlanningParameter()
{
    switch (type) {
    case Type::TypeString:
        delete m_value.s;
        break;
    case Type::TypeArray:
        delete m_value.a;
        break;
    case Type::TypeMap:
        delete m_value.m;
        break;
    }
}
#endif

} // namespace motion
} // namespace sbpl

#endif

