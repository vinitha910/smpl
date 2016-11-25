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

#include <smpl/graph/manip_lattice_egraph.h>

#include <fstream>

#include <boost/filesystem.hpp>
#include <leatherman/print.h>
#include <smpl/csv_parser.h>
#include <smpl/graph/manip_lattice_action_space.h>

namespace sbpl {
namespace motion {

auto ManipLatticeEgraph::RobotCoordHash::operator()(const argument_type& s) const ->
    result_type
{
    std::size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.begin(), s.end()));
    return seed;
}

ManipLatticeEgraph::ManipLatticeEgraph(
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params,
    OccupancyGrid* grid)
:
    ManipLattice(robot, checker, params, grid),
    m_coord_to_id(),
    m_egraph(),
    m_egraph_state_ids()
{
}

// Strategies for building experience graphs from continuous path data:
//
// 1. The path data comes in the form as a set of continuous states
//
// discretize every state in the set of continuous states
// remove any duplicate discrete states
// for each pair of discrete points
//     if there exists an action that transitions between the two points
//         add the transition path to the experience graph
//
// 2. The path data comes in the form as a set of paths (sequences of continuous
//    states). The intention is to reuse the input paths verbatim with little
//    discretization error.
//
// for each path, p
//     dp0 = discretize(p0)
//     dpf = discretize(p0)
//     edge <- [ p0 ]
//     for each point pi in path \ p0
//         dpf = discretize(pi)
//         if dpf != dp0
//             add edge (dp0, dpf, edge)
//         else
//             edge = edge + [ pi ]
//
// TODO: Considerations:
//
// (1) What should be done about about context-specific actions, such as IK
// motions to the current goal?
// (2) Should effort be made to explicitly connect two states with the same
// discrete coordinate but differing continuous coordinates?
// (2a) One method might be to allow discontinuous jumps, denoted by either the
// the discretization or an explicit tolerance
// (3a) Intersections between paths can be tested and a new state at the
// intersection can be created
bool ManipLatticeEgraph::loadExperienceGraph(const std::string& path)
{
    ROS_INFO("Load Experience Graph at %s", path.c_str());

    boost::filesystem::path p(path);
    if (!boost::filesystem::is_directory(p)) {
        ROS_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        const std::string& filepath = dit->path().generic_string();
        std::vector<RobotState> egraph_states;
        if (!parseExperienceGraphFile(filepath, egraph_states)) {
            continue;
        }

        if (egraph_states.empty()) {
            continue;
        }

        ROS_INFO("Create hash entries for experience graph states");

        const RobotState& pp = egraph_states.front(); // previous robot state
        RobotCoord pdp(robot()->jointVariableCount()); // previous robot coord
        stateToCoord(egraph_states.front(), pdp);
        ExperienceGraph::node_id pid = m_egraph.insert_node(pp);
        m_coord_to_id[pdp] = pid;
        ManipLatticeState* entry = createHashEntry(pdp, pp);
        m_egraph_state_ids.resize(pid + 1, -1);
        m_egraph_state_ids[pid] = entry->stateID;

        std::vector<RobotState> edge_data;
        for (size_t i = 1; i < egraph_states.size(); ++i) {
            const RobotState& p = egraph_states[i];
            RobotCoord dp(robot()->jointVariableCount());
            stateToCoord(p, dp);
            if (dp != pdp) {
                ManipLatticeState* entry = createHashEntry(dp, p);
                // found a new discrete state along the path
                ExperienceGraph::node_id id = m_egraph.insert_node(p);
                m_egraph_state_ids.resize(id + 1, -1);
                m_egraph_state_ids[id] = entry->stateID;
                m_coord_to_id[dp] = id;
                m_egraph.insert_edge(pid, id, edge_data);
                pdp = dp;
                pid = id;
                edge_data.clear();
            } else {
                // gather intermediate robot states
                edge_data.push_back(p);
            }
        }
    }

    ROS_INFO("Experience graph contains %zu nodes and %zu edges", m_egraph.num_nodes(), m_egraph.num_edges());
    return true;
}

void ManipLatticeEgraph::getShortcutSuccPath(
    int state_id,
    std::vector<int>& succs,
    std::vector<int>& costs)
{
}

void ManipLatticeEgraph::getSnapSuccs(
    int state_id,
    std::vector<int>& succs,
    std::vector<int>& costs)
{
}

bool ManipLatticeEgraph::isOnExperienceGraph(int state)
{
    return false;
}

const ExperienceGraph* ManipLatticeEgraph::getExperienceGraph() const
{
    return &m_egraph;
}

ExperienceGraph* ManipLatticeEgraph::getExperienceGraph()
{
    return &m_egraph;
}

int ManipLatticeEgraph::getStateID(ExperienceGraph::node_id n) const
{
    if (n >= m_egraph_state_ids.size()) {
        return -1;
    } else {
        return m_egraph_state_ids[n];
    }
}

Extension* ManipLatticeEgraph::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphExtension>()) {
        return this;
    } else {
        return ManipLattice::getExtension(class_code);
    }
}

bool ManipLatticeEgraph::parseExperienceGraphFile(
    const std::string& filepath,
    std::vector<RobotState>& egraph_states) const
{
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        return false;
    }

    CSVParser parser;
    const bool with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        ROS_ERROR("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    ROS_INFO("Parsed experience graph file");
    ROS_INFO("  Has Header: %s", parser.hasHeader() ? "true" : "false");
    ROS_INFO("  %zu records", parser.recordCount());
    ROS_INFO("  %zu fields", parser.fieldCount());

    const size_t jvar_count = robot()->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count) {
        ROS_ERROR("Parsed experience graph contains insufficient number of joint variables");
        return false;
    }

    egraph_states.reserve(parser.totalFieldCount());
    for (size_t i = 0; i < parser.recordCount(); ++i) {
        RobotState state(jvar_count);
        for (size_t j = 0; j < parser.fieldCount(); ++j) {
            try {
                state[j] = std::stod(parser.fieldAt(i, j));
            } catch (const std::invalid_argument& ex) {
                ROS_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            } catch (const std::out_of_range& ex) {
                ROS_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            }
        }
        egraph_states.push_back(std::move(state));
    }

    ROS_INFO("Read %zu states from experience graph file", egraph_states.size());
    return true;
}

/// An attempt to construct the discrete experience graph by discretizing all
/// input continuous states and connecting them via edges available in the
/// canonical action set. This turns out to not work very well since the points
/// are not often able to be connected by the limited action set. It also
/// necessitates imposing restrictions on the action set, since context-specific
/// actions don't make sense before an actual planning request.
void ManipLatticeEgraph::rasterizeExperienceGraph()
{
//    std::vector<RobotCoord> egraph_coords;
//    for (const RobotState& state : egraph_states) {
//        RobotCoord coord(robot()->jointVariableCount());
//        stateToCoord(state, coord);
//        egraph_coords.push_back(std::move(coord));
//    }
//
//    auto it = std::unique(egraph_coords.begin(), egraph_coords.end());
//    egraph_coords.erase(it, egraph_coords.end());
//
//    ROS_INFO("Experience contains %zu discrete states", egraph_coords.size());
//    for (const RobotCoord& coord : egraph_coords) {
//        ROS_INFO("  %s", to_string(coord).c_str());
//    }
//
//    ROS_INFO("Insert states into experience graph and map coords to experience graph nodes");
//
//    // insert all coords into egraph and initialize coord -> egraph node mapping
//    RobotState state(robot()->jointVariableCount());
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        coordToState(*it, state);
//        m_coord_to_id[*it] = m_egraph.insert_node(state);
//    }
//
//    ROS_INFO("Insert experience graph edges into experience graph");
//
//    int edge_count = 0;
//    ManipLatticeActionSpace* aspace =
//            dynamic_cast<ManipLatticeActionSpace*>(actionSpace().get());
//    if (!aspace) {
//        ROS_ERROR("ManipLatticeEgraph requires action space to be a ManipLatticeActionSpace");
//        return false;
//    }
//
//    // save action space configuration
//    bool mprim_enabled_state[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        mprim_enabled_state[i] = aspace->useAmp((MotionPrimitive::Type)i);
//    }
//    bool use_long_and_short_mprims = aspace->useLongAndShortPrims();
//
//    // disable context-specific motion primitives
//    aspace->useAmp(MotionPrimitive::SHORT_DISTANCE, true);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_RPY, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, false);
//
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        const ExperienceGraph::node_id n = m_coord_to_id[*it];
//        RobotState source(robot()->jointVariableCount());
//        coordToState(*it, source);
//        std::vector<Action> actions;
//        aspace->apply(source, actions);
//        ROS_INFO("%zu actions from egraph state", actions.size());
//        for (const Action& action : actions) {
//            RobotCoord last(robot()->jointVariableCount());
//            stateToCoord(action.back(), last);
//            ROS_INFO("Check for experience graph edge %s -> %s", to_string(*it).c_str(), to_string(last).c_str());
//            auto iit = m_coord_to_id.find(last);
//            if (iit != m_coord_to_id.end() && !m_egraph.edge(n, iit->second)) {
//                m_egraph.insert_edge(n, iit->second);
//                ++edge_count;
//            }
//        }
//    }
//    ROS_INFO("Experience graph contains %d edges", edge_count);
//
//    // restore action space configuration
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        aspace->useAmp((MotionPrimitive::Type)i, mprim_enabled_state[i]);
//    }
//    aspace->useLongAndShortPrims(use_long_and_short_mprims);
}

} // namespace motion
} // namespace sbpl
