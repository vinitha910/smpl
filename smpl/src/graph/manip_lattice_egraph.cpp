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

#include <smpl/csv_parser.h>

namespace sbpl {
namespace motion {

auto ManipLatticeEgraph::ivec_hash::operator()(const argument_type& s) const ->
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
    m_egraph()
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

    std::ifstream fin(path);
    if (!fin.is_open()) {
        ROS_ERROR("Failed to open '%s' for reading", path.c_str());
        return false;
    }

    CSVParser parser;
    const bool with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        ROS_ERROR("Failed to parse experience graph file '%s'", path.c_str());
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

    std::vector<RobotState> egraph_states;
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

    ROS_INFO("Create hash entries for experience graph states");
    std::vector<RobotCoord> egraph_coords;
    for (const RobotState& state : egraph_states) {
        RobotCoord coord(robot()->jointVariableCount());
        stateToCoord(state, coord);
        egraph_coords.push_back(std::move(coord));

        createHashEntry(coord, state);
    }

    auto it = std::unique(egraph_coords.begin(), egraph_coords.end());
    egraph_coords.erase(it, egraph_coords.end());

    ROS_INFO("Experience contains %zu discrete states", egraph_coords.size());

    ROS_INFO("Insert states into experience graph and map coords to experience graph nodes");

    // insert all coords into egraph and initialize coord -> egraph node mapping
    RobotState state(robot()->jointVariableCount());
    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
        coordToState(*it, state);
        m_coord_to_id[*it] = m_egraph.insert_node(state);
    }

    ROS_INFO("Insert experience graph edges into experience graph");

    int edge_count = 0;
    ActionSpacePtr aspace = actionSpace();
    if (!aspace) {
        ROS_WARN("No action space available to rasterize experience graph");
        return true;
    }

    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
        const ExperienceGraph::node_id n = m_coord_to_id[*it];
        RobotState source(robot()->jointVariableCount());
        coordToState(*it, source);
        std::vector<Action> actions;
        aspace->apply(source, actions);
        for (const Action& action : actions) {
            RobotCoord last(robot()->jointVariableCount());
            stateToCoord(action.back(), last);
            auto iit = m_coord_to_id.find(last);
            if (iit != m_coord_to_id.end() && !m_egraph.edge(n, iit->second)) {
                m_egraph.insert_edge(n, iit->second);
                ++edge_count;
            }
        }
    }
    ROS_INFO("Experience graph contains %d edges", edge_count);

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

} // namespace motion
} // namespace sbpl
