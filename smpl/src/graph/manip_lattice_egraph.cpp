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

bool ManipLatticeEgraph::loadExperienceGraph(const std::string& path)
{
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

    size_t jvar_count = robot()->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count) {
        ROS_ERROR("Parsed experience graph contains insufficient number of joint variables");
        return false;
    }

    std::vector<double> egraph_states;
    egraph_states.reserve(parser.totalFieldCount());
    for (size_t i = 0; i < parser.recordCount(); ++i) {
        for (size_t j = 0; j < parser.fieldCount(); ++j) {
            try {
                double var = std::stod(parser.fieldAt(i, j));
                egraph_states.push_back(var);
            } catch (const std::invalid_argument& ex) {
                ROS_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            } catch (const std::out_of_range& ex) {
                ROS_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            }
        }
    }

    ActionSpacePtr actions = actionSpace();
//    std::vector<std::vector<int>>

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
