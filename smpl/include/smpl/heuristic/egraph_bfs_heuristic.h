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

#ifndef SMPL_EGRAPH_BFS_HEURISTIC_H
#define SMPL_EGRAPH_BFS_HEURISTIC_H

// standard includes
#include <vector>

// project includes
#include <smpl/grid.h>
#include <smpl/intrusive_heap.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/heuristic/egraph_heuristic.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace sbpl {
namespace motion {

class DijkstraEgraphHeuristic3D :
    public RobotHeuristic,
    public ExperienceGraphHeuristicExtension
{
public:

    DijkstraEgraphHeuristic3D(
        const RobotPlanningSpacePtr& pspace,
        const OccupancyGrid* grid);

    visualization_msgs::MarkerArray getValuesVisualization();

    /// \name Required Public Functions from ExperienceGraphHeuristicExtension
    ///@{
    void getEquivalentStates(
        int state_id,
        std::vector<int>& ids) const override;

    void getShortcutSuccs(
        int state_id,
        std::vector<int>& shortcut_ids) override;
    ///@}

    /// \name Required Public Functions from RobotHeuristic
    ///@{
    double getMetricStartDistance(double x, double y, double z) override;
    double getMetricGoalDistance(double x, double y, double z) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Reimplemented Public Functions from RobotPlanningSpaceObserver
    ///@{
    void updateGoal(const GoalConstraint& goal) override;
    ///@}

    /// \name Required Public Functions from Heuristic
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override;
    ///@}

private:

    static const int Unknown = std::numeric_limits<int>::max() >> 1;
    static const int Wall = std::numeric_limits<int>::max();
    static const int Infinity = Unknown;

    struct Cell : public heap_element
    {
        int dist;

        Cell() = default;
        explicit Cell(int d) : heap_element(), dist(d) { }
    };

    Grid3<Cell> m_dist_grid;

    struct CellCompare
    {
        bool operator()(const Cell& a, const Cell& b) const {
            return a.dist < b.dist;
        }
    };

    double m_eg_eps;

    intrusive_heap<Cell, CellCompare> m_open;

    PointProjectionExtension* m_pp;
    ExperienceGraphExtension* m_eg;

    // map down-projected state cells to adjacent down-projected state cells
    struct Vector3iHash
    {
        typedef Eigen::Vector3i argument_type;
        typedef std::size_t result_type;

        result_type operator()(const argument_type& s) const;
    };

    hash_map<Eigen::Vector3i, std::vector<Eigen::Vector3i>, Vector3iHash> m_egraph_edges;

    void projectExperienceGraph();
    int getGoalHeuristic(const Eigen::Vector3i& dp);
};

} // namespace motion
} // namespace sbpl

#endif
