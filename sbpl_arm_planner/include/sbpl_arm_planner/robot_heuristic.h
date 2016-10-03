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

#ifndef sbpl_manip_robot_heuristic_h
#define sbpl_manip_robot_heuristic_h

// standard includes
#include <stdint.h>
#include <limits>

// system includes
#include <sbpl/heuristics/heuristic.h>

// project includes
#include <sbpl_arm_planner/forward.h>
#include <sbpl_arm_planner/robot_planning_space_observer.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/planning_params.h>
#include <sbpl_arm_planner/robot_planning_space.h>

namespace sbpl {
namespace manip {

class RobotHeuristic : public Heuristic, public RobotPlanningSpaceObserver
{
public:

    static const int Infinity = std::numeric_limits<int16_t>::max();

    RobotHeuristic(
        RobotPlanningSpace* pspace,
        const OccupancyGrid* grid);

    virtual ~RobotHeuristic();

    /// \brief Return the heuristic distance of the planning link to the start.
    ///
    /// This distance is used by the manipulation lattice to determine whether
    /// to activate context-aware actions.
    virtual double getMetricStartDistance(double x, double y, double z) = 0;

    /// \brief Return the heuristic distance of the planning link to the goal.
    ///
    /// This distance is used by the manipulation lattice to determine whether
    /// to activate context-aware actions.
    virtual double getMetricGoalDistance(double x, double y, double z) = 0;

    virtual bool setGoal(const GoalConstraint& goal);

    RobotPlanningSpace* planningSpace() { return m_pspace; }
    const RobotPlanningSpace* planningSpace() const { return m_pspace; }

    const PlanningParams* params() const { return m_pspace->params(); }

    const OccupancyGrid* grid() const { return m_grid; }

    /// \name Restate Required Public Functions from Heuristic
    ///@{
    virtual int GetGoalHeuristic(int state_id) = 0;
    virtual int GetStartHeuristic(int state_id) = 0;
    virtual int GetFromToHeuristic(int from_id, int to_id) = 0;
    ///@}

private:

    RobotPlanningSpace* m_pspace;
    const OccupancyGrid* m_grid;
};

} // namespace manip
} // namespace sbpl

#endif
