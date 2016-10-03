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

#include <sbpl_arm_planner/bfs_heuristic.h>

// system includes
#include <leatherman/viz.h>

// project includes
#include <sbpl_arm_planner/bfs3d/bfs3d.h>

namespace sbpl {
namespace manip {

BfsHeuristic::BfsHeuristic(ManipLattice* env, const OccupancyGrid* grid) :
    RobotHeuristic(env, grid),
    m_bfs()
{
    syncGridAndBfs();
}

BfsHeuristic::~BfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool BfsHeuristic::setGoal(const GoalConstraint& goal)
{
    int gx, gy, gz;
    grid()->worldToGrid(
            goal.tgt_off_pose[0], goal.tgt_off_pose[1], goal.tgt_off_pose[2],
            gx, gy, gz);

    ROS_DEBUG_NAMED(params()->heuristic_log_, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);

    if (!m_bfs->inBounds(gx, gy, gz)) {
        ROS_ERROR_NAMED(params()->heuristic_log_, "Heuristic goal is out of BFS bounds");
        return false;
    }

    m_bfs->run(gx, gy, gz);
    return true;
}

double BfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    int start_id = planningSpace()->getStartStateID();
    ManipLattice* manip_lattice = (ManipLattice*)planningSpace();
    const ManipLatticeState* start_state = manip_lattice->getHashEntry(start_id);
    if (start_state) {
        // compute the manhattan distance to the start cell
        std::vector<double> pose;
        if (!manip_lattice->computePlanningFrameFK(start_state->state, pose)) {
            ROS_ERROR_NAMED(params()->heuristic_log_, "Failed to compute forward kinematics for the planning frame");
            return 0.0;
        }

        int sx, sy, sz;
        grid()->worldToGrid(pose[0], pose[1], pose[2], sx, sy, sz);

        int gx, gy, gz;
        grid()->worldToGrid(x, y, z, gx, gy, gz);

        const int dx = sx - gx;
        const int dy = sy - gy;
        const int dz = sz - gz;
        return grid()->getResolution() * (abs(dx) + abs(dy) + abs(dz));
    }
    else {
        return 0.0;
    }
}

double BfsHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->getResolution();
    }
    else {
        return (double)m_bfs->getDistance(gx, gy, gz) * grid()->getResolution();
    }
}

int BfsHeuristic::GetGoalHeuristic(int state_id)
{
    ManipLattice* manip_lattice = (ManipLattice*)planningSpace();
    const ManipLatticeState* state = manip_lattice->getHashEntry(state_id);
    if (state) {
        return getBfsCostToGoal(
                *m_bfs, state->xyz[0], state->xyz[1], state->xyz[2]);
    }
    else {
        return 0;
    }
}

int BfsHeuristic::GetStartHeuristic(int state_id)
{
    ROS_WARN_ONCE("BfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int BfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        ROS_WARN_ONCE("BfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

visualization_msgs::MarkerArray BfsHeuristic::getWallsVisualization() const
{
    std::vector<geometry_msgs::Point> points;
    int dimX, dimY, dimZ;
    grid()->getGridSize(dimX, dimY, dimZ);
    for (int z = 0; z < dimZ; z++) {
        for (int y = 0; y < dimY; y++) {
            for (int x = 0; x < dimX; x++) {
                if (m_bfs->isWall(x, y, z)) {
                    geometry_msgs::Point p;
                    grid()->gridToWorld(x, y, z, p.x, p.y, p.z);
                    points.push_back(p);
                }
            }
        }
    }

    ROS_DEBUG_NAMED(params()->heuristic_log_, "BFS Visualization contains %zu points", points.size());

    std_msgs::ColorRGBA color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    visualization_msgs::Marker cubes_marker = viz::getCubesMarker(
            points,
            grid()->getResolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls",
            0);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(std::move(cubes_marker));
    return ma;
}

visualization_msgs::MarkerArray BfsHeuristic::getValuesVisualization() const
{
    visualization_msgs::MarkerArray ma;
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;
    int dimX, dimY, dimZ;
    grid()->getGridSize(dimX, dimY, dimZ);
    for (int z = 0; z < dimZ; ++z) {
        for (int y = 0; y < dimY; ++y) {
            for (int x = 0; x < dimX; ++x) {
                // skip cells without valid distances from the start
                if (m_bfs->isWall(x, y, z) || m_bfs->isUndiscovered(x, y, z)) {
                    continue;
                }

                int d = m_bfs->getDistance(x, y, z);
                grid()->gridToWorld(
                        x, y, z, p.position.x, p.position.y, p.position.z);
                double hue = d / 30.0 * 300;
                ma.markers.push_back(viz::getTextMarker(
                        p,
                        std::to_string(d),
                        0.009,
                        hue,
                        grid()->getReferenceFrame(),
                        "bfs_values",
                        ma.markers.size()));
            }
        }
    }
    return ma;
}

void BfsHeuristic::syncGridAndBfs()
{
    int xc, yc, zc;
    grid()->getGridSize(xc, yc, zc);
//    ROS_DEBUG_NAMED(params()->heuristic_log_, "Initializing BFS of size %d x %d x %d = %d", xc, yc, zc, xc * yc * zc);
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int z = 0; z < zc; ++z) {
        for (int y = 0; y < yc; ++y) {
            for (int x = 0; x < xc; ++x) {
                const double radius = params()->planning_link_sphere_radius_;
                if (grid()->getDistance(x, y, z) <= radius) {
                    m_bfs->setWall(x, y, z);
                    ++wall_count;
                }
            }
        }
    }

    ROS_DEBUG_NAMED(params()->heuristic_log_, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int BfsHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return INT_MAX;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return INT_MAX;
    }
    else {
        return params()->cost_per_cell_ * bfs.getDistance(x, y, z);
    }
}

} // namespace manip
} // namespace sbpl
