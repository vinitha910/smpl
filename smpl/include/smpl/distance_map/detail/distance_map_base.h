////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#ifndef SMPL_DISTANCE_MAP_BASE_H
#define SMPL_DISTANCE_MAP_BASE_H

// standard includes
#include <array>
#include <utility>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <moveit/distance_field/distance_field.h>

// project includes
#include <smpl/grid.h>

#define SMPL_DMAP_RETURN_CHANGED_CELLS 0

namespace sbpl {

#define SMPL_DMAP_RETURN_CHANGED_CELLS 0

static const int NUM_DIRECTIONS = 2 * 27;
static const int NON_BORDER_NEIGHBOR_LIST_SIZE = 460;
static const int BORDER_NEIGHBOR_LIST_SIZE = 316;
static const int NEIGHBOR_LIST_SIZE =
        NON_BORDER_NEIGHBOR_LIST_SIZE + BORDER_NEIGHBOR_LIST_SIZE;

class DistanceMapBase
{
public:

    DistanceMapBase(
        double origin_x, double origin_y, double origin_z,
        double size_x, double size_y, double size_z,
        double resolution,
        double max_dist);

    double sizeX() const;
    double sizeY() const;
    double sizeZ() const;
    double originX() const;
    double originY() const;
    double originZ() const;
    double resolution() const;

    double maxDistance() const;

    int numCellsX() const;
    int numCellsY() const;
    int numCellsZ() const;

    void gridToWorld(
        int x, int y, int z,
        double& world_x, double& world_y, double& world_z) const;

    void worldToGrid(
        double world_x, double world_y, double world_z,
        int& x, int& y, int& z) const;

    bool isCellValid(int x, int y, int z) const;

    double getDistance(double x, double y, double z) const;
    double getDistance(int x, int y, int z) const;

protected:

    struct Cell
    {
        int x;
        int y;
        int z;

        int dist;
        int dist_new;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
        int dist_old;
#endif
        Cell* obs;
        int bucket;
        int dir;

        int pos;
    };

    Grid3<Cell> m_cells;

    // origin of grid in world coordinates
    double m_origin_x;
    double m_origin_y;
    double m_origin_z;

    // size of the grid in world units
    double m_size_x;
    double m_size_y;
    double m_size_z;

    // resolution of the grid in world units
    double m_res;

    // max propagation distance in world units
    double m_max_dist;
    double m_inv_res;

    // max propagation distance in cells
    int m_dmax_int;
    int m_dmax_sqrd_int;

    int m_bucket;

    int m_no_update_dir;

    std::array<Eigen::Vector3i, 27> m_neighbors;
    std::array<int, NEIGHBOR_LIST_SIZE> m_indices;
    std::array<std::pair<int, int>, NUM_DIRECTIONS> m_neighbor_ranges;
    std::array<int, NEIGHBOR_LIST_SIZE> m_neighbor_offsets;
    std::array<int, NEIGHBOR_LIST_SIZE> m_neighbor_dirs;

    std::vector<double> m_sqrt_table;

    typedef std::vector<Cell*> bucket_type;
    typedef std::vector<bucket_type> bucket_list;
    bucket_list m_open;

    std::vector<Cell*> m_rem_stack;

    void updateVertex(Cell* c);
};

} // namespace sbpl

#endif
