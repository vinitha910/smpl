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

#ifndef SMPL_DISTANCE_MAP_H
#define SMPL_DISTANCE_MAP_H

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

static const int NUM_DIRECTIONS = 2 * 27;
static const int NON_BORDER_NEIGHBOR_LIST_SIZE = 460;
static const int BORDER_NEIGHBOR_LIST_SIZE = 316;
static const int NEIGHBOR_LIST_SIZE =
        NON_BORDER_NEIGHBOR_LIST_SIZE + BORDER_NEIGHBOR_LIST_SIZE;

/// An unsigned distance transform implementation that computes distance values
/// incrementally up to a maximum threshold. The distance of each cell is
/// determined as the closer of the distance from its nearest obstacle cell and
/// the distance to the nearest border cell. Distances for cells outside the
/// specified bounding volume will be reported as 0. Border cells are defined
/// as imaginary cells that are adjacent to valid in-bounds cells but are not
/// themselves within the bounding volume.
///
/// The basis for this class can be found in 'Sebastian Scherer, David Ferguson,
/// and Sanjiv Singh, "Efficient C-Space and Cost Function Updates in 3D for
/// Unmanned Aerial Vehicles," Proceedings International Conference on Robotics
/// and Automation, May, 2009.', albeit with some implementation differences.
/// Notably, it is preferred to compute the distance updates for simultaneous
/// obstacle insertion and removal in a two-phase fashion rather than
/// interleaving the updates for the insertion and removal of obstacles. While
/// the latter should complete in fewer propagation iterations, it is not
/// usually worth the additional overhead to maintain the priority queue
/// correctly in this domain.
class DistanceMap
{
public:

    DistanceMap(
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

    void addPointsToMap(const std::vector<Eigen::Vector3d>& points);
    void removePointsFromMap(const std::vector<Eigen::Vector3d>& points);
    void updatePointsInMap(
        const std::vector<Eigen::Vector3d>& old_points,
        const std::vector<Eigen::Vector3d>& new_points);

    void reset();

private:

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

    typedef std::vector<Cell*> bucket_type;
    typedef std::vector<bucket_type> bucket_list;
    bucket_list m_open;

    std::vector<Cell*> m_rem_stack;

    void updateVertex(Cell* c);

    int distance(const Cell& n, const Cell& s) const;
    int distanceEuclidSqrd(const Cell& n, const Cell& s) const;
    int distanceEuclidSqrdConservative(const Cell& n, const Cell& s) const;
    int distanceQuasiEuclid(const Cell& n, const Cell& s) const;

    void lower(Cell* s);
    void raise(Cell* s);
    void waveout(Cell* n);
    void propagate();

    void lowerBounded(Cell* s);
    void propagateBorder();
};

/// A wrapper around DistanceMap to provide interoperability with the
/// DistanceField interface in MoveIt!.
class DistanceMapMoveIt : public distance_field::DistanceField
{
public:

    DistanceMapMoveIt(
        double origin_x, double origin_y, double origin_z,
        double size_x, double size_y, double size_z,
        double resolution,
        double max_dist);

    void addPointsToField(const EigenSTL::vector_Vector3d& points) override;

    void removePointsFromField(const EigenSTL::vector_Vector3d& points) override;

    void updatePointsInField(
        const EigenSTL::vector_Vector3d& old_points,
        const EigenSTL::vector_Vector3d& new_points) override;

    void reset() override;

    double getDistance(double x, double y, double z) const override;

    double getDistance(int x, int y, int z) const override;

    bool isCellValid(int x, int y, int z) const override;

    int getXNumCells() const override;

    int getYNumCells() const override;

    int getZNumCells() const override;

    bool gridToWorld(
        int x, int y, int z,
        double& world_x, double& world_y, double& world_z) const override;

    bool worldToGrid(
        double world_x, double world_y, double world_z,
        int& x, int& y, int& z) const override;

    bool writeToStream(std::ostream& stream) const override;

    bool readFromStream(std::istream& stream) override;

    double getUninitializedDistance() const override;

private:

    DistanceMap m_dm;
};

} // namespace sbpl

#endif
