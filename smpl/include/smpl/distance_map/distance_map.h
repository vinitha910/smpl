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

#include "detail/distance_map_base.h"

namespace sbpl {

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
///
/// The base class DistanceMapBase completes implements functionality
/// independent of the distance function that is used to update cells. It is
/// not intended to be used directly. DistanceMap implements functionality that
/// is dependent on the distance function used.
///
/// The class passed through as the template parameter specifies the distance
/// function used to compute the distance updates from neighboring cells by
/// implementing a member function with the following signature:
///
///     int distance(const Cell& s, const Cell& n);
///
/// that returns the new distance for cell s, being updated from adjacent cell
/// n. If the distance function is made private (and it likely should be, since
/// the Cell struct is private to DistanceMapBase), the derived class must
/// declare DistanceMap as a friend.
template <typename Derived>
class DistanceMap : public DistanceMapBase
{
public:

    DistanceMap(
        double origin_x, double origin_y, double origin_z,
        double size_x, double size_y, double size_z,
        double resolution,
        double max_dist);

    void addPointsToMap(const std::vector<Eigen::Vector3d>& points);
    void removePointsFromMap(const std::vector<Eigen::Vector3d>& points);
    void updatePointsInMap(
        const std::vector<Eigen::Vector3d>& old_points,
        const std::vector<Eigen::Vector3d>& new_points);

    void reset();

    friend Derived;

private:

    int distance(const Cell& n, const Cell& s);

    void lower(Cell* s);
    void raise(Cell* s);
    void waveout(Cell* n);
    void propagate();

    void lowerBounded(Cell* s);
    void propagateRemovals();
    void propagateBorder();
};

/// A wrapper around DistanceMap<Derived> to provide interoperability with the
/// DistanceField interface in MoveIt!.
template <typename DistanceMap>
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

#include "detail/distance_map.hpp"

#endif
