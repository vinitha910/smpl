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

#include <smpl/ros/propagation_distance_field.h>

// standard includes
#include <cmath>

namespace sbpl {

PropagationDistanceField::PropagationDistanceField(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double res,
    double max_distance,
    bool propagate_negative_distances)
:
    DistanceMapInterface(origin_x, origin_y, origin_z, size_x, size_y, size_z, res),
    m_df(size_x, size_y, size_z,
            res,
            origin_x, origin_y, origin_z,
            max_distance,
            propagate_negative_distances),
    m_max_distance(max_distance),
    m_propagate_negative_distances(propagate_negative_distances),
    m_half_res(0.5 * res),
    m_error(std::sqrt(3.0) * res)
{
}

PropagationDistanceField::PropagationDistanceField(
    const octomap::OcTree& octree,
    const octomap::point3d& bbx_min,
    const octomap::point3d& bbx_max,
    double max_distance,
    bool propagate_negative_distances)
:
    DistanceMapInterface(
            bbx_min.x(), bbx_min.y(), bbx_min.z(),
            bbx_max.x() - bbx_min.x(), bbx_max.y() - bbx_min.y(), bbx_max.z() - bbx_min.z(),
            octree.getResolution()),
    m_df(octree, bbx_min, bbx_max, max_distance, propagate_negative_distances),
    m_max_distance(max_distance),
    m_propagate_negative_distances(propagate_negative_distances),
    m_error()
{
    m_half_res = 0.5 * m_df.getResolution();
    m_error = std::sqrt(3.0) * m_df.getResolution();
}

PropagationDistanceField::PropagationDistanceField(
    std::istream& stream,
    double max_distance,
    bool propagate_negative_distances)
:
    DistanceMapInterface(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    m_df(stream, max_distance, propagate_negative_distances),
    m_max_distance(max_distance),
    m_propagate_negative_distances(propagate_negative_distances),
    m_error()
{
    m_origin_x = m_df.getOriginX();
    m_origin_y = m_df.getOriginY();
    m_origin_z = m_df.getOriginZ();
    m_size_x = m_df.getSizeX();
    m_size_y = m_df.getSizeY();
    m_size_z = m_df.getSizeZ();
    m_res = m_df.getResolution();
    m_half_res = 0.5 * m_df.getResolution();
    m_error = std::sqrt(3.0) * m_df.getResolution();
}

DistanceMapInterface* PropagationDistanceField::clone() const
{
    PropagationDistanceField* df = new PropagationDistanceField(
            originX(), originY(), originZ(),
            sizeX(), sizeY(), sizeZ(),
            resolution(),
            m_max_distance,
            m_propagate_negative_distances);
    std::vector<Eigen::Vector3d> points;
    for (int x = 0; x < numCellsX(); ++x) {
    for (int y = 0; y < numCellsY(); ++y) {
    for (int z = 0; z < numCellsZ(); ++z) {
        // TODO: find out obstacle distance value when negative distances exist
        if (m_df.getDistance(x, y, z) == 0.0) {
            double wx, wy, wz;
            m_df.gridToWorld(x, y, z, wx, wy, wz);
            points.emplace_back(wx, wy, wz);
        }
    }
    }
    }
    df->addPointsToMap(points);
    return df;
}

void PropagationDistanceField::addPointsToMap(
    const std::vector<Eigen::Vector3d>& points)
{
    EigenSTL::vector_Vector3d pts = toAlignedVector(points);
    m_df.addPointsToField(pts);
}

void PropagationDistanceField::removePointsFromMap(
    const std::vector<Eigen::Vector3d>& points)
{
    EigenSTL::vector_Vector3d pts = toAlignedVector(points);
    m_df.removePointsFromField(pts);
}

void PropagationDistanceField::updatePointsInMap(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    EigenSTL::vector_Vector3d old_pts = toAlignedVector(old_points);
    EigenSTL::vector_Vector3d new_pts = toAlignedVector(new_points);
    m_df.updatePointsInField(old_pts, new_pts);
}

void PropagationDistanceField::reset()
{
    m_df.reset();
}

double PropagationDistanceField::getUninitializedDistance() const
{
    return m_df.getUninitializedDistance();
}

int PropagationDistanceField::numCellsX() const
{
    return m_df.getXNumCells();
}

int PropagationDistanceField::numCellsY() const
{
    return m_df.getYNumCells();
}

int PropagationDistanceField::numCellsZ() const
{
    return m_df.getZNumCells();
}

double PropagationDistanceField::getMetricDistance(
    double x, double y, double z) const
{
    return m_df.getDistance(x, y, z);
}

double PropagationDistanceField::getCellDistance(int x, int y, int z) const
{
    return m_df.getDistance(x, y, z);
}

double PropagationDistanceField::getMetricSquaredDistance(
    double x, double y, double z) const
{
    Eigen::Vector3i gp;
    if (!m_df.worldToGrid(x, y, z, gp.x(), gp.y(), gp.z())) {
        return m_df.getResolution() * m_df.getMaximumDistanceSquared();
    }

    const double ox = m_df.getOriginX();
    const double oy = m_df.getOriginY();
    const double oz = m_df.getOriginZ();
    const double res = m_df.getResolution();

    // compute the squared distance from (x, y, z) to the nearest point on
    // the obstacle cell positioned at \p npos
    auto edge_dist = [&](const Eigen::Vector3i& npos)
    {
        // nearest obstacle cell -> nearest obstacle center
        double nx = ox + res * npos.x();
        double ny = oy + res * npos.y();
        double nz = oz + res * npos.z();
//        m_grid->gridToWorld(npos.x(), npos.y(), npos.z(), nx, ny, nz);

        // nearest obstacle center -> nearest obstacle corner
        if (gp.x() > npos.x()) {
            nx += m_half_res;
        } else if (gp.x() < npos.x()) {
            nx -= m_half_res;
        } else {
            nx = x;
        }

        if (gp.y() > npos.y()) {
            ny += m_half_res;
        } else if (gp.y() < npos.y()) {
            ny -= m_half_res;
        } else {
            ny = y;
        }

        if (gp.z() > npos.z()) {
            nz += m_half_res;
        } else if (gp.z() < npos.z()) {
            nz -= m_half_res;
        } else {
            nz = z;
        }

        const double dx = x - nx;
        const double dy = y - ny;
        const double dz = z - nz;

        return dx * dx + dy * dy + dz * dz;
    };

    const bool border =
            gp.x() == 0 | gp.y() == 0 | gp.z() == 0 |
            gp.x() == m_df.getXNumCells() - 1 |
            gp.y() == m_df.getYNumCells() - 1 |
            gp.z() == m_df.getZNumCells() - 1;

    // check the 8 nearest cells and take the minimum of the distances from
    // (x, y, z) to their nearest obstacles
    double min_d2 = res * m_df.getMaximumDistanceSquared();
    int nx_last = -1, ny_last = -1, nz_last = -1;


// for PropagationDistanceField with no negative propagation, NULL is returned
// if the cell's nearest neighbor is itself or if the propagation distance is
// 0. Both instances only occur if the cell is an obstacle. If the cell has an
// unknown nearest obstacle, it's nearest neighbor will point to an
// uninitialized cell at coordinates (-1, -1, -1).
#define UPDATE_DIST_NO_BOUNDS(_x, _y, _z) \
do { \
double dist; \
Eigen::Vector3i npos; \
if (!m_df.getNearestCell(_x, _y, _z, dist, npos)) { \
    if (npos.x() == nx_last & npos.y() == ny_last & npos.z() == nz_last) { \
        continue; \
    } \
    nx_last = npos.x(); ny_last = npos.y(); nz_last = npos.z(); \
    const double d2 = edge_dist(npos); \
    if (d2 < min_d2) { \
        min_d2 = d2; \
    } \
} else if (npos.x() < 0) { \
    if (nx_last < 0) { \
        continue; \
    } \
    nx_last = npos.x(); \
    dist -= m_error; \
    const double d2 = dist * dist; \
    if (d2 < min_d2) { \
        min_d2 = d2; \
    } \
} else { \
    if (npos.x() == nx_last & npos.y() == ny_last & npos.z() == nz_last) { \
        continue; \
    } \
    nx_last = npos.x(); ny_last = npos.y(); nz_last = npos.z();\
    const double d2 = edge_dist(npos); \
    if (d2 < min_d2) { \
        min_d2 = d2; \
    } \
} \
} while (0)

    // obstacle cell -> nearest distance to it \
    // unknown nearest obstacle cell -> conservative nearest distance \
    // known obstacle cell \

#define UPDATE_DIST_BOUNDS(x, y, z) \
do { \
if (m_df.isCellValid(x, y, z)) { \
    UPDATE_DIST_NO_BOUNDS(x, y, z); \
} \
} while (0)

    if (border) {
        Eigen::Vector3i gpp(gp);
        for (int dx = -1; dx <= 1; ++dx) {
            gp.x() = gp.x() + dx;
        for (int dy = -1; dy <= 1; ++dy) {
            gp.y() = gp.y() + dy;
        for (int dz = -1; dz <= 1; ++dz) {
            gp.z() = gp.z() + dz;
            UPDATE_DIST_BOUNDS(gpp.x(), gpp.y(), gpp.z());
        }
        }
        }
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() + -1, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() + -1, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() + -1, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() +  0, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() +  0, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() +  0, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() +  1, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() +  1, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() + -1, gp.y() +  1, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() + -1, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() + -1, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() + -1, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() +  0, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() +  0, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() +  0, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() +  1, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() +  1, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() +  0, gp.y() +  1, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() + -1, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() + -1, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() + -1, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() +  0, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() +  0, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() +  0, gp.z() +  1);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() +  1, gp.z() + -1);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() +  1, gp.z() +  0);
//            UPDATE_DIST_BOUNDS(gp.x() +  1, gp.y() +  1, gp.z() +  1);
    }
    else {
        Eigen::Vector3i gpp(gp);
        for (int dx = -1; dx <= 1; ++dx) {
            gpp.x() = gp.x() + dx;
        for (int dy = -1; dy <= 1; ++dy) {
            gpp.y() = gp.y() + dy;
        for (int dz = -1; dz <= 1; ++dz) {
            gpp.z() = gp.z() + dz;
            UPDATE_DIST_NO_BOUNDS(gpp.x(), gpp.y(), gpp.z());
        }
        }
        }
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() + -1, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() + -1, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() + -1, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() +  0, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() +  0, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() +  0, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() +  1, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() +  1, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() + -1, gp.y() +  1, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() + -1, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() + -1, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() + -1, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() +  0, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() +  0, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() +  0, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() +  1, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() +  1, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  0, gp.y() +  1, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() + -1, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() + -1, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() + -1, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() +  0, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() +  0, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() +  0, gp.z() +  1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() +  1, gp.z() + -1);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() +  1, gp.z() +  0);
//            UPDATE_DIST_NO_BOUNDS(gp.x() +  1, gp.y() +  1, gp.z() +  1);
    }

    return min_d2;
}

bool PropagationDistanceField::isCellValid(int x, int y, int z) const
{
    return m_df.isCellValid(x, y, z);
}

void PropagationDistanceField::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    (void)m_df.gridToWorld(x, y, z, world_x, world_y, world_z);
}

void PropagationDistanceField::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    (void)m_df.worldToGrid(world_x, world_y, world_z, x, y, z);
}

EigenSTL::vector_Vector3d PropagationDistanceField::toAlignedVector(
    const std::vector<Eigen::Vector3d>& vin) const
{
    EigenSTL::vector_Vector3d vout(vin.size());
    std::copy(vin.begin(), vin.end(), vout.begin());
    return vout;
}

} // namespace sbpl
