////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl/occupancy_grid.h>

// standard includes
#include <memory>

// system includes
#include <ros/console.h>
#include <leatherman/viz.h>

// project includes
#include <smpl/ros/propagation_distance_field.h>

namespace sbpl {

/// \class OccupancyGrid
///
/// OccupancyGrid is a lightweight wrapper around DistanceMapInterface, with
/// some additional functionality.
///
/// The first additional feature is the presence of a reference frame id that is
/// attached to the distance map. The reference frame is used solely for
/// preparing visualizations and does there are no active transformations
/// managed by this class.
///
/// The second additional feature is cell reference counting. Each cell may
/// count the number of times it has been inserted into the distance map. Only
/// when a cell has been removed the same number of times it has been inserted,
/// will it be removed from the distance map. The cell is only actually inserted
/// once into the distance map, regardless of the number of consecutive
/// insertions into the occupancy grid.
///
/// If cell reference counting is enabled with a distance field passed to the
/// OccupancyGrid on construction, the caller must not directly modify the
/// distance map. This may corrupt the invariant that the obstacle exists in
/// the distance map if its reference count is non-zero.
///
/// An arbitrary distance map implementation may be used with this class. If
/// none is specified, by calling the verbose constructor, an instance of
/// sbpl::PropagationDistanceField is constructed.

/// Construct an Occupancy Grid.
///
/// \param size_x Dimension of the grid along the X axis, in meters
/// \param size_y Dimension of the grid along the Y axis, in meters
/// \param size_z Dimension of the grid along the Z axis, in meters
/// \param resolution Resolution of the grid, in meters
/// \param origin_x X Coordinate of origin, in meters
/// \param origin_y Y Coordinate of origin, in meters
/// \param origin_z Z Coordinate of origin, in meters
/// \param max_dist The maximum distance away from obstacles to propagate
///     the distance field, in meters
/// \param propagate_negative_distances Whether to compute signed distances to
///     nearest obstacle boundaries
/// \param ref_counted Whether to reference count cells
OccupancyGrid::OccupancyGrid(
    double size_x, double size_y, double size_z,
    double resolution,
    double origin_x, double origin_y, double origin_z,
    double max_dist,
    bool ref_counted)
:
    m_grid(std::make_shared<PropagationDistanceField>(
            origin_x, origin_y, origin_z,
            size_x, size_y, size_z,
            resolution,
            max_dist)),
    reference_frame_(),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->numCellsY() * m_grid->numCellsZ()),
    m_y_stride(m_grid->numCellsZ()),
    m_counts()
{
    // distance field guaranteed to be empty -> faster initialization
    if (m_ref_counted) {
        m_counts.assign(getCellCount(), 0);
    }
}

/// Construct an OccupancyGrid from an existing distance map.
///
/// \param df The distance field
/// \param ref_counted Whether to reference count cells
OccupancyGrid::OccupancyGrid(
    const DistanceMapInterfacePtr& df,
    bool ref_counted)
:
    m_grid(df),
    reference_frame_(),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->numCellsY() * m_grid->numCellsZ()),
    m_y_stride(m_grid->numCellsZ()),
    m_counts()
{
    initRefCounts();
}

/// Copy constructor. Constructs the Occupancy Grid with a deep copy of the
/// contents of \p o.
OccupancyGrid::OccupancyGrid(const OccupancyGrid& o)
{
    m_grid.reset(o.m_grid->clone());
    reference_frame_ = o.reference_frame_;
    m_ref_counted = o.m_ref_counted;
    m_x_stride = o.m_x_stride;
    m_y_stride = o.m_y_stride;
    m_counts = o.m_counts;
}

/// Reset the grid, removing all obstacles setting distances to their
/// uninitialized values.
void OccupancyGrid::reset()
{
    // TODO: reset reference counts
    m_grid->reset();
}

/// Get all occupied voxels within an oriented cube region of the grid.
void OccupancyGrid::getOccupiedVoxels(
    const Eigen::Affine3d& pose,
    const std::vector<double>& dim,
    std::vector<Eigen::Vector3d>& voxels) const
{
    Eigen::Vector3d pos(pose.translation());
    Eigen::Matrix3d m(pose.rotation());

    for (double x = -0.5 * dim[0]; x <= 0.5 * dim[0]; x += m_grid->resolution()) {
    for (double y = -0.5 * dim[1]; y <= 0.5 * dim[1]; y += m_grid->resolution()) {
    for (double z = -0.5 * dim[2]; z <= 0.5 * dim[2]; z += m_grid->resolution()) {
        Eigen::Vector3d point(x, y, z);
        point = pose * point;

        if (getDistanceFromPoint(point.x(), point.y(), point.z()) <= 0.0) {
            voxels.push_back(point);
        }
    }
    }
    }
}

/// Get all occupied voxels within an axis-aligned cube region of the grid.
void OccupancyGrid::getOccupiedVoxels(
    double x_center,
    double y_center,
    double z_center,
    double radius,
    std::vector<Eigen::Vector3d>& voxels) const
{
    int x_c, y_c, z_c;
    worldToGrid(x_center, y_center, z_center, x_c, y_c, z_c);
    int radius_c = radius / resolution() + 0.5;

    Eigen::Vector3d v;

    iterateCells(
            x_c - radius_c, y_c - radius_c, z_c - radius_c,
            x_c + radius_c, y_c + radius_c, z_c + radius_c,
            [&](int x, int y, int z)
            {
                if (getDistanceFromPoint(x, y, z) == 0.0) {
                    gridToWorld(x, y, z, v.x(), v.y(), v.z());
                    voxels.push_back(v);
                }
            });
}

/// Count the number of obstacles in the occupancy grid.
size_t OccupancyGrid::getOccupiedVoxelCount() const
{
    size_t count = 0;
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getCellDistance(x, y, z) <= 0.0) {
            ++count;
        }
    });
    return count;
}

/// Gather all the obstacle points in the occupancy grid.
void OccupancyGrid::getOccupiedVoxels(
    std::vector<Eigen::Vector3d>& voxels) const
{
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getCellDistance(x, y, z) <= 0.0) {
            double wx, wy, wz;
            m_grid->gridToWorld(x, y, z, wx, wy, wz);
            voxels.emplace_back(wx, wy, wz);
        }
    });
}

/// Return a visualization of the distance map.
///
/// The visualization_msgs::MarkerArray's contents vary depending on the
/// argument:
///
///     "bounds": line markers for the bounding box of the distance field in
///               the namespace "collision_space_bounds"
///     "distance_field": cube markers for all voxels nearby occupied voxels
///                       in the namespace "distance_field"
///     "occupied_voxels" list of points representing all occupied voxels in
///                       the namespace "occupied voxels"
///
/// \param type "bounds", "distance_field", "occupied_voxels"
visualization_msgs::MarkerArray OccupancyGrid::getVisualization(
    const std::string& type) const
{
    if (type == "bounds") {
        return getBoundingBoxVisualization();
    }
    else if (type == "distance_field") {
        return getDistanceFieldVisualization();
    }
    else if (type == "occupied_voxels") {
        return getOccupiedVoxelsVisualization();
    }
    else {
        ROS_ERROR("No Occupancy Grid visualization of type '%s' found", type.c_str());
        return visualization_msgs::MarkerArray();
    }
}

/// Return a visualization of the bounding box of the distance map.
visualization_msgs::MarkerArray
OccupancyGrid::getBoundingBoxVisualization() const
{
    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker m;
    m.header.frame_id = getReferenceFrame();
    m.ns = "collision_space_bounds";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = resolution();
    leatherman::msgHSVToRGB(10.0, 1.0, 1.0, m.color);

    m.points.reserve(4 * (m_grid->numCellsX() + 2) + 4 * (m_grid->numCellsY()) + 4 * (m_grid->numCellsZ()));

    geometry_msgs::Point p;
    int x, y, z;

    for (x = -1; x <= m_grid->numCellsX(); ++x) {
        y = -1, z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->numCellsY(), z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = -1, z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->numCellsY(), z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    for (y = 0; y < m_grid->numCellsY(); ++y) {
        x = -1, z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = m_grid->numCellsX(), z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = -1, z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = m_grid->numCellsX(), z = m_grid->numCellsZ();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    for (z = 0; z < m_grid->numCellsZ(); ++z) {
        y = -1, x = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->numCellsY(), x = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = -1, x = m_grid->numCellsX();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->numCellsY(), x = m_grid->numCellsX();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    ma.markers = { m };

    return ma;
}

/// Return a visualization of the distance values stored in the distance map.
visualization_msgs::MarkerArray
OccupancyGrid::getDistanceFieldVisualization() const
{
    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker m;
    m.header.frame_id = reference_frame_;
    m.ns = "distance_field";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = m_grid->resolution();
    m.color.r = 1.0f;
    m.color.g = 0.5f;
    m.color.b = 0.0f;
    m.color.a = 0.4f;

    const double min_value = m_grid->resolution();
    const double max_value = m_grid->getUninitializedDistance();
    iterateCells([&](int x, int y, int z)
    {
        const double d = m_grid->getCellDistance(x, y, z);
        if (d >= min_value && d <= max_value) {
            double wx, wy, wz;
            m_grid->gridToWorld(x, y, z, wx, wy, wz);
            geometry_msgs::Point p;
            p.x = wx; p.y = wy; p.z = wz;
            m.points.push_back(p);
        }
    });

    ma.markers.push_back(m);
    return ma;
}

/// Return a visualization of the obstacle cells stored in the distance map.
visualization_msgs::MarkerArray
OccupancyGrid::getOccupiedVoxelsVisualization() const
{
    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker marker;

    marker.header.seq = 0;
    marker.header.stamp = ros::Time(0);
    marker.header.frame_id = getReferenceFrame();

    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);

    marker.scale.x = m_grid->resolution();
    marker.scale.y = m_grid->resolution();
    marker.scale.z = m_grid->resolution();

    marker.color.r = 0.8f;
    marker.color.g = 0.3f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;

    std::vector<Eigen::Vector3d> voxels;
    getOccupiedVoxels(voxels);

    marker.points.resize(voxels.size());
    for (size_t i = 0; i < voxels.size(); ++i) {
        marker.points[i].x = voxels[i].x();
        marker.points[i].y = voxels[i].y();
        marker.points[i].z = voxels[i].z();
    }

    ma.markers.push_back(marker);
    return ma;
}

/// Add a set of obstacle cells to the occupancy grid.
void OccupancyGrid::addPointsToField(
    const std::vector<Eigen::Vector3d>& points)
{
    if (m_ref_counted) {
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(points.size());
        int gx, gy, gz;
        for (const Eigen::Vector3d& v : points) {
            worldToGrid(v.x(), v.y(), v.z(), gx, gy, gz);

            if (isInBounds(gx, gy, gz)) {
                const int idx = coordToIndex(gx, gy, gz);

                if (m_counts[idx] == 0) {
                    pts.emplace_back(v.x(), v.y(), v.z());
                }

                ++m_counts[idx];
            }
        }
        m_grid->addPointsToMap(pts);
    }
    else {
        m_grid->addPointsToMap(points);
    }

}

/// Remove a set of obstacle cells from the occupancy grid.
void OccupancyGrid::removePointsFromField(
    const std::vector<Eigen::Vector3d>& points)
{
    if (m_ref_counted) {
        std::vector<Eigen::Vector3d> pts;
        pts.reserve(points.size());
        int gx, gy, gz;
        for (const Eigen::Vector3d& v : points) {
            worldToGrid(v.x(), v.y(), v.z(), gx, gy, gz);

            if (isInBounds(gx, gy, gz)) {
                int idx = coordToIndex(gx, gy, gz);

                if (m_counts[idx] > 0) {
                    --m_counts[idx];
                    if (m_counts[idx] == 0) {
                        pts.emplace_back(v.x(), v.y(), v.z());
                    }
                }
            }
        }
        m_grid->removePointsFromMap(pts);
    }
    else {
        m_grid->removePointsFromMap(points);
    }
}

/// Update the occupancy grid, removing obstacles that exist in the old obstacle
/// set, but not in the new obstacle set, and adding obstacles that exist in the
/// new obstacle set, but not in the old obstacle set.
void OccupancyGrid::updatePointsInField(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    // TODO: ref counting
    m_grid->updatePointsInMap(old_points, new_points);
}

void OccupancyGrid::initRefCounts()
{
    if (!m_ref_counted) {
        m_counts.clear();
        return;
    }

    int gidx = 0;
    m_counts.resize(getCellCount());
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getCellDistance(x, y, z) <= 0.0) {
            m_counts[gidx++] = 1;
        }
        else {
            m_counts[gidx++] = 0;
        }
    });
}

template <typename CellFunction>
void OccupancyGrid::iterateCells(CellFunction f) const
{
    for (int gx = 0; gx < m_grid->numCellsX(); ++gx) {
    for (int gy = 0; gy < m_grid->numCellsY(); ++gy) {
    for (int gz = 0; gz < m_grid->numCellsZ(); ++gz) {
        f(gx, gy, gz);
    }
    }
    }
}

template <typename CellFunction>
void OccupancyGrid::iterateCells(
    int fx, int fy, int fz,
    int tx, int ty, int tz,
    CellFunction f) const
{
    // NOTE: [x, y, z] = major -> minor indexes
    for (int x = fx; x < tx; ++x) {
    for (int y = fy; y < ty; ++y) {
    for (int z = fz; z < tz; ++z) {
        f(x, y, z);
    }
    }
    }
}

} // namespace sbpl
