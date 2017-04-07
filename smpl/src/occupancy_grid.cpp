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
#include <sstream>

 // system includes
#include <ros/console.h>
#include <leatherman/viz.h>

namespace sbpl {

/// \brief Construct an Occupancy Grid
/// \param size_x Dimension of the grid along the X axis, in meters
/// \param size_y Dimension of the grid along the Y axis, in meters
/// \param size_z Dimension of the grid along the Z axis, in meters
/// \param resolution Resolution of the grid, in meters
/// \param origin_x X Coordinate of origin, in meters
/// \param origin_y Y Coordinate of origin, in meters
/// \param origin_z Z Coordinate of origin, in meters
/// \param max_dist The maximum distance away from obstacles to propagate
///     the distance field, in meters
OccupancyGrid::OccupancyGrid(
    double size_x, double size_y, double size_z,
    double resolution,
    double origin_x, double origin_y, double origin_z,
    double max_dist,
    bool propagate_negative_distances,
    bool ref_counted)
:
    reference_frame_(),
    m_grid(std::make_shared<distance_field::PropagationDistanceField>(
            size_x, size_y, size_z,
            resolution,
            origin_x, origin_y, origin_z,
            max_dist, propagate_negative_distances)),
    m_df((distance_field::PropagationDistanceField*)m_grid.get()),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->getYNumCells() * m_grid->getZNumCells()),
    m_y_stride(m_grid->getZNumCells()),
    m_counts()
{
    // distance field guaranteed to be empty -> faster initialization
    if (m_ref_counted) {
        m_counts.assign(getCellCount(), 0);
    }
    m_half_res = 0.5 * getResolution();
    m_error = sqrt(3.0) * getResolution();
}

/// \sa distance_field::PropagationDistanceField::PropagationDistanceField(const octomap::OcTree&, const octomap::point3d&, const octomap::point3d&, double, bool);
OccupancyGrid::OccupancyGrid(
    const octomap::OcTree& octree,
    const octomap::point3d& bbx_min,
    const octomap::point3d& bby_min,
    double max_distance,
    bool propagate_negative_distances,
    bool ref_counted)
:
    reference_frame_(),
    m_grid(std::make_shared<distance_field::PropagationDistanceField>(
            octree, bbx_min, bby_min, max_distance, propagate_negative_distances)),
    m_df((distance_field::PropagationDistanceField*)m_grid.get()),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->getYNumCells() * m_grid->getZNumCells()),
    m_y_stride(m_grid->getZNumCells()),
    m_counts()
{
    initRefCounts();
    m_half_res = 0.5 * getResolution();
    m_error = sqrt(3.0) * getResolution();
}

/// \sa distance_field::PropagationDistanceField::PropagationDistanceField(std::istream&, double, bool);
OccupancyGrid::OccupancyGrid(
    std::istream& stream,
    double max_distance,
    bool propagate_negative_distances,
    bool ref_counted)
:
    reference_frame_(),
    m_grid(std::make_shared<distance_field::PropagationDistanceField>(
            stream, max_distance, propagate_negative_distances)),
    m_df((distance_field::PropagationDistanceField*)m_grid.get()),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->getYNumCells() * m_grid->getZNumCells()),
    m_y_stride(m_grid->getZNumCells()),
    m_counts()
{
    initRefCounts();
    m_half_res = 0.5 * getResolution();
}

/// \brief Construct an OccupancyGrid from an existing distance field
OccupancyGrid::OccupancyGrid(
    const PropagationDistanceFieldPtr& df,
    bool ref_counted)
:
    reference_frame_(),
    m_grid(df),
    m_df(dynamic_cast<distance_field::PropagationDistanceField*>(m_grid.get())),
    m_ref_counted(ref_counted),
    m_x_stride(m_grid->getYNumCells() * m_grid->getZNumCells()),
    m_y_stride(m_grid->getZNumCells()),
    m_counts()
{
    initRefCounts();
    m_half_res = 0.5 * getResolution();
    m_error = sqrt(3.0) * getResolution();
}

OccupancyGrid::OccupancyGrid(const OccupancyGrid& o)
{
    reference_frame_ = o.reference_frame_;
    if (o.m_grid) {
        std::stringstream ss;
        o.m_grid->writeToStream(ss);
        m_grid = std::make_shared<distance_field::PropagationDistanceField>(
                ss,
                // really, no other way to get the propagation distance?
                o.m_grid->getUninitializedDistance(),
                // really, no way to get this either?
                false);
        m_df = (distance_field::PropagationDistanceField*)(m_grid.get());
    }
    m_ref_counted = o.m_ref_counted;
    m_x_stride = o.m_x_stride;
    m_y_stride = o.m_y_stride;
    m_counts = o.m_counts;
    m_half_res = o.m_half_res;
    m_error = sqrt(3.0) * getResolution();
}

OccupancyGrid::~OccupancyGrid()
{
}

void OccupancyGrid::getGridSize(int& dim_x, int& dim_y, int& dim_z) const
{
    dim_x = m_grid->getXNumCells();
    dim_y = m_grid->getYNumCells();
    dim_z = m_grid->getZNumCells();
}

void OccupancyGrid::getWorldSize(double& dim_x, double& dim_y, double& dim_z) const
{
    dim_x = m_grid->getSizeX();
    dim_y = m_grid->getSizeY();
    dim_z = m_grid->getSizeZ();
}

void OccupancyGrid::reset()
{
    m_grid->reset();
}

void OccupancyGrid::getOrigin(double& wx, double& wy, double& wz) const
{
    m_grid->gridToWorld(0, 0, 0, wx, wy, wz);
}

void OccupancyGrid::getOccupiedVoxels(
    const Eigen::Affine3d& pose,
    const std::vector<double>& dim,
    std::vector<Eigen::Vector3d>& voxels) const
{
    Eigen::Vector3d pos(pose.translation());
    Eigen::Matrix3d m(pose.rotation());

    for (double x = 0 - 0.5 * dim[0]; x <= 0.5 * dim[0]; x += m_grid->getResolution()) {
    for (double y = 0 - 0.5 * dim[1]; y <= 0.5 * dim[1]; y += m_grid->getResolution()) {
    for (double z = 0 - 0.5 * dim[2]; z <= 0.5 * dim[2]; z += m_grid->getResolution()) {
        Eigen::Vector3d point(x, y, z);
        point = pose * point;

        if (getDistanceFromPoint(point.x(), point.y(), point.z()) <= 0.0) {
            voxels.push_back(point);
        }
    }
    }
    }
}

void OccupancyGrid::getOccupiedVoxels(
    double x_center,
    double y_center,
    double z_center,
    double radius,
    std::vector<Eigen::Vector3d>& voxels) const
{
    int x_c, y_c, z_c;
    worldToGrid(x_center, y_center, z_center, x_c, y_c, z_c);
    int radius_c = radius / getResolution() + 0.5;

    Eigen::Vector3d v;

    iterateCells(
            x_c - radius_c, y_c - radius_c, z_c - radius_c,
            x_c + radius_c, y_c + radius_c, z_c + radius_c,
            [&](int x, int y, int z)
            {
                if (getCell(x, y, z) == 0) {
                    gridToWorld(x, y, z, v.x(), v.y(), v.z());
                    voxels.push_back(v);
                }
            });
}

size_t OccupancyGrid::getOccupiedVoxelCount() const
{
    size_t count = 0;
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getDistance(x, y, z) == 0.0) {
            ++count;
        }
    });
    return count;
}

void OccupancyGrid::getOccupiedVoxels(
    std::vector<Eigen::Vector3d>& voxels) const
{
    iterateCells([&](int x, int y, int z)
    {
        if (m_grid->getDistance(x, y, z) == 0.0) {
            double wx, wy, wz;
            m_grid->gridToWorld(x, y, z, wx, wy, wz);
            voxels.emplace_back(wx, wy, wz);
        }
    });
}

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
    m.scale.x = m.scale.y = m.scale.z = getResolution();
    leatherman::msgHSVToRGB(10.0, 1.0, 1.0, m.color);

    m.points.reserve(4 * (m_grid->getXNumCells() + 2) + 4 * (m_grid->getYNumCells()) + 4 * (m_grid->getZNumCells()));

    geometry_msgs::Point p;
    int x, y, z;

    for (x = -1; x <= m_grid->getXNumCells(); ++x) {
        y = -1, z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->getYNumCells(), z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = -1, z = m_grid->getZNumCells();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->getYNumCells(), z = m_grid->getZNumCells();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    for (y = 0; y < m_grid->getYNumCells(); ++y) {
        x = -1, z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = m_grid->getXNumCells(), z = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = -1, z = m_grid->getZNumCells();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = m_grid->getXNumCells(), z = m_grid->getZNumCells();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    for (z = 0; z < m_grid->getZNumCells(); ++z) {
        y = -1, x = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->getYNumCells(), x = -1;
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = -1, x = m_grid->getXNumCells();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = m_grid->getYNumCells(), x = m_grid->getXNumCells();
        m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    ma.markers = { m };

    return ma;
}

visualization_msgs::MarkerArray
OccupancyGrid::getDistanceFieldVisualization() const
{
    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker m;
    m_grid->getIsoSurfaceMarkers(
            m_grid->getResolution(),
            getMaxDistance(),
            getReferenceFrame(),
            ros::Time(0),
            m);
    m.color.a += 0.2;

    ma.markers.push_back(m);
    return ma;
}

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

    marker.scale.x = m_grid->getResolution();
    marker.scale.y = m_grid->getResolution();
    marker.scale.z = m_grid->getResolution();

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

void OccupancyGrid::addPointsToField(
    const std::vector<Eigen::Vector3d>& points)
{
    EigenSTL::vector_Vector3d pts;

    if (m_ref_counted) {
        int gx, gy, gz;
        pts.reserve(points.size());
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
    }
    else {
        pts = toAlignedVector(points);
    }

    m_grid->addPointsToField(pts);
}

void OccupancyGrid::removePointsFromField(
    const std::vector<Eigen::Vector3d>& points)
{
    EigenSTL::vector_Vector3d pts;

    if (m_ref_counted) {
        int gx, gy, gz;
        pts.reserve(points.size());
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
    }
    else {
        pts = toAlignedVector(points);
    }

    m_grid->removePointsFromField(pts);
}

void OccupancyGrid::updatePointsInField(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    // TODO: ref counting
    m_grid->updatePointsInField(toAlignedVector(old_points), toAlignedVector(new_points));
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
        if (m_grid->getDistance(x, y, z) <= 0.0) {
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
    for (int gx = 0; gx < m_grid->getXNumCells(); ++gx) {
        for (int gy = 0; gy < m_grid->getYNumCells(); ++gy) {
            for (int gz = 0; gz < m_grid->getZNumCells(); ++gz) {
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

EigenSTL::vector_Vector3d OccupancyGrid::toAlignedVector(
    const std::vector<Eigen::Vector3d>& v) const
{
    EigenSTL::vector_Vector3d pts(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        pts[i] = Eigen::Vector3d(v[i].x(), v[i].y(), v[i].z());
    }
    return pts;
}

} // namespace sbpl
