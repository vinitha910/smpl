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

#include <sbpl_arm_planner/occupancy_grid.h>

 // system includes
#include <ros/console.h>
#include <leatherman/viz.h>

namespace sbpl {

OccupancyGrid::OccupancyGrid(
    double size_x, double size_y, double size_z,
    double resolution,
    double origin_x, double origin_y, double origin_z,
    double max_dist,
    bool propagate_negative_distances,
    bool ref_counted)
:
    reference_frame_(),
    grid_(std::make_shared<distance_field::PropagationDistanceField>(
            size_x, size_y, size_z,
            resolution,
            origin_x, origin_y, origin_z,
            max_dist, propagate_negative_distances)),
    m_ref_counted(ref_counted),
    m_x_stride(grid_->getYNumCells() * grid_->getZNumCells()),
    m_y_stride(grid_->getZNumCells()),
    m_counts()
{
    // distance field guaranteed to be empty -> faster initialization
    if (m_ref_counted) {
        m_counts.assign(getCellCount(), 0);
    }
    m_half_res = 0.5 * getResolution();
}

OccupancyGrid::OccupancyGrid(
    const octomap::OcTree& octree,
    const octomap::point3d& bbx_min,
    const octomap::point3d& bby_min,
    double max_distance,
    bool propagate_negative_distances,
    bool ref_counted)
:
    reference_frame_(),
    grid_(std::make_shared<distance_field::PropagationDistanceField>(
            octree, bbx_min, bby_min, max_distance, propagate_negative_distances)),
    m_ref_counted(ref_counted),
    m_x_stride(grid_->getYNumCells() * grid_->getZNumCells()),
    m_y_stride(grid_->getZNumCells()),
    m_counts()
{
    initRefCounts();
    m_half_res = 0.5 * getResolution();
}

OccupancyGrid::OccupancyGrid(
    std::istream& stream,
    double max_distance,
    bool propagate_negative_distances,
    bool ref_counted)
:
    reference_frame_(),
    grid_(std::make_shared<distance_field::PropagationDistanceField>(
            stream, max_distance, propagate_negative_distances)),
    m_ref_counted(ref_counted),
    m_x_stride(grid_->getYNumCells() * grid_->getZNumCells()),
    m_y_stride(grid_->getZNumCells()),
    m_counts()
{
    initRefCounts();
    m_half_res = 0.5 * getResolution();
}

OccupancyGrid::OccupancyGrid(
    const PropagationDistanceFieldPtr& df,
    bool ref_counted)
:
    reference_frame_(),
    grid_(df),
    m_ref_counted(ref_counted),
    m_x_stride(grid_->getYNumCells() * grid_->getZNumCells()),
    m_y_stride(grid_->getZNumCells()),
    m_counts()
{
    initRefCounts();
    m_half_res = 0.5 * getResolution();
}

OccupancyGrid::~OccupancyGrid()
{
}

void OccupancyGrid::getGridSize(int& dim_x, int& dim_y, int& dim_z) const
{
    dim_x = grid_->getXNumCells();
    dim_y = grid_->getYNumCells();
    dim_z = grid_->getZNumCells();
}

void OccupancyGrid::getWorldSize(double& dim_x, double& dim_y, double& dim_z) const
{
    dim_x = grid_->getSizeX();
    dim_y = grid_->getSizeY();
    dim_z = grid_->getSizeZ();
}

void OccupancyGrid::reset()
{
    grid_->reset();
}

void OccupancyGrid::getOrigin(double& wx, double& wy, double& wz) const
{
    grid_->gridToWorld(0, 0, 0, wx, wy, wz);
}

void OccupancyGrid::getOccupiedVoxels(
    const geometry_msgs::Pose& pose,
    const std::vector<double>& dim,
    std::vector<Eigen::Vector3d>& voxels) const
{
    Eigen::Vector3d vin, vout, v(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Matrix3d m(Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));

    for (double x = 0 - dim[0] / 2.0; x <= dim[0] / 2.0; x += grid_->getResolution()) {
        for (double y = 0 - dim[1] / 2.0; y <= dim[1] / 2.0; y += grid_->getResolution()) {
            for (double z = 0 - dim[2] / 2.0; z <= dim[2] / 2.0; z += grid_->getResolution()) {
                vin(0) = x;
                vin(1) = y;
                vin(2) = z;
                vout = m * vin;
                vout += v;

                if (getDistanceFromPoint(v.x(), v.y(), v.z()) <= 0.0) {
                    voxels.push_back(vout);
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
        if (grid_->getDistance(x, y, z) == 0.0) {
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
        if (grid_->getDistance(x, y, z) == 0.0) {
            double wx, wy, wz;
            grid_->gridToWorld(x, y, z, wx, wy, wz);
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

    m.points.reserve(4 * (grid_->getXNumCells() + 2) + 4 * (grid_->getYNumCells()) + 4 * (grid_->getZNumCells()));

    geometry_msgs::Point p;
    int x, y, z;

    for (x = -1; x <= grid_->getXNumCells(); ++x) {
        y = -1, z = -1;
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = grid_->getYNumCells(), z = -1;
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = -1, z = grid_->getZNumCells();
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = grid_->getYNumCells(), z = grid_->getZNumCells();
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    for (y = 0; y < grid_->getYNumCells(); ++y) {
        x = -1, z = -1;
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = grid_->getXNumCells(), z = -1;
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = -1, z = grid_->getZNumCells();
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        x = grid_->getXNumCells(), z = grid_->getZNumCells();
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);
    }

    for (z = 0; z < grid_->getZNumCells(); ++z) {
        y = -1, x = -1;
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = grid_->getYNumCells(), x = -1;
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = -1, x = grid_->getXNumCells();
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
        m.points.push_back(p);

        y = grid_->getYNumCells(), x = grid_->getXNumCells();
        grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
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
    grid_->getIsoSurfaceMarkers(
            grid_->getResolution(),
            getMaxDistance(),
            getReferenceFrame(),
            ros::Time::now(),
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
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = getReferenceFrame();

    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);

    marker.scale.x = grid_->getResolution();
    marker.scale.y = grid_->getResolution();
    marker.scale.z = grid_->getResolution();

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

    grid_->addPointsToField(pts);
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

    grid_->removePointsFromField(pts);
}

void OccupancyGrid::updatePointsInField(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    // TODO: ref counting
    grid_->updatePointsInField(toAlignedVector(old_points), toAlignedVector(new_points));
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
        if (grid_->getDistance(x, y, z) <= 0.0) {
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
    for (int gx = 0; gx < grid_->getXNumCells(); ++gx) {
        for (int gy = 0; gy < grid_->getYNumCells(); ++gy) {
            for (int gz = 0; gz < grid_->getZNumCells(); ++gz) {
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
