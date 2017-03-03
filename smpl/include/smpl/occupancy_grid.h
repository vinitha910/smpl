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

#ifndef SMPL_OCCUPANCY_GRID_H
#define SMPL_OCCUPANCY_GRID_H

// standard includes
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

// system includes
#include <Eigen/Geometry>
#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <ros/console.h>
#include <sys/stat.h>
#include <visualization_msgs/MarkerArray.h>

namespace sbpl {

typedef std::shared_ptr<distance_field::DistanceField> PropagationDistanceFieldPtr;
typedef std::shared_ptr<const distance_field::DistanceField> PropagationDistanceFieldConstPtr;

/// \brief Lightweight layer on top of the PropagationDistanceField class that
/// carries occupancy information in some reference frame as well as the
/// distance transform
class OccupancyGrid
{
public:

    OccupancyGrid(
        double size_x, double size_y, double size_z,
        double resolution,
        double origin_x, double origin_y, double origin_z,
        double max_dist,
        bool propagate_negative_distances = false,
        bool ref_counted = false);

    OccupancyGrid(
        const octomap::OcTree& octree,
        const octomap::point3d& bbx_min,
        const octomap::point3d& bby_min,
        double max_distance,
        bool propagate_negative_distances = false,
        bool ref_counted = false);

    OccupancyGrid(
        std::istream& stream,
        double max_distance,
        bool propagate_negative_distances = false,
        bool ref_counted = false);

    OccupancyGrid(
        const PropagationDistanceFieldPtr& df,
        bool ref_counted = false);

    OccupancyGrid(const OccupancyGrid& o);

    ~OccupancyGrid();

    /// \brief Return a pointer to the distance field
    const PropagationDistanceFieldPtr& getDistanceField() const;

    /// \name Attributes
    ///@{

    int numCellsX() const { return m_grid->getXNumCells(); }
    int numCellsY() const { return m_grid->getYNumCells(); }
    int numCellsZ() const { return m_grid->getZNumCells(); }
    double originX() const { return m_grid->getOriginX(); }
    double originY() const { return m_grid->getOriginY(); }
    double originZ() const { return m_grid->getOriginZ(); }
    double sizeX() const { return m_grid->getSizeX(); }
    double sizeY() const { return m_grid->getSizeY(); }
    double sizeZ() const { return m_grid->getSizeZ(); }

    double resolution() const { return m_grid->getResolution(); }

    /// \brief Get the dimensions of the grid, in cells
    void getGridSize(int &dim_x, int &dim_y, int &dim_z) const;

    /// \brief Get the dimensions of the world, in meters
    void getWorldSize(double &dim_x, double &dim_y, double &dim_z) const;

    /// \brief Get the origin of the world, in meters
    void getOrigin(double &wx, double &wy, double &wz) const;

    /// \brief Get the resolution of the world, in meters
    double getResolution() const;
    double getHalfResolution() const;

    /// \brief Get the maximum distance to which the distance field is computed
    double getMaxDistance() const;

    const std::string& getReferenceFrame() const;
    void setReferenceFrame(const std::string& frame);

    ///@}

    /// \name Grid Cell Accessors
    ///@{

    /// \brief Convert grid cell coords into world coords
    void gridToWorld(
        int x, int y, int z,
        double &wx, double &wy, double &wz) const;

    /// \brief Convert world coords into grid cell coords
    void worldToGrid(
        double wx, double wy, double wz,
        int &x, int &y, int &z) const;

    /// \brief Convert world coords into grid cell coords
    /// \param wx A double array of size 3 containing the world coordinates
    /// \param gx An int array of size 3 for storing the grid cell coordinates
    void worldToGrid(const double* wx, int* gx) const;

    /// \brief Get the distance, in cells, to the nearest occupied cell
    unsigned char getCell(int x, int y, int z) const;

    /// \brief Get the distance, in meters, to the nearest occupied cell
    double getCell(const int* xyz) const;

    /// \brief Get the distance, in meters, to the nearest occupied cell
    double getDistance(int x, int y, int z) const;

    /// \brief Get the distance, in meters, to the nearest occupied cell
    double getDistanceFromPoint(double x, double y, double z) const;

    /// \brief Get the distance to the, in meters, to the border
    double getDistanceToBorder(int x, int y, int z) const;

    double getDistanceToBorder(double x, double y, double z) const;

    // TODO: this whole implicit casting nonsense makes me nervous...factor
    // these (T, T, T) triples into point class representations
    bool isInBounds(double x, double y, double z) const;
    bool isInBounds(int x, int y, int z) const;

    size_t getOccupiedVoxelCount() const;

    /// \brief Get all occupied voxels in the grid
    void getOccupiedVoxels(std::vector<Eigen::Vector3d>& voxels) const;

    /// \brief Get all occupied voxels within a cubic region of the grid
    void getOccupiedVoxels(
        const Eigen::Affine3d& pose,
        const std::vector<double>& dim,
        std::vector<Eigen::Vector3d>& voxels) const;

    /// \brief Get all occupied voxels within a cubic region of the grid
    void getOccupiedVoxels(
        double x_center,
        double y_center,
        double z_center,
        double radius,
        std::vector<Eigen::Vector3d>& voxels) const;

    ///@}

    /// \name Modifiers
    ///@{

    void addPointsToField(const std::vector<Eigen::Vector3d>& points);
    void removePointsFromField(const std::vector<Eigen::Vector3d>& points);

    /// \sa distance_field::PropagationDistanceField::updatePointsInField
    void updatePointsInField(
        const std::vector<Eigen::Vector3d>& old_points,
        const std::vector<Eigen::Vector3d>& new_points);

    /// \sa distance_field::PropagationDistanceField::reset
    void reset();

    ///@}

    /// \name Visualization
    ///@{

    /// \brief Return a visualization of the distance field
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
    visualization_msgs::MarkerArray getVisualization(
        const std::string& type) const;

    visualization_msgs::MarkerArray getBoundingBoxVisualization() const;
    visualization_msgs::MarkerArray getDistanceFieldVisualization() const;
    visualization_msgs::MarkerArray getOccupiedVoxelsVisualization() const;

    ///@}

private:

    std::string reference_frame_;
    PropagationDistanceFieldPtr m_grid;

    bool m_ref_counted;
    int m_x_stride;
    int m_y_stride;
    std::vector<int> m_counts;
    double m_half_res;

    void initRefCounts();

    int coordToIndex(int x, int y, int z) const;

    int getCellCount() const;

    template <typename CellFunction>
    void iterateCells(CellFunction f) const;

    template <typename CellFunction>
    void iterateCells(
        int fx, int fy, int fz,
        int tx, int ty, int tz,
        CellFunction f) const;

    EigenSTL::vector_Vector3d toAlignedVector(
        const std::vector<Eigen::Vector3d>& v) const;
};

inline
const PropagationDistanceFieldPtr& OccupancyGrid::getDistanceField() const
{
    return m_grid;
}

inline
double OccupancyGrid::getResolution() const
{
    return m_grid->getResolution();
}

inline
double OccupancyGrid::getHalfResolution() const
{
    return m_half_res;
}

inline
double OccupancyGrid::getMaxDistance() const
{
    // HACK: embedded knowledge of PropagationDistanceField here
    return m_grid->getUninitializedDistance();
}

inline
void OccupancyGrid::gridToWorld(
    int x, int y, int z,
    double& wx, double& wy, double& wz) const
{
    m_grid->gridToWorld(x, y, z, wx, wy, wz);
}

inline
void OccupancyGrid::worldToGrid(
    double wx, double wy, double wz,
    int& x, int& y, int& z) const
{
    m_grid->worldToGrid(wx, wy, wz, x, y, z);
}

inline
void OccupancyGrid::worldToGrid(const double* wx, int* gx) const
{
    m_grid->worldToGrid(wx[0], wx[1], wx[2], gx[0], gx[1], gx[2]);
}

inline
const std::string& OccupancyGrid::getReferenceFrame() const
{
    return reference_frame_;
}

inline
void OccupancyGrid::setReferenceFrame(const std::string& frame)
{
    reference_frame_ = frame;
}

inline
unsigned char OccupancyGrid::getCell(int x, int y, int z) const
{
    return (unsigned char)(m_grid->getDistance(x,y,z) / m_grid->getResolution());
}

inline
double OccupancyGrid::getCell(const int* xyz) const
{
    return m_grid->getDistance(xyz[0], xyz[1], xyz[2]);
}

inline
double OccupancyGrid::getDistance(int x, int y, int z) const
{
    return m_grid->getDistance(x, y, z);
}

inline
double OccupancyGrid::getDistanceFromPoint(double x, double y, double z) const
{
    return m_grid->getDistance(x, y, z);
}

inline
double OccupancyGrid::getDistanceToBorder(int x, int y, int z) const
{
    if (!isInBounds(x, y, z)) {
        return 0.0;
    }
    int dx = std::min(m_grid->getXNumCells() - x, x);
    int dy = std::min(m_grid->getYNumCells() - y, y);
    int dz = std::min(m_grid->getZNumCells() - z, z);
    return m_grid->getResolution() * std::min(dx, std::min(dy, dz));
}

inline
double OccupancyGrid::getDistanceToBorder(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return getDistanceToBorder(gx, gy, gz);
}

inline
bool OccupancyGrid::isInBounds(int x, int y, int z) const
{
    return (x >= 0 && x < m_grid->getXNumCells() &&
            y >= 0 && y < m_grid->getYNumCells() &&
            z >= 0 && z < m_grid->getZNumCells());
}

inline
bool OccupancyGrid::isInBounds(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return isInBounds(gx, gy, gz);
}

inline
int OccupancyGrid::coordToIndex(int x, int y, int z) const
{
    return x * m_x_stride + y * m_y_stride + z;
}

inline
int OccupancyGrid::getCellCount() const
{
    return m_grid->getXNumCells() * m_grid->getYNumCells() * m_grid->getZNumCells();
}

typedef std::shared_ptr<OccupancyGrid> OccupancyGridPtr;
typedef std::shared_ptr<const OccupancyGrid> OccupancyGridConstPtr;

} // namespace sbpl

#endif
