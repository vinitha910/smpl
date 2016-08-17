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

#ifndef sbpl_occupancy_grid_h
#define sbpl_occupancy_grid_h

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
#include <moveit_msgs/CollisionObject.h>
#include <ros/console.h>
#include <sys/stat.h>
#include <visualization_msgs/MarkerArray.h>

namespace sbpl {

typedef std::shared_ptr<distance_field::PropagationDistanceField> PropagationDistanceFieldPtr;
typedef std::shared_ptr<const distance_field::PropagationDistanceField> PropagationDistanceFieldConstPtr;

/// \brief Lightweight layer on top of the PropagationDistanceField class that
/// carries occupancy information in some reference frame as well as the
/// distance transform
class OccupancyGrid
{
public:

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
    OccupancyGrid(
        double size_x, double size_y, double size_z,
        double resolution,
        double origin_x, double origin_y, double origin_z,
        double max_dist,
        bool propagate_negative_distances = false,
        bool ref_counted = false);

    /// \sa distance_field::PropagationDistanceField::PropagationDistanceField(
    ///         const octomap::OcTree&,
    ///         const octomap::point3d&,
    ///         const octomap::point3d&,
    ///         double, bool);
    OccupancyGrid(
        const octomap::OcTree& octree,
        const octomap::point3d& bbx_min,
        const octomap::point3d& bby_min,
        double max_distance,
        bool propagate_negative_distances = false,
        bool ref_counted = false);

    /// \sa distance_field::PropagationDistanceField::PropagationDistanceField(
    //         std::istream&, double, bool);
    OccupancyGrid(
        std::istream& stream,
        double max_distance,
        bool propagate_negative_distances = false,
        bool ref_counted = false);

    /// \brief Construct an OccupancyGrid with an unmanaged distance field
    /// \param df A pointer to the unmanaged distance field
    OccupancyGrid(const PropagationDistanceFieldPtr& df, bool ref_counted = false);

    ~OccupancyGrid();

    /// \brief Return a pointer to the distance field
    const PropagationDistanceFieldPtr& getDistanceField() const;

    /// \name Attributes
    ///@{

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
    void worldToGrid(double* wx, int* gx) const;

    /// \brief Get the distance, in cells, to the nearest occupied cell
    unsigned char getCell(int x, int y, int z) const;

    /// \brief Get the distance, in meters, to the nearest occupied cell
    double getCell(const int* xyz) const;

    /// \brief Get the distance, in meters, to the nearest occupied cell
    double getDistance(int x, int y, int z) const;

    /// \brief Get the distance, in meters, to the nearest occupied cell
    double getDistanceFromPoint(double x, double y, double z) const;

    // TODO: this whole implicit casting nonsense makes me nervous...factor
    // these (T, T, T) triples into point class representations
    bool isInBounds(double x, double y, double z) const;
    bool isInBounds(int x, int y, int z) const;

    size_t getOccupiedVoxelCount() const;

    /// \brief Get all occupied voxels in the grid
    void getOccupiedVoxels(std::vector<Eigen::Vector3d>& voxels) const;

    /// \brief Get all occupied voxels within a cubic region of the grid
    void getOccupiedVoxels(
        const geometry_msgs::Pose& pose,
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
    PropagationDistanceFieldPtr grid_;

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
    return grid_;
}

inline
double OccupancyGrid::getResolution() const
{
    return grid_->getResolution();
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
    return grid_->getUninitializedDistance();
}

inline
void OccupancyGrid::gridToWorld(
    int x, int y, int z,
    double& wx, double& wy, double& wz) const
{
    grid_->gridToWorld(x, y, z, wx, wy, wz);
}

inline
void OccupancyGrid::worldToGrid(
    double wx, double wy, double wz,
    int& x, int& y, int& z) const
{
    grid_->worldToGrid(wx, wy, wz, x, y, z);
}

inline
void OccupancyGrid::worldToGrid(double* wx, int* gx) const
{
    grid_->worldToGrid(wx[0], wx[1], wx[2], gx[0], gx[1], gx[2]);
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
    return (unsigned char)(grid_->getDistance(x,y,z) / grid_->getResolution());
}

inline
double OccupancyGrid::getCell(const int* xyz) const
{
    return grid_->getDistance(xyz[0], xyz[1], xyz[2]);
}

inline
double OccupancyGrid::getDistance(int x, int y, int z) const
{
    return grid_->getDistance(x, y, z);
}

inline
double OccupancyGrid::getDistanceFromPoint(double x, double y, double z) const
{
    return grid_->getDistance(x, y, z);
}

inline
bool OccupancyGrid::isInBounds(int x, int y, int z) const
{
    return (x >= 0 && x < grid_->getXNumCells() &&
            y >= 0 && y < grid_->getYNumCells() &&
            z >= 0 && z < grid_->getZNumCells());
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
    return grid_->getXNumCells() * grid_->getYNumCells() * grid_->getZNumCells();
}

typedef std::shared_ptr<OccupancyGrid> OccupancyGridPtr;
typedef std::shared_ptr<const OccupancyGrid> OccupancyGridConstPtr;

} // namespace sbpl

#endif
