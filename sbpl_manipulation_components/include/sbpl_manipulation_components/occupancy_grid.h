////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Maxim Likhachev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the University of Pennsylvania nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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

#ifndef sbpl_arm_planner_OccupancyGrid_h
#define sbpl_arm_planner_OccupancyGrid_h

#include <cmath>
#include <fstream>
#include <vector>
#include <Eigen/Geometry>
#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/console.h>
#include <sys/stat.h>
#include <tf/LinearMath/Vector3.h>
#include <visualization_msgs/MarkerArray.h>

namespace sbpl_arm_planner {

/// \brief Lightweight layer on top of the PropagationDistanceField class that
/// carries occupancy information in some reference frame as well as the
/// distance transform
class OccupancyGrid
{
public:

    /// \brief Construct an Occupancy Grid
    /// \param dim_x Dimension of the grid along the X axis, in meters
    /// \param dim_y Dimension of the grid along the Y axis, in meters
    /// \param dim_z Dimension of the grid along the Z axis, in meters
    /// \param resolution Resolution of the grid, in meters
    /// \param origin_x X Coordinate of origin, in meters
    /// \param origin_y Y Coordinate of origin, in meters
    /// \param origin_z Z Coordinate of origin, in meters
    /// \param max_dist The maximum distance away from obstacles to propagate
    ///     the distance field, in meters
    OccupancyGrid(
        double dim_x, double dim_y, double dim_z,
        double resolution,
        double origin_x, double origin_y, double origin_z,
        double max_dist);

    /// \brief Construct an OccupancyGrid with an unmanaged distance field
    /// \param df A pointer to the unmanaged distance field
    OccupancyGrid(distance_field::PropagationDistanceField* df);

    ~OccupancyGrid();

    /// \brief Return a pointer to the distance field
    distance_field::PropagationDistanceField* getDistanceFieldPtr() const;

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

    /// \brief Convert world coords into grid cell coords*/
    void worldToGrid(
        double wx, double wy, double wz,
        int &x, int &y, int &z) const;

    unsigned char getCell(int x, int y, int z) const;

    double getCell(int *xyz) const;

    double getDistance(int x, int y, int z) const;

    double getDistanceFromPoint(double x, double y, double z) const;

    // TODO: this whole implicit casting nonsense makes me nervous...factor
    // these (T, T, T) triples into point class representations
    bool isInBounds(double x, double y, double z) const;
    bool isInBounds(int x, int y, int z) const;

    /// \brief Get all occupied voxels in the grid
    void getOccupiedVoxels(std::vector<geometry_msgs::Point>& voxels) const;

    /// \brief Get all occupied voxels within a cubic region of the grid
    void getOccupiedVoxels(
        const geometry_msgs::Pose& pose,
        const std::vector<double>& dim,
        std::vector<Eigen::Vector3d>& voxels) const;

    /// \brief Get all occupied voxels within a spherical region of the grid
    void getOccupiedVoxels(
        double x_center,
        double y_center,
        double z_center,
        double radius,
        std::vector<geometry_msgs::Point>& voxels) const;

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

    bool delete_grid_;
    std::string reference_frame_;
    distance_field::PropagationDistanceField* grid_;

    EigenSTL::vector_Vector3d toAlignedVector(
        const std::vector<Eigen::Vector3d>& v) const;
};

inline
distance_field::PropagationDistanceField*
OccupancyGrid::getDistanceFieldPtr() const
{
    return grid_;
}

inline 
double OccupancyGrid::getResolution() const
{
    return grid_->getResolution();
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
    double &wx, double &wy, double &wz) const
{
    grid_->gridToWorld(x, y, z, wx, wy, wz);
}

inline
void OccupancyGrid::worldToGrid(
    double wx, double wy, double wz,
    int &x, int &y, int &z) const
{
    grid_->worldToGrid(wx, wy, wz, x, y, z);

    if ((x > 10000) || (y > 10000) || (z > 10000) || (x < 0) || (y < 0) || (z < 0)) {
        ROS_ERROR("[grid] worldToGrid converted %0.5f %0.5f %0.5f to %d %d %d", wx, wy, wz, x, y, z);
        fflush(stdout);
    }
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
double OccupancyGrid::getCell(int *xyz) const
{
    return grid_->getDistance(xyz[0], xyz[1], xyz[2]);
}

inline
double OccupancyGrid::getDistance(int x, int y, int z) const
{
    return grid_->getDistance(x,y,z);
}

inline
double OccupancyGrid::getDistanceFromPoint(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return grid_->getDistance(gx, gy, gz);
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

} // namespace sbpl_arm_planner

#endif
