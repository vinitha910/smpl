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

#include <smpl/distance_map/detail/distance_map_base.h>

namespace sbpl {

DistanceMapBase::DistanceMapBase(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double resolution,
    double max_dist)
:
    m_cells(),
    m_origin_x(origin_x),
    m_origin_y(origin_y),
    m_origin_z(origin_z),
    m_size_x(size_x),
    m_size_y(size_y),
    m_size_z(size_z),
    m_res(resolution),
    m_max_dist(max_dist),
    m_inv_res(1.0 / resolution),
    m_dmax_int((int)std::ceil(m_max_dist * m_inv_res)),
    m_dmax_sqrd_int(m_dmax_int * m_dmax_int),
    m_bucket(m_dmax_sqrd_int + 1),
    m_no_update_dir(dirnum(0, 0, 0)),
    m_neighbors(),
    m_indices(),
    m_neighbor_ranges(),
    m_neighbor_offsets(),
    m_neighbor_dirs(),
    m_open(),
    m_rem_stack()
{
    int cell_count_x = (int)(size_x * m_inv_res + 0.5) + 2;
    int cell_count_y = (int)(size_y * m_inv_res + 0.5) + 2;
    int cell_count_z = (int)(size_z * m_inv_res + 0.5) + 2;

    m_open.resize(m_dmax_sqrd_int + 1);

    m_sqrt_table.resize(m_dmax_sqrd_int + 1, 0.0);
    for (int i = 0; i < m_dmax_sqrd_int + 1; ++i) {
        m_sqrt_table[i] = m_res * std::sqrt((double)i);
    }

    // initialize non-border free cells
    m_cells.resize(cell_count_x, cell_count_y, cell_count_z);
    for (int x = 1; x < m_cells.xsize() - 1; ++x) {
    for (int y = 1; y < m_cells.ysize() - 1; ++y) {
    for (int z = 1; z < m_cells.zsize() - 1; ++z) {
        Cell& c = m_cells(x, y, z);
        c.x = x;
        c.y = y;
        c.z = z;
        c.dist = m_dmax_sqrd_int;
        c.dist_new = m_dmax_sqrd_int;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
        c.dist_old = m_dmax_sqrd_int;
#endif
        c.obs = nullptr;
        c.bucket = -1;
        c.dir = m_no_update_dir;
    }
    }
    }

    // init neighbors for forward propagation
    CreateNeighborUpdateList(m_neighbors, m_indices, m_neighbor_ranges);

    for (size_t i = 0; i < m_indices.size(); ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];
        m_neighbor_offsets[i] = 0;
        m_neighbor_offsets[i] += neighbor.x() * m_cells.zsize() * m_cells.ysize();
        m_neighbor_offsets[i] += neighbor.y() * m_cells.zsize();
        m_neighbor_offsets[i] += neighbor.z() * 1;

        if (i < NON_BORDER_NEIGHBOR_LIST_SIZE) {
            m_neighbor_dirs[i] = dirnum(neighbor.x(), neighbor.y(), neighbor.z());
        } else {
            m_neighbor_dirs[i] = dirnum(neighbor.x(), neighbor.y(), neighbor.z(), 1);
        }
    }

    initBorderCells();
}

/// Return the size along the x axis of the bounding volume.
double DistanceMapBase::sizeX() const
{
    return m_size_x;
}

/// Return the size along the y axis of the bounding volume.
double DistanceMapBase::sizeY() const
{
    return m_size_y;
}

/// Return the size along the z axis of the bounding volume.
double DistanceMapBase::sizeZ() const
{
    return m_size_z;
}

/// Return the origin along x axis.
double DistanceMapBase::originX() const
{
    return m_origin_x;
}

/// Return the origin along the y axis.
double DistanceMapBase::originY() const
{
    return m_origin_y;
}

/// Return the origin along the z axis.
double DistanceMapBase::originZ() const
{
    return m_origin_z;
}

/// Return the resolution of the distance map.
double DistanceMapBase::resolution() const
{
    return m_res;
}

/// Return the distance value for an invalid cell.
double DistanceMapBase::maxDistance() const
{
    return m_max_dist;
}

/// Return the number of cells along the x axis.
int DistanceMapBase::numCellsX() const
{
    return m_cells.xsize() - 2;
}

/// Return the number of cells along the y axis.
int DistanceMapBase::numCellsY() const
{
    return m_cells.ysize() - 2;
}

/// Return the number of cells along the z axis.
int DistanceMapBase::numCellsZ() const
{
    return m_cells.zsize() - 2;
}

/// Return the effective grid coordinates of the cell containing the given point
/// specified in world coordinates.
void DistanceMapBase::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    world_x = (m_origin_x - m_res) + (x + 1) * m_res;
    world_y = (m_origin_y - m_res) + (y + 1) * m_res;
    world_z = (m_origin_z - m_res) + (z + 1) * m_res;
}

/// Return the point in world coordinates marking the center of the cell at the
/// given effective grid coordinates.
void DistanceMapBase::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    x = (int)(m_inv_res * (world_x - (m_origin_x - m_res)) + 0.5) - 1;
    y = (int)(m_inv_res * (world_y - (m_origin_y - m_res)) + 0.5) - 1;
    z = (int)(m_inv_res * (world_z - (m_origin_z - m_res)) + 0.5) - 1;
}

/// Test if a cell is outside the bounding volume.
bool DistanceMapBase::isCellValid(int x, int y, int z) const
{
    return x >= 0 && x < m_cells.xsize() - 2 &&
        y >= 0 && y < m_cells.ysize() - 2 &&
        z >= 0 && z < m_cells.zsize() - 2;
}

/// Return the distance of a cell from its nearest obstacle. This function will
/// also consider the distance to the nearest border cell. A value of 0.0 is
/// returned for obstacle cells and cells outside of the bounding volume.
double DistanceMapBase::getDistance(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return getDistance(gx, gy, gz);
}

/// Return the distance of a cell from its nearest obstacle cell. This function
/// will also consider the distance to the nearest border cell. A value of 0.0
/// is returned for obstacle cells and cells outside of the bounding volume.
double DistanceMapBase::getDistance(int x, int y, int z) const
{
    if (!isCellValid(x, y, z)) {
        return 0.0;
    }

    int d2 = m_cells(x + 1, y + 1, z + 1).dist;
    return m_sqrt_table[d2];
}

void DistanceMapBase::initBorderCells()
{
    auto init_obs_cell = [&](int x, int y, int z) {
        Cell& c = m_cells(x, y, z);
        c.x = x;
        c.y = y;
        c.z = z;
        c.dist = m_dmax_sqrd_int;
        c.dist_new = 0;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
        c.dist_old = m_dmax_sqrd_int;
#endif
        c.obs = &c;
        c.bucket = -1;

        int src_dir_x = (x == 0) ? 1 : (x == m_cells.xsize() - 1 ? -1 : 0);
        int src_dir_y = (y == 0) ? 1 : (y == m_cells.ysize() - 1 ? -1 : 0);
        int src_dir_z = (z == 0) ? 1 : (z == m_cells.zsize() - 1 ? -1 : 0);
        c.dir = dirnum(src_dir_x, src_dir_y, src_dir_z, 1);
        updateVertex(&c);
    };

    // initialize border cells
    for (int y = 0; y < m_cells.ysize(); ++y) {
    for (int z = 0; z < m_cells.zsize(); ++z) {
        init_obs_cell(0, y, z);
        init_obs_cell(m_cells.xsize() - 1, y, z);
    }
    }
    for (int x = 1; x < m_cells.xsize() - 1; ++x) {
    for (int z = 0; z < m_cells.zsize(); ++z) {
        init_obs_cell(x, 0, z);
        init_obs_cell(x, m_cells.ysize() - 1, z);
    }
    }
    for (int x = 1; x < m_cells.xsize() - 1; ++x) {
    for (int y = 1; y < m_cells.ysize() - 1; ++y) {
        init_obs_cell(x, y, 0);
        init_obs_cell(x, y, m_cells.zsize() - 1);
    }
    }
}

void DistanceMapBase::updateVertex(Cell* o)
{
    const int key = std::min(o->dist, o->dist_new);
    assert(key < m_open.size());
    if (o->bucket >= 0) { // in heap
        assert(o->bucket < m_open.size());
        BUCKET_UPDATE(o, key);
    } else { // not in the heap yet
        BUCKET_INSERT(o, key);
    }
    if (key < m_bucket) {
        m_bucket = key;
    }
}

} // namespace sbpl
