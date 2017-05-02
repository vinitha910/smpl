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

#include <smpl/distance_map/sparse_distance_map.h>

// standard includes
#include <set>

namespace sbpl {

SparseDistanceMap::SparseDistanceMap(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double resolution,
    double max_dist)
:
    DistanceMapInterface(
            origin_x, origin_y, origin_z,
            size_x, size_y, size_z,
            resolution),
    m_cells(),
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
    // init neighbors for forward propagation
    CreateNeighborUpdateList(m_neighbors, m_indices, m_neighbor_ranges);

    for (size_t i = 0; i < m_indices.size(); ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];
        m_neighbor_offsets[i] = 0;
        m_neighbor_offsets[i] += neighbor.x() * m_cell_count_z * m_cell_count_y;
        m_neighbor_offsets[i] += neighbor.y() * m_cell_count_z;
        m_neighbor_offsets[i] += neighbor.z() * 1;

        if (i < NON_BORDER_NEIGHBOR_LIST_SIZE) {
            m_neighbor_dirs[i] = dirnum(neighbor.x(), neighbor.y(), neighbor.z());
        } else {
            m_neighbor_dirs[i] = dirnum(neighbor.x(), neighbor.y(), neighbor.z(), 1);
        }
    }

    m_cell_count_x = (int)(size_x * m_inv_res + 0.5);
    m_cell_count_y = (int)(size_y * m_inv_res + 0.5);
    m_cell_count_z = (int)(size_z * m_inv_res + 0.5);

    m_cells.resize(m_cell_count_x, m_cell_count_y, m_cell_count_z);

    m_open.resize(m_dmax_sqrd_int + 1);

    m_sqrt_table.resize(m_dmax_sqrd_int + 1, 0.0);
    for (int i = 0; i < m_dmax_sqrd_int + 1; ++i) {
        m_sqrt_table[i] = m_res * std::sqrt((double)i);
    }

    reset();
}

/// Return the distance value for an invalid cell.
double SparseDistanceMap::maxDistance() const
{
    return m_max_dist;
}

/// Return the distance of a cell from its nearest obstacle. This function will
/// also consider the distance to the nearest border cell. A value of 0.0 is
/// returned for obstacle cells and cells outside of the bounding volume.
double SparseDistanceMap::getDistance(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return getDistance(gx, gy, gz);
}

/// Return the distance of a cell from its nearest obstacle cell. This function
/// will also consider the distance to the nearest border cell. A value of 0.0
/// is returned for obstacle cells and cells outside of the bounding volume.
double SparseDistanceMap::getDistance(int x, int y, int z) const
{
    if (!isCellValid(x, y, z)) {
        return 0.0;
    }

    int d2 = m_cells.get(x, y, z).dist;
    return m_sqrt_table[d2];
}

bool SparseDistanceMap::isCellValid(const Eigen::Vector3i& gp) const
{
    return gp.x() >= 0 & gp.x() < m_cell_count_x &
        gp.y() >= 0 & gp.y() < m_cell_count_y &
        gp.z() >= 0 & gp.z() < m_cell_count_z;
}

DistanceMapInterface* SparseDistanceMap::clone() const
{
    return new SparseDistanceMap(*this);
}

/// Add a set of obstacle points to the distance map and update the distance
/// values of affected cells. Points outside the map and cells that are already
/// marked as obstacles will be ignored.
void SparseDistanceMap::addPointsToMap(
    const std::vector<Eigen::Vector3d>& points)
{
    for (const Eigen::Vector3d& p : points) {
        int gx, gy, gz;
        worldToGrid(p.x(), p.y(), p.z(), gx, gy, gz);
        if (!isCellValid(gx, gy, gz)) {
            continue;
        }

        Cell& c = m_cells(gx, gy, gz); // force stable
        if (c.dist_new > 0) {
            c.dir = m_no_update_dir;
            c.dist_new = 0;
            c.obs = &c;
            c.ox = gx;
            c.oy = gy;
            c.oz = gz;
            updateVertex(&c, gx, gy, gz);
        }
    }

    propagate();
}

/// Remove a set of obstacle points from the distance map and update the
/// distance values of affected cells. Points outside the map and cells that
/// are already marked as obstacles will be ignored.
void SparseDistanceMap::removePointsFromMap(
    const std::vector<Eigen::Vector3d>& points)
{
    for (const Eigen::Vector3d& p : points) {
        int gx, gy, gz;
        worldToGrid(p.x(), p.y(), p.z(), gx, gy, gz);
        if (!isCellValid(gx, gy, gz)) {
            continue;
        }

        Cell& c = m_cells(gx, gy, gz); // force stable

        if (c.obs != &c) {
            continue;
        }

        c.dist_new = m_dmax_sqrd_int;
        c.obs = nullptr;
        c.ox = c.oy = c.oz = -1;

        c.dist = m_dmax_sqrd_int;
        c.dir = m_no_update_dir;
        m_rem_stack.emplace_back(&c, gx, gy, gz);
    }

    propagateRemovals();
}

/// Add the set (new_points - old_points) of obstacle cells and remove the set
/// (old_points - new_points) of obstacle cells and update the distance values
/// of affected cells. Points outside the map will be ignored.
void SparseDistanceMap::updatePointsInMap(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    std::set<Eigen::Vector3i, Eigen_Vector3i_compare> old_point_set;
    for (const Eigen::Vector3d& wp : old_points) {
        Eigen::Vector3i gp;
        worldToGrid(wp.x(), wp.y(), wp.z(), gp.x(), gp.y(), gp.z());
        if (isCellValid(gp.x(), gp.y(), gp.z())) {
            old_point_set.insert(gp);
        }
    }

    std::set<Eigen::Vector3i, Eigen_Vector3i_compare> new_point_set;
    for (const Eigen::Vector3d& wp : new_points) {
        Eigen::Vector3i gp;
        worldToGrid(wp.x(), wp.y(), wp.z(), gp.x(), gp.y(), gp.z());
        if (isCellValid(gp.x(), gp.y(), gp.z())) {
            new_point_set.insert(gp);
        }
    }

    Eigen_Vector3i_compare comp;

    std::vector<Eigen::Vector3i> old_not_new;
    std::set_difference(
            old_point_set.begin(), old_point_set.end(),
            new_point_set.begin(), new_point_set.end(),
            std::inserter(old_not_new, old_not_new.end()),
            comp);

    std::vector<Eigen::Vector3i> new_not_old;
    std::set_difference(
            new_point_set.begin(), new_point_set.end(),
            old_point_set.begin(), old_point_set.end(),
            std::inserter(new_not_old, new_not_old.end()),
            comp);

    // remove obstacle cells that were in the old cloud but not the new cloud
    for (const Eigen::Vector3i& p : old_not_new) {
        Cell& c = m_cells(p.x(), p.y(), p.z()); // force stable
        if (c.obs != &c) {
            continue; // skip already-free cells
        }
        c.dir = m_no_update_dir;
        c.dist_new = m_dmax_sqrd_int;
        c.dist = m_dmax_sqrd_int;
        c.obs = nullptr;
        c.ox = c.oy = c.oz = -1;
        m_rem_stack.emplace_back(&c, p.x(), p.y(), p.z());
    }

    propagateRemovals();

    // add obstacle cells that are in the new cloud but not the old cloud
    for (const Eigen::Vector3i& p : new_not_old) {
        Cell& c = m_cells(p.x(), p.y(), p.z()); // force stable
        if (c.dist_new == 0) {
            continue; // skip already-obstacle cells
        }
        c.dir = m_no_update_dir;
        c.dist_new = 0;
        c.obs = &c;
        c.ox = p.x();
        c.oy = p.y();
        c.oz = p.z();

        updateVertex(&c, p.x(), p.y(), p.z());
    }

    propagate();
}

/// Reset all points in the distance map to their uninitialized (free) values.
void SparseDistanceMap::reset()
{
    Cell initial;
    initial.dist = m_dmax_sqrd_int;
    initial.dist_new = m_dmax_sqrd_int;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
    initial.dist_old = m_dmax_sqrd_int;
#endif
    initial.obs = nullptr;
    initial.ox = initial.oy = initial.oz = -1;

    initial.bucket = -1;
    initial.dir = m_no_update_dir;

    m_cells.reset(initial);
}

/// Return the number of cells along the x axis.
int SparseDistanceMap::numCellsX() const
{
    return m_cell_count_x;
}

/// Return the number of cells along the y axis.
int SparseDistanceMap::numCellsY() const
{
    return m_cell_count_y;
}

/// Return the number of cells along the z axis.
int SparseDistanceMap::numCellsZ() const
{
    return m_cell_count_z;
}

double SparseDistanceMap::getUninitializedDistance() const
{
    return m_max_dist;
}

double SparseDistanceMap::getMetricDistance(double x, double y, double z) const
{
    return getDistance(x, y, z);
}

double SparseDistanceMap::getCellDistance(int x, int y, int z) const
{
    return getDistance(x, y, z);
}

/// Return the effective grid coordinates of the cell containing the given point
/// specified in world coordinates.
void SparseDistanceMap::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    world_x = (m_origin_x) + (x) * m_res;
    world_y = (m_origin_y) + (y) * m_res;
    world_z = (m_origin_z) + (z) * m_res;
}

/// Return the point in world coordinates marking the center of the cell at the
/// given effective grid coordinates.
void SparseDistanceMap::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    x = (int)(m_inv_res * (world_x - m_origin_x) + 0.5);
    y = (int)(m_inv_res * (world_y - m_origin_y) + 0.5);
    z = (int)(m_inv_res * (world_z - m_origin_z) + 0.5);
}

/// Test if a cell is outside the bounding volume.
bool SparseDistanceMap::isCellValid(int x, int y, int z) const
{
    return x >= 0 & x < m_cell_count_x &
        y >= 0 & y < m_cell_count_y &
        z >= 0 & z < m_cell_count_z;
}

void SparseDistanceMap::updateVertex(Cell* o, int cx, int cy, int cz)
{
    const int key = std::min(o->dist, o->dist_new);
    assert(key < m_open.size());
    if (o->bucket >= 0) { // update in heap
        assert(o->bucket < m_open.size());

        // swap places with last element and remove from end of current bucket
        bucket_element &e = m_open[o->bucket][o->pos];
        e = m_open[o->bucket].back();
        e.c->pos = o->pos;
        m_open[o->bucket].pop_back();

        // place at the end of new bucket
        o->pos = m_open[key].size();
        m_open[key].emplace_back(o, cx, cy, cz);
        o->bucket = key;
    } else { // not in the heap yet
        // place at the end of new bucket
        o->pos = m_open[key].size();
        m_open[key].emplace_back(o, cx, cy, cz);
        o->bucket = key;
    }
    if (key < m_bucket) {
        m_bucket = key;
    }
}

int SparseDistanceMap::distance(int nx, int ny, int nz, const Cell& s)
{
    int dx = nx - s.ox;
    int dy = ny - s.oy;
    int dz = nz - s.oz;

    return dx * dx + dy * dy + dz * dz;
}

void SparseDistanceMap::lower(Cell* s, int sx, int sy, int sz)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[s->dir];
    for (int i = nfirst; i != nlast; ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];
        const Eigen::Vector3i& nx = Eigen::Vector3i(sx, sy, sz) + neighbor;
        if (!isCellValid(nx)) {
            continue;
        }

        Cell* n = &m_cells(nx.x(), nx.y(), nx.z()); // force stable
//        if (n->dist_new > s->dist_new)
        {
            int dp = distance(nx.x(), nx.y(), nx.z(), *s);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = s->obs;
                n->ox = s->ox;
                n->oy = s->oy;
                n->oz = s->oz;
                n->dir = m_neighbor_dirs[i];
                updateVertex(n, nx.x(), nx.y(), nx.z());
            }
        }
    }
}

void SparseDistanceMap::raise(Cell* s, int sx, int sy, int sz)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[m_no_update_dir];
    for (int i = nfirst; i != nlast; ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];
        const Eigen::Vector3i& nx = Eigen::Vector3i(sx, sy, sz) + neighbor;
        if (!isCellValid(nx)) {
            continue;
        }
        Cell* n = &m_cells(nx.x(), nx.y(), nx.z()); // force stable
        waveout(n, nx.x(), nx.y(), nx.z());
    }
    waveout(s, sx, sy, sz);
}

void SparseDistanceMap::waveout(Cell* n, int nx, int ny, int nz)
{
    if (n == n->obs) {
        return;
    }

    n->dist_new = m_dmax_sqrd_int;
    Cell* obs_old = n->obs;
    n->obs = nullptr;
    n->ox = n->oy = n->oz = -1; // TODO(Andrew: required?)

    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[m_no_update_dir];
    for (int i = nfirst; i != nlast; ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];
        const Eigen::Vector3i& ax = Eigen::Vector3i(nx, ny, nz) + neighbor;
        if (!isCellValid(ax)) {
            continue;
        }
        Cell* a = &m_cells(ax.x(), ax.y(), ax.z()); // force stable
        auto valid = [](Cell* c) { return c && c->obs == c; };
        if (valid(a->obs)) {
            int dp = distance(nx, ny, nz, *a);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = a->obs;
                n->ox = a->ox;
                n->oy = a->oy;
                n->oz = a->oz;
                n->dir = m_no_update_dir;
            }
        }
    }

    if (n->obs != obs_old) {
//        n->dir = m_no_update_dir;
        updateVertex(n, nx, ny, nz);
    }
}

void SparseDistanceMap::propagate()
{
    while (m_bucket < (int)m_open.size()) {
        while (!m_open[m_bucket].empty()) {
            assert(m_bucket >= 0 && m_bucket < m_open.size());

            bucket_element e = m_open[m_bucket].back();
            m_open[m_bucket].pop_back();
            Cell* s = e.c;
            s->bucket = -1;

            if (s->dist_new < s->dist) {
                s->dist = s->dist_new;

                // foreach n in adj(min)
                lower(s, e.x, e.y, e.z);

#if SMPL_DMAP_RETURN_CHANGED_CELLS
                if (s->dist != s->dist_old) {
                    // insert(C, s)
                    s->dist_old = s->dist;
                }
#endif
            } else {
                s->dist = m_dmax_sqrd_int;
                s->dir = m_no_update_dir;
                raise(s, e.x, e.y, e.z);
                if (s->dist != s->dist_new) {
                    updateVertex(s, e.x, e.y, e.z);
                }
            }
        }
        ++m_bucket;
    }
}

void SparseDistanceMap::lowerBounded(Cell* s, int sx, int sy, int sz)
{
    return lower(s, sx, sy, sz);
}

void SparseDistanceMap::propagateRemovals()
{
    while (!m_rem_stack.empty()) {
        bucket_element e = m_rem_stack.back();
        Cell* s = e.c;
        m_rem_stack.pop_back();

        int nfirst, nlast;
        std::tie(nfirst, nlast) = m_neighbor_ranges[m_no_update_dir];
        for (int i = nfirst; i != nlast; ++i) {
            const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];
            const Eigen::Vector3i& nx = Eigen::Vector3i(e.x, e.y, e.z) + neighbor;
            if (!isCellValid(nx)) {
                continue;
            }
            Cell* n = &m_cells(nx.x(), nx.y(), nx.z()); // force stable
            auto valid = [](Cell* c) { return c && c->obs == c; };
            if (!valid(n->obs)) {
                if (n->dist_new != m_dmax_sqrd_int) {
                    n->dist_new = m_dmax_sqrd_int;
                    n->dist = m_dmax_sqrd_int;
                    n->obs = nullptr;
                    n->ox = n->oy = n->oz = -1;
                    n->dir = m_no_update_dir;
                    m_rem_stack.emplace_back(n, nx.x(), nx.y(), nx.z());
                }
            } else {
                updateVertex(n, nx.x(), nx.y(), nx.z());
            }
        }
    }

    // see note in Cell::operator==
    m_cells.prune([&](const Cell& c) { return !c.obs; });

    propagateBorder();
}

void SparseDistanceMap::propagateBorder()
{
    while (m_bucket < (int)m_open.size()) {
        while (!m_open[m_bucket].empty()) {
            assert(m_bucket >= 0 && m_bucket < m_open.size());
            bucket_element e = m_open[m_bucket].back();
            m_open[m_bucket].pop_back();
            Cell* s = e.c;
            s->bucket = -1;

//            if (s->dist_new < s->dist)
            {
                assert(s->dist_new < s->dist);
                s->dist = s->dist_new;

                // foreach n in adj(min)
                lowerBounded(s, e.x, e.y, e.z);

#if SMPL_DMAP_RETURN_CHANGED_CELLS
                if (s->dist != s->dist_old) {
                    // insert(C, s)
                    s->dist_old = s->dist;
                }
#endif
            }
        }
        ++m_bucket;
    }
}

} // namespace sbpl
