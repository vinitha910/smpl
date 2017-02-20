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

#include <smpl/geometry/distance_map.h>

// standard includes
#include <cmath>
#include <algorithm>
#include <set>

namespace sbpl {

struct Eigen_Vector3i_compare
{
    bool operator()(const Eigen::Vector3i& u, const Eigen::Vector3i& v)
    {
        return std::tie(u.x(), u.y(), u.z()) < std::tie(v.x(), v.y(), v.z());
    }
};

void CreateNeighborUpdateList(
    std::array<Eigen::Vector3i, 27>& neighbors,
    std::array<int, NEIGHBOR_LIST_SIZE>& indices,
    std::array<std::pair<int, int>, NUM_DIRECTIONS>& ranges);

inline
int dirnum(int dx, int dy, int dz, int edge = 0)
{
    return 27 * edge + 9 * (dx + 1) + 3 * (dy + 1) + (dz + 1);
}

#define VECTOR_BUCKET_LIST_INSERT(o, key) \
{\
    o->pos = m_open[key].size();\
    m_open[key].push_back(o);\
    o->bucket = key;\
}

#define VECTOR_BUCKET_LIST_UPDATE(o, key) \
{\
    m_open[o->bucket][o->pos] = m_open[o->bucket].back();\
\
    m_open[o->bucket][o->pos]->pos = o->pos;\
    m_open[o->bucket].pop_back();\
\
    o->pos = m_open[key].size();\
    m_open[key].push_back(o);\
    o->bucket = key;\
}

#define VECTOR_BUCKET_LIST_POP(s, b) \
{\
    s = m_open[b].back();\
    m_open[b].pop_back();\
    s->bucket = -1;\
}

#define BUCKET_INSERT(o, key) do { VECTOR_BUCKET_LIST_INSERT(o, key) } while (0)
#define BUCKET_UPDATE(o, key) do { VECTOR_BUCKET_LIST_UPDATE(o, key) } while (0)
#define BUCKET_POP(s, bucket) do { VECTOR_BUCKET_LIST_POP(s, bucket) } while (0)

void CreateNeighborUpdateList(
    std::array<Eigen::Vector3i, 27>& neighbors,
    std::array<int, NEIGHBOR_LIST_SIZE>& indices,
    std::array<std::pair<int, int>, NUM_DIRECTIONS>& ranges)
{
    int i = 0;
    int n = 0;
    for (int edge = 0; edge < 2; ++edge) {
    for (int sx = -1; sx <= 1; ++sx) {
    for (int sy = -1; sy <= 1; ++sy) {
    for (int sz = -1; sz <= 1; ++sz) {
        if (!edge) {
            neighbors[n++] = Eigen::Vector3i(sx, sy, sz);
        }
        int d = dirnum(sx, sy, sz, edge);
        int nfirst = i;
        for (int tx = -1; tx <= 1; ++tx) {
        for (int ty = -1; ty <= 1; ++ty) {
        for (int tz = -1; tz <= 1; ++tz) {
            if (tx == 0 && ty == 0 && tz == 0) {
                continue;
            }
            if (edge) {
                if (
                    !(((tx == -1 && sx == 1) || (tx == 1 & sx == -1)) ||
                    ((ty == -1 && sy == 1) || (ty == 1 & sy == -1)) ||
                    ((tz == -1 && sz == 1) || (tz == 1 & sz == -1))))
                {
                    indices[i++] = dirnum(tx, ty, tz);
                }
            } else {
                if (tx * sx + ty * sy + tz * sz >= 0) {
                    indices[i++] = dirnum(tx, ty, tz);
                }
            }
        }
        }
        }
        int nlast = i;
        ranges[d].first = nfirst;
        ranges[d].second = nlast;
    }
    }
    }
    }
}

DistanceMap::DistanceMap(
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

    propagateBorder();
}

/// Return the size along the x axis of the bounding volume.
double DistanceMap::sizeX() const
{
    return m_size_x;
}

/// Return the size along the y axis of the bounding volume.
double DistanceMap::sizeY() const
{
    return m_size_y;
}

/// Return the size along the z axis of the bounding volume.
double DistanceMap::sizeZ() const
{
    return m_size_z;
}

/// Return the origin along x axis.
double DistanceMap::originX() const
{
    return m_origin_x;
}

/// Return the origin along the y axis.
double DistanceMap::originY() const
{
    return m_origin_y;
}

/// Return the origin along the z axis.
double DistanceMap::originZ() const
{
    return m_origin_z;
}

/// Return the resolution of the distance map.
double DistanceMap::resolution() const
{
    return m_res;
}

/// Return the distance value for an invalid cell.
double DistanceMap::maxDistance() const
{
    return m_max_dist;
}

/// Return the number of cells along the x axis.
int DistanceMap::numCellsX() const
{
    return m_cells.xsize() - 2;
}

/// Return the number of cells along the y axis.
int DistanceMap::numCellsY() const
{
    return m_cells.ysize() - 2;
}

/// Return the number of cells along the z axis.
int DistanceMap::numCellsZ() const
{
    return m_cells.zsize() - 2;
}

/// Return the effective grid coordinates of the cell containing the given point
/// specified in world coordinates.
void DistanceMap::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    world_x = (m_origin_x - m_res) + (x + 1) * m_res;
    world_y = (m_origin_y - m_res) + (y + 1) * m_res;
    world_z = (m_origin_z - m_res) + (z + 1) * m_res;
}

/// Return the point in world coordinates marking the center of the cell at the
/// given effective grid coordinates.
void DistanceMap::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    x = (int)(m_inv_res * (world_x - (m_origin_x - m_res)) + 0.5) - 1;
    y = (int)(m_inv_res * (world_y - (m_origin_y - m_res)) + 0.5) - 1;
    z = (int)(m_inv_res * (world_z - (m_origin_z - m_res)) + 0.5) - 1;
}

/// Test if a cell is outside the bounding volume.
bool DistanceMap::isCellValid(int x, int y, int z) const
{
    return x >= 0 && x < m_cells.xsize() - 2 &&
        y >= 0 && y < m_cells.ysize() - 2 &&
        z >= 0 && z < m_cells.zsize() - 2;
}

/// Return the distance of a cell from its nearest obstacle. This function will
/// also consider the distance to the nearest border cell. A value of 0.0 is
/// returned for obstacle cells and cells outside of the bounding volume.
double DistanceMap::getDistance(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return getDistance(gx, gy, gz);
}

/// Return the distance of a cell from its nearest obstacle cell. This function
/// will also consider the distance to the nearest border cell. A value of 0.0
/// is returned for obstacle cells and cells outside of the bounding volume.
double DistanceMap::getDistance(int x, int y, int z) const
{
    if (!isCellValid(x, y, z)) {
        return 0.0;
    }

    return sqrt(m_res * m_cells(x + 1, y + 1, z + 1).dist);
}

/// Add a set of obstacle points to the distance map and update the distance
/// values of affected cells. Points outside the map and cells that are already
/// marked as obstacles will be ignored.
void DistanceMap::addPointsToMap(
    const std::vector<Eigen::Vector3d>& points)
{
    for (const Eigen::Vector3d& p : points) {
        int gx, gy, gz;
        worldToGrid(p.x(), p.y(), p.z(), gx, gy, gz);
        if (!isCellValid(gx, gy, gz)) {
            continue;
        }

        ++gx; ++gy; ++gz;

        Cell& c = m_cells(gx, gy, gz);
        if (c.dist_new > 0) {
            c.dir = m_no_update_dir;
            c.dist_new = 0;
            c.obs = &c;
            updateVertex(&c);
        }
    }

    propagate();
}

/// Remove a set of obstacle points from the distance map and update the
/// distance values of affected cells. Points outside the map and cells that
/// are already marked as obstacles will be ignored.
void DistanceMap::removePointsFromMap(
    const std::vector<Eigen::Vector3d>& points)
{
    for (const Eigen::Vector3d& p : points) {
        int gx, gy, gz;
        worldToGrid(p.x(), p.y(), p.z(), gx, gy, gz);
        if (!isCellValid(gx, gy, gz)) {
            continue;
        }

        ++gx; ++gy; ++gz;

        Cell& c = m_cells(gx, gy, gz);

        if (c.obs != &c) {
            continue;
        }

        c.dist_new = m_dmax_sqrd_int;
        c.obs = nullptr;

        c.dist = m_dmax_sqrd_int;
        c.dir = m_no_update_dir;
        m_rem_stack.push_back(&c);
    }

    while (!m_rem_stack.empty()) {
        Cell* s = m_rem_stack.back();
        m_rem_stack.pop_back();

        int nfirst, nlast;
        std::tie(nfirst, nlast) = m_neighbor_ranges[m_no_update_dir];
        for (int i = nfirst; i != nlast; ++i) {
            Cell* n = s + m_neighbor_offsets[i];
            auto valid = [](Cell* c) { return c && c->obs == c; };
            if (!valid(n->obs)) {
                if (n->dist_new != m_dmax_sqrd_int) {
                    n->dist_new = m_dmax_sqrd_int;
                    n->dist = m_dmax_sqrd_int;
                    n->obs = nullptr;
                    n->dir = m_no_update_dir;
                    m_rem_stack.push_back(n);
                }
            } else {
                updateVertex(n);
            }
        }
    }

    propagateBorder();
}

/// Add the set (new_points - old_points) of obstacle cells and remove the set
/// (old_points - new_points) of obstacle cells and update the distance values
/// of affected cells. Points outside the map will be ignored.
void DistanceMap::updatePointsInMap(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    std::set<Eigen::Vector3i, Eigen_Vector3i_compare> old_point_set;
    for (const Eigen::Vector3d& wp : old_points) {
        Eigen::Vector3i gp;
        worldToGrid(wp.x(), wp.y(), wp.z(), gp.x(), gp.y(), gp.z());
        if (isCellValid(gp.x(), gp.y(), gp.z())) {
            ++gp.x(); ++gp.y(); ++gp.z();
            old_point_set.insert(gp);
        }
    }

    std::set<Eigen::Vector3i, Eigen_Vector3i_compare> new_point_set;
    for (const Eigen::Vector3d& wp : new_points) {
        Eigen::Vector3i gp;
        worldToGrid(wp.x(), wp.y(), wp.z(), gp.x(), gp.y(), gp.z());
        if (isCellValid(gp.x(), gp.y(), gp.z())) {
            ++gp.x(); ++gp.y(); ++gp.z();
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

    std::vector<Eigen::Vector3i> new_not_in_current;
    for (const Eigen::Vector3i& v : new_not_old) {
        const Cell& c = m_cells(v.x(), v.y(), v.z());
        if (c.dist_new > 0) {
            new_not_in_current.push_back(v);
        }
    }

    for (const Eigen::Vector3i& p : old_not_new) {
        Cell& c = m_cells(p.x(), p.y(), p.z());
        c.dist_new = m_dmax_sqrd_int;
        c.obs = nullptr;
        if (c.dist < m_dmax_sqrd_int) {
            updateVertex(&c);
        }
    }

    for (const Eigen::Vector3i& p : new_not_in_current) {
        Cell& c = m_cells(p.x(), p.y(), p.z());
        if (c.dist_new > 0) {
            c.dist_new = 0;
            c.obs = &c;
            updateVertex(&c);
        }
    }

    propagate();
}

/// Reset all points in the distance map to their uninitialized (free) values.
void DistanceMap::reset()
{
    for (size_t x = 1; x < m_cells.xsize() - 1; ++x) {
    for (size_t y = 1; y < m_cells.ysize() - 1; ++y) {
    for (size_t z = 1; z < m_cells.zsize() - 1; ++z) {
        Cell& c = m_cells(x, y, z);
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
    // TODO: need to propagate removed cells to maintain distance to the edge
}

inline
void DistanceMap::updateVertex(Cell* o)
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

inline
int DistanceMap::distance(const Cell& n, const Cell& s) const
{
    return distanceEuclidSqrd(n, s);
//    return distanceEuclidSqrdConservative(n, s);
//    return distanceQuasiEuclid(n, s);
}

inline
int DistanceMap::distanceEuclidSqrd(const Cell& n, const Cell& s) const
{
    int dx = n.x - s.obs->x;
    int dy = n.y - s.obs->y;
    int dz = n.z - s.obs->z;

    return dx * dx + dy * dy + dz * dz;
}

inline
int DistanceMap::distanceEuclidSqrdConservative(
    const Cell& n,
    const Cell& s) const
{
    int dx = n.x - s.obs->x;
    int dy = n.y - s.obs->y;
    int dz = n.z - s.obs->z;

    if (dx > 0) {
        dx -= 1;
    } else if (dx < 0) {
        dx += 1;
    }

    if (dy > 0) {
        dy -= 1;
    } else if (dy < 0) {
        dy += 1;
    }

    if (dz > 0) {
        dz -= 1;
    } else if (dz < 0) {
        dz += 1;
    }

    return dx * dx + dy * dy + dz * dz;
}

inline
int DistanceMap::distanceQuasiEuclid(const Cell& n, const Cell& s) const
{
    int dx = n.x - s.x;
    int dy = n.y - s.y;
    int dz = n.z - s.z;

    return dx * dx + dy * dy + dz * dz + s.dist_new;
}

inline
void DistanceMap::lower(Cell* s)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[s->dir];
    for (int i = nfirst; i != nlast; ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];

        Cell* n = s + m_neighbor_offsets[i];
//        if (n->dist_new > s->dist_new)
        {
            int dp = distance(*n, *s);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = s->obs;
                n->dir = m_neighbor_dirs[i];
                updateVertex(n);
            }
        }
    }
}

inline
void DistanceMap::raise(Cell* s)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[m_no_update_dir];
    for (int i = nfirst; i != nlast; ++i) {
        Cell* n = s + m_neighbor_offsets[i];
        waveout(n);
    }
    waveout(s);
}

inline
void DistanceMap::waveout(Cell* n)
{
    if (n == n->obs) {
        return;
    }

    n->dist_new = m_dmax_sqrd_int;
    Cell* obs_old = n->obs;
    n->obs = nullptr;

    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[m_no_update_dir];
    for (int i = nfirst; i != nlast; ++i) {
        Cell* a = n + m_neighbor_offsets[i];
        auto valid = [](Cell* c) { return c && c->obs == c; };
        if (valid(a->obs)) {
            int dp = distance(*n, *a);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = a->obs;
                n->dir = m_no_update_dir;
            }
        }
    }

    if (n->obs != obs_old) {
//        n->dir = m_no_update_dir;
        updateVertex(n);
    }
}

void DistanceMap::propagate()
{
    int iter_count = 0;
    while (m_bucket < (int)m_open.size()) {
        while (!m_open[m_bucket].empty()) {
            ++iter_count;
            assert(m_bucket >= 0 && m_bucket < m_open.size());
            Cell* s;
            BUCKET_POP(s, m_bucket);

            if (s->dist_new < s->dist) {
                s->dist = s->dist_new;

                // foreach n in adj(min)
                lower(s);

#if SMPL_DMAP_RETURN_CHANGED_CELLS
                if (s->dist != s->dist_old) {
                    // insert(C, s)
                    s->dist_old = s->dist;
                }
#endif
            } else {
                s->dist = m_dmax_sqrd_int;
                s->dir = m_no_update_dir;
                raise(s);
                if (s->dist != s->dist_new) {
                    updateVertex(s);
                }
            }
        }
        ++m_bucket;
    }
}

void DistanceMap::lowerBounded(Cell* s)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[s->dir];
    for (int i = nfirst; i != nlast; ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];

        Cell* n = s + m_neighbor_offsets[i];
        if (n->dist_new > s->dist_new) {
            int dp = distance(*n, *s);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = s->obs;
                n->dir = m_neighbor_dirs[i];
                updateVertex(n);
            }
        }
    }
}

void DistanceMap::propagateBorder()
{
    while (m_bucket < (int)m_open.size()) {
        while (!m_open[m_bucket].empty()) {
            assert(m_bucket >= 0 && m_bucket < m_open.size());
            Cell* s;
            BUCKET_POP(s, m_bucket);

//            if (s->dist_new < s->dist)
            {
                assert(s->dist_new < s->dist);
                s->dist = s->dist_new;

                // foreach n in adj(min)
                lowerBounded(s);

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

DistanceMapMoveIt::DistanceMapMoveIt(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double resolution,
    double max_dist)
:
    DistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z),
    m_dm(origin_x, origin_y, origin_z, size_x, size_y, size_z, resolution, max_dist)
{
}

void DistanceMapMoveIt::addPointsToField(const EigenSTL::vector_Vector3d& points)
{
    std::vector<Eigen::Vector3d> pts(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pts[i] = points[i];
    }

    m_dm.addPointsToMap(pts);
}

void DistanceMapMoveIt::removePointsFromField(
    const EigenSTL::vector_Vector3d& points)
{
    std::vector<Eigen::Vector3d> pts(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pts[i] = points[i];
    }

    m_dm.removePointsFromMap(pts);
}

void DistanceMapMoveIt::updatePointsInField(
    const EigenSTL::vector_Vector3d& old_points,
    const EigenSTL::vector_Vector3d& new_points)
{
    std::vector<Eigen::Vector3d> old_pts(old_points.size());
    for (size_t i = 0; i < old_points.size(); ++i) {
        old_pts[i] = old_points[i];
    }

    std::vector<Eigen::Vector3d> new_pts(new_points.size());
    for (size_t i = 0; i < new_points.size(); ++i) {
        new_pts[i] = new_points[i];
    }
    m_dm.updatePointsInMap(old_pts, new_pts);
}

void DistanceMapMoveIt::reset()
{
    m_dm.reset();
}

double DistanceMapMoveIt::getDistance(double x, double y, double z) const
{
    return m_dm.getDistance(x, y, z);
}

double DistanceMapMoveIt::getDistance(int x, int y, int z) const
{
    return m_dm.getDistance(x, y, z);
}

bool DistanceMapMoveIt::isCellValid(int x, int y, int z) const
{
    return m_dm.isCellValid(x, y, z);
}

int DistanceMapMoveIt::getXNumCells() const
{
    return m_dm.numCellsX();
}

int DistanceMapMoveIt::getYNumCells() const
{
    return m_dm.numCellsY();
}

int DistanceMapMoveIt::getZNumCells() const
{
    return m_dm.numCellsZ();
}

bool DistanceMapMoveIt::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    m_dm.gridToWorld(x, y, z, world_x, world_y, world_z);
    return m_dm.isCellValid(x, y, z);
}

bool DistanceMapMoveIt::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    m_dm.worldToGrid(world_x, world_y, world_z, x, y, z);
    return m_dm.isCellValid(x, y, z);
}

bool DistanceMapMoveIt::writeToStream(std::ostream& stream) const
{
    return false;
}

bool DistanceMapMoveIt::readFromStream(std::istream& stream)
{
    return false;
}

double DistanceMapMoveIt::getUninitializedDistance() const
{
    return m_dm.maxDistance();
}

} // namespace sbpl
