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

#ifndef SMPL_DISTANCE_MAP_HPP
#define SMPL_DISTANCE_MAP_HPP

#include "../distance_map.h"

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

template <typename Derived>
DistanceMap<Derived>::DistanceMap(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double resolution,
    double max_dist)
:
    DistanceMapBase(
        origin_x, origin_y, origin_z,
        size_x, size_y, size_z,
        resolution,
        max_dist)
{
    propagateBorder();
}

/// Add a set of obstacle points to the distance map and update the distance
/// values of affected cells. Points outside the map and cells that are already
/// marked as obstacles will be ignored.
template <typename Derived>
void DistanceMap<Derived>::addPointsToMap(
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
template <typename Derived>
void DistanceMap<Derived>::removePointsFromMap(
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
template <typename Derived>
void DistanceMap<Derived>::updatePointsInMap(
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
template <typename Derived>
void DistanceMap<Derived>::reset()
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

template <typename Derived>
int DistanceMap<Derived>::distance(const Cell& n, const Cell& s)
{
    return static_cast<Derived*>(this)->distance(n, s);
}

template <typename Derived>
void DistanceMap<Derived>::lower(Cell* s)
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

template <typename Derived>
void DistanceMap<Derived>::raise(Cell* s)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[m_no_update_dir];
    for (int i = nfirst; i != nlast; ++i) {
        Cell* n = s + m_neighbor_offsets[i];
        waveout(n);
    }
    waveout(s);
}

template <typename Derived>
void DistanceMap<Derived>::waveout(Cell* n)
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

template <typename Derived>
void DistanceMap<Derived>::propagate()
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

template <typename Derived>
void DistanceMap<Derived>::lowerBounded(Cell* s)
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

template <typename Derived>
void DistanceMap<Derived>::propagateBorder()
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

template <typename Derived>
DistanceMapMoveIt<Derived>::DistanceMapMoveIt(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double resolution,
    double max_dist)
:
    DistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z),
    m_dm(origin_x, origin_y, origin_z, size_x, size_y, size_z, resolution, max_dist)
{
}

template <typename Derived>
void DistanceMapMoveIt<Derived>::addPointsToField(
    const EigenSTL::vector_Vector3d& points)
{
    std::vector<Eigen::Vector3d> pts(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pts[i] = points[i];
    }

    m_dm.addPointsToMap(pts);
}

template <typename Derived>
void DistanceMapMoveIt<Derived>::removePointsFromField(
    const EigenSTL::vector_Vector3d& points)
{
    std::vector<Eigen::Vector3d> pts(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pts[i] = points[i];
    }

    m_dm.removePointsFromMap(pts);
}

template <typename Derived>
void DistanceMapMoveIt<Derived>::updatePointsInField(
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

template <typename Derived>
void DistanceMapMoveIt<Derived>::reset()
{
    m_dm.reset();
}

template <typename Derived>
double DistanceMapMoveIt<Derived>::getDistance(double x, double y, double z) const
{
    return m_dm.getDistance(x, y, z);
}

template <typename Derived>
double DistanceMapMoveIt<Derived>::getDistance(int x, int y, int z) const
{
    return m_dm.getDistance(x, y, z);
}

template <typename Derived>
bool DistanceMapMoveIt<Derived>::isCellValid(int x, int y, int z) const
{
    return m_dm.isCellValid(x, y, z);
}

template <typename Derived>
int DistanceMapMoveIt<Derived>::getXNumCells() const
{
    return m_dm.numCellsX();
}

template <typename Derived>
int DistanceMapMoveIt<Derived>::getYNumCells() const
{
    return m_dm.numCellsY();
}

template <typename Derived>
int DistanceMapMoveIt<Derived>::getZNumCells() const
{
    return m_dm.numCellsZ();
}

template <typename Derived>
bool DistanceMapMoveIt<Derived>::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    m_dm.gridToWorld(x, y, z, world_x, world_y, world_z);
    return m_dm.isCellValid(x, y, z);
}

template <typename Derived>
bool DistanceMapMoveIt<Derived>::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    m_dm.worldToGrid(world_x, world_y, world_z, x, y, z);
    return m_dm.isCellValid(x, y, z);
}

template <typename Derived>
bool DistanceMapMoveIt<Derived>::writeToStream(std::ostream& stream) const
{
    return false;
}

template <typename Derived>
bool DistanceMapMoveIt<Derived>::readFromStream(std::istream& stream)
{
    return false;
}

template <typename Derived>
double DistanceMapMoveIt<Derived>::getUninitializedDistance() const
{
    return m_dm.maxDistance();
}

} // namespace sbpl

#endif
