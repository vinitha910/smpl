////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

// standard includes
#include <sstream>

// system includes
#include <leatherman/print.h>

// project includes
#include <sbpl_collision_checking/base_collision_models.h>

namespace sbpl {
namespace collision {

namespace sphere_traits {

template <typename Sphere> struct radius { };
template <typename Sphere> struct center_x { };
template <typename Sphere> struct center_y { };
template <typename Sphere> struct center_z { };

template <>
struct radius<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.radius;
    }
};

template <>
struct radius<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.radius;
    }
};

template <>
struct center_x<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.x;
    }
};

template <>
struct center_x<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.center.x();
    }
};

template <>
struct center_y<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.y;
    }
};

template <>
struct center_y<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.center.y();
    }
};

template <>
struct center_z<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.z;
    }
};

template <>
struct center_z<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.center.z();
    }
};

} // namespace sphere_traits

template <typename Sphere>
inline double get_radius(const Sphere& sphere)
{
    return sphere_traits::radius<Sphere>::get(sphere);
}

template <typename Sphere>
inline double get_x(const Sphere& sphere)
{
    return sphere_traits::center_x<Sphere>::get(sphere);
}

template <typename Sphere>
inline double get_y(const Sphere& sphere)
{
    return sphere_traits::center_y<Sphere>::get(sphere);
}

template <typename Sphere>
inline double get_z(const Sphere& sphere)
{
    return sphere_traits::center_z<Sphere>::get(sphere);
}

std::ostream& operator<<(std::ostream& o, const CollisionSphereModel& csm)
{
    o << "{ name: " << csm.name << ", center: (" << csm.center.x() << ", " <<
            csm.center.y() << ", " << csm.center.z() << "), radius: " <<
            csm.radius << ", priority: " << csm.priority << " }";
    return o;
}

CollisionSphereTree::CollisionSphereTree() :
    m_tree()
{
}

void CollisionSphereTree::buildFrom(
    const std::vector<CollisionSphereConfig>& spheres)
{
    // create array of points
    std::vector<const CollisionSphereConfig*> sptrs(spheres.size());
    for (size_t i = 0; i < spheres.size(); ++i) {
        sptrs[i] = &spheres[i];
    }

    std::vector<int> parent_indices(sptrs.size());
    buildRecursive<CollisionSphereConfig>(sptrs.begin(), sptrs.end());
    const CollisionSphereModel* root = &m_tree[0];
    for (size_t i = 0; i < m_tree.size(); ++i) {
        CollisionSphereModel& sphere = m_tree[i];
        sphere.left = &m_tree[reinterpret_cast<size_t>(sphere.left)];
        sphere.right = &m_tree[reinterpret_cast<size_t>(sphere.right)];
    }

//    // queue of (sphere, parent, right?) tuples
//    std::queue<std::tuple<CollisionSphereModel*, CollisionSphereModel*, bool>> q;
//
//    // compute total sphere count
//    size_t count = 0;
//    q.push(std::make_tuple(root, nullptr, false));
//    while (!q.empty()) {
//        CollisionSphereModel* s;
//        CollisionSphereModel* p;
//        bool right;
//        std::tie(s, p, right) = q.front();
//        q.pop();
//        ++count;
//
//        if (s->left) {
//            q.push(std::make_tuple(s->left, s, false));
//        }
//        if (s->right) {
//            q.push(std::make_tuple(s->right, s, true));
//        }
//    }
//
//    ROS_WARN("Created %zu total spheres", count);
//
//    // slerp tree into contiguous array
//    std::vector<CollisionSphereModel> spheres(count);
//    q.push(std::make_tuple(root, nullptr, false));
//    size_t sidx = 0;
//    while (!q.empty()) {
//        CollisionSphereModel* sphere;
//        CollisionSphereModel* parent;
//        bool right;
//        std::tie(sphere, parent, right) = q.front();
//
//        q.pop();
//
//        spheres[sidx] = *sphere;
//        if (parent) {
//            if (right) {
//                parent->right = &spheres[sidx];
//            }
//            else {
//                parent->left = &spheres[sidx];
//            }
//        }
//
//        if (sphere->left) {
//            q.push(std::make_tuple(sphere->left, &spheres[sidx], false));
//        }
//        if (sphere->right) {
//            q.push(std::make_tuple(sphere->right, &spheres[sidx], true));
//        }
//    }
//
//    struct SphereTreeDeleter
//    {
//        void del(CollisionSphereModel* s)
//        {
//            ROS_INFO("Delete %p", s);
//            if (s->left) {
//                del(s->left);
//            }
//            if (s->right) {
//                del(s->right);
//            }
//            delete s;
//        }
//    };
//
//    // delete the temporary sphere tree
//    SphereTreeDeleter().del(root);
//
//    ospheres = std::move(spheres);
}

/// \brief Compute the bounding sphere tree for a subset of model spheres
/// \return The index into \p m_tree where the top-most bounding sphere was stored
template <typename Sphere>
size_t CollisionSphereTree::buildRecursive(
    typename std::vector<const Sphere*>::iterator msfirst,
    typename std::vector<const Sphere*>::iterator mslast)
{
    if (mslast == msfirst) {
        ROS_INFO("Zero spheres base case");
        return -1;
    }

    const auto count = std::distance(msfirst, mslast);
    if (count == 1) {
        m_tree.emplace_back();
        CollisionSphereModel& cs = m_tree.back();
        const Sphere& s = **msfirst;
        cs.name = s.name; // ok, no i'm not making traits for these
        cs.center = Eigen::Vector3d(get_x(s), get_y(s), get_z(s));
        cs.radius = get_radius(s);
        cs.priority = s.priority; // ...or this
        reinterpret_cast<size_t&>(cs.left) = 0;
        reinterpret_cast<size_t&>(cs.right) = 0;
        ROS_INFO("Leaf sphere '%s' %p", cs.name.c_str(), &cs);
        const size_t this_idx = m_tree.size() - 1;
        return this_idx;
    }

    ROS_INFO("Building hierarchy from %zu spheres", count);

    // compute the largest axis of the bounding box along which to split
    const int split_axis = computeLargestBoundingBoxAxis<Sphere>(msfirst, mslast);
    ROS_INFO("Splitting along axis %d", split_axis);

    // compute the average centroid of all spheres at this level
    Eigen::Vector3d model_bounding_sphere_center = Eigen::Vector3d::Zero();
    for (auto it = msfirst; it != mslast; ++it) {
        const Sphere& s = **it;
        model_bounding_sphere_center += Eigen::Vector3d(get_x(s), get_y(s), get_z(s));
    }
    model_bounding_sphere_center /= count;

    // compute the radius required to encompass all model spheres at this level
    double model_bounding_sphere_radius = 0.0;
    for (auto it = msfirst; it != mslast; ++it) {
        Eigen::Vector3d c((*it)->x, (*it)->y, (*it)->z);
        if (c.squaredNorm() > model_bounding_sphere_radius) {
            model_bounding_sphere_radius = c.squaredNorm();
        }
    }
    model_bounding_sphere_radius = sqrt(model_bounding_sphere_radius);

    // split the tree along the largest axis by the centroid
    auto pred = [&](const Sphere* s)
    {
        switch (split_axis) {
        case 0:
            return get_x(*s) < model_bounding_sphere_center.x();
        case 1:
            return get_y(*s) < model_bounding_sphere_center.y();
        case 2:
            return get_z(*s) < model_bounding_sphere_center.z();
        }
    };
    auto msmid = std::partition(msfirst, mslast, pred);

    // recurse on both subtrees
    const size_t left_idx = buildRecursive<Sphere>(msfirst, msmid);
    const size_t right_idx = buildRecursive<Sphere>(msmid, mslast);

    const CollisionSphereModel& sl = m_tree[left_idx];
    const CollisionSphereModel& sr = m_tree[right_idx];

    // compute the optimal sphere to contain both child spheres; mathematical!
    const Eigen::Vector3d& p = sl.center;
    const Eigen::Vector3d& q = sr.center;
    Eigen::Vector3d v = q - p;
    Eigen::Vector3d a = p + v + v.normalized() * sr.radius;
    Eigen::Vector3d b = q - v - v.normalized() * sl.radius;
    Eigen::Vector3d child_bounding_sphere_center = 0.5 * (a + b);
    double child_bounding_sphere_radius = 0.5 * (a - b).norm();

    // create the collision sphere model
    m_tree.emplace_back();
    size_t this_idx = m_tree.size() - 1;
    CollisionSphereModel& sphere = m_tree[this_idx];
    if (child_bounding_sphere_radius < model_bounding_sphere_radius) {
        sphere.center = child_bounding_sphere_center;
        sphere.radius = child_bounding_sphere_radius;
        ROS_INFO("Using child bounding sphere: (%0.3f, %0.3f, %0.3f), %0.3f", sphere.center.x(), sphere.center.y(), sphere.center.z(), sphere.radius);
    }
    else {
        sphere.center = model_bounding_sphere_center;
        sphere.radius = model_bounding_sphere_radius;
        ROS_INFO("Using model bounding sphere: (%0.3f, %0.3f, %0.3f), %0.3f", sphere.center.x(), sphere.center.y(), sphere.center.z(), sphere.radius);
    }
    sphere.name;
    sphere.priority = 0;
    reinterpret_cast<size_t&>(sphere.left) = left_idx;
    reinterpret_cast<size_t&>(sphere.right) = right_idx;
    ROS_INFO("sptr: %p", &sphere);
    return this_idx;
}

template <typename Sphere>
int CollisionSphereTree::computeLargestBoundingBoxAxis(
    typename std::vector<const Sphere*>::iterator first,
    typename std::vector<const Sphere*>::iterator last)
{
    if (std::distance(first, last) == 0) {
        return 0;
    }

    Eigen::Vector3d min = Eigen::Vector3d(get_x(**first), get_y(**first), get_z(**first));
    Eigen::Vector3d max = Eigen::Vector3d(get_x(**first), get_y(**first), get_z(**first));
    for (auto it = first; it != last; ++it) {
        const Sphere& sphere = **it;
        // update bounding box
        if (get_x(sphere) < min.x()) {
            min.x() = get_x(sphere);
        }
        if (get_y(sphere) < min.y()) {
            min.y() = get_y(sphere);
        }
        if (get_z(sphere) < min.z()) {
            min.z() = get_z(sphere);
        }
        if (get_x(sphere) > max.x()) {
            max.x() = get_x(sphere);
        }
        if (get_y(sphere) > max.y()) {
            max.y() = get_y(sphere);
        }
        if (get_z(sphere) > max.z()) {
            max.z() = get_z(sphere);
        }
    }

    const double spanx = max.x() - min.x();
    const double spany = max.y() - min.y();
    const double spanz = max.z() - min.z();
    if (spanx > spany && spanx > spanz) {
        return 0;
    }
    else if (spany > spanz) {
        return 1;
    }
    else {
        return 2;
    }
    return 0; // all the same
}

std::ostream& operator<<(std::ostream& o, const CollisionSpheresModel& csm)
{
    o << "{ link_index: " << csm.link_index << ", spheres: " << csm.spheres <<
            " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionVoxelsModel& cvm)
{
    o << "{ link_index: " << cvm.link_index << ", voxel_res: " <<
            cvm.voxel_res << ", voxels: [" << cvm.voxels.size() << "]" << " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionGroupModel& cgm)
{
    o << "{ name: " << cgm.name << ", link_indices: " << cgm.link_indices << " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const SphereIndex& i)
{
    o << "(" << i.ss << ", " << i.s <<")";
    return o;
}

std::string to_string(const SphereIndex& i)
{
    std::stringstream ss; ss << i; return ss.str();
}

} // namespace collision
} // namespace sbpl
