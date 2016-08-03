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

#ifndef sbpl_collision_base_collision_models_h
#define sbpl_collision_base_collision_models_h

// standard includes
#include <ostream>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

/// \brief Collision Sphere Model Specification
struct CollisionSphereModel
{
    std::string name;
    Eigen::Vector3d center; ///< offset from link center
    double radius;
    int priority;
    const CollisionSphereModel *left, *right;
};

std::ostream& operator<<(std::ostream& o, const CollisionSphereModel& csm);

class CollisionSphereTree
{
public:

    CollisionSphereTree();

    void buildFrom(const std::vector<CollisionSphereConfig>& spheres);
//    void buildFrom(const std::vector<CollisionSphereModel>& spheres);
//    void buildFrom(const std::vector<const CollisionSphereModel*>& spheres);

    const CollisionSphereModel* root() const { return m_tree.data(); }
    size_t size() const { return m_tree.size(); }

private:

    template <typename Sphere>
    size_t buildRecursive(
        typename std::vector<const Sphere*>::iterator msfirst,
        typename std::vector<const Sphere*>::iterator mslast);

    template <typename Sphere>
    int computeLargestBoundingBoxAxis(
        typename std::vector<const Sphere*>::iterator msfirst,
        typename std::vector<const Sphere*>::iterator mslast);

    std::vector<CollisionSphereModel> m_tree;
};

/// \brief Collision Spheres Model Specification
struct CollisionSpheresModel
{
    int link_index;
//    CollisionSphereTree spheres;
    std::vector<CollisionSphereModel> spheres;
};

std::ostream& operator<<(std::ostream& o, const CollisionSpheresModel& csm);

/// \brief Collision Voxels Model Specification
struct CollisionVoxelsModel
{
    int link_index; // -1 if not attached to a link
    double voxel_res;
    std::vector<Eigen::Vector3d> voxels; // in the link frame
};

std::ostream& operator<<(std::ostream& o, const CollisionVoxelsModel& cvm);

/// \brief Collision Group Model Specification
struct CollisionGroupModel
{
    std::string name;
    std::vector<int> link_indices;
};

std::ostream& operator<<(std::ostream& o, const CollisionGroupModel& cgm);

struct SphereIndex
{
    int ss;
    int s;

    SphereIndex() { }
    SphereIndex(int ss, int s) : ss(ss), s(s) { }
};

std::ostream& operator<<(std::ostream& o, const SphereIndex& i);
std::string to_string(const SphereIndex& i);

} // namespace collision
} // namespace sbpl

#endif
