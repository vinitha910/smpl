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

namespace sbpl {
namespace collision {

class BodyCollisionModel
{
public:

    BodyCollisionModel();

    /// \brief Add a set of collision spheres
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const CollisionSphereConfig&. Objects
    /// of type Indexer must meet the requirements of a Callable type with
    /// ArgumentType const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void insertCollisionSpheres(
        InputSphereIt first, InputSphereIt last,
        const Indexer& indexer);

    /// \brief Add a set of collision spheres models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const CollisionSpheresModelConfig&.
    /// Objects of type Indexer must meet the requirements of a Callable type
    /// with ArgumentType const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void insertCollisionSpheresModels(
        InputIt first, Inputit last,
        const Indexer& indexer);

    /// \brief Add a set of collision voxels models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const CollisionVoxelsModelConfig&.
    /// Objects of type Indexer must meet the requirements of a Callable type
    /// with ArgumentType const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void insertCollisionVoxelsModels(
        InputIt first, InputIt last,
        const Indexer& indexer);

    /// \brief Add a set of group models
    ///
    /// Objects of type InputIt must be able to be dereferenced and then
    /// implicitly convertible to type const CollisionGroupConfig&. Objects
    /// of type Indexer must meet the requirements of a Callable type with
    /// ArgumentType const std::string& and ReturnType int
    template <typename InputIt, typename Indexer>
    void insertGroupModels(
        InputIt first, InputIt last,
        const Indexer& index);

    void removeCollisionSpheres(
        InputIt first, InputIt last)

private:

    std::vector<CollisionSphereModel>   m_sphere_models;
    std::vector<CollisionSpheresModel>  m_spheres_models;
    std::vector<CollisionVoxelsModel>   m_voxels_models;
    std::vector<CollisionGroupModel>    m_group_models;
    hash_map<std::string, int>          m_group_name_to_index;
};

template <typename InputIt, typename Indexer>
void BodyCollisionModel::insertCollisionSpheres(
    InputSphereIt first, InputSphereIt last,
    const Indexer& indexer)
{
    auto new_sphere_count = std::distance(first, last);
    auto prev_size = m_sphere_models.size();
    auto prev_capacity = m_sphere_models.capacity();
    m_sphere_models.resize(prev_size + new_sphere_count);
    size_t i = prev_size;
    for (auto it = first; it != last; ++it) {
        const CollisionSphereConfig& config = *it;
        CollisionSphereModel& sphere = m_sphere_models[i++];
        sphere.name = config.name;
        sphere.center = Eigen::Vector3d(config.x, config.y, config.z);
        sphere.radius = config.radius;
        sphere.priority = config.priority;
    }

    if (m_sphere_models.size() > prev_capacity) {
        // regenerate references from all
    }
}

template <typename InputIt, typename Indexer>
void BodyCollisionModel::insertCollisionSpheresModels(
    InputIt first, Inputit last,
    const Indexer& indexer)
{

}

template <typename InputIt, typename Indexer>
void BodyCollisionModel::insertCollisionVoxelsModels(
    InputIt first, InputIt last,
    const Indexer& indexer)
{

}

template <typename InputIt, typename Indexer>
void BodyCollisionModel::insertGroupModels(
    InputIt first, InputIt last,
    const Indexer& index)
{

}

} // namespace collision
} // namespace sbpl
