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

#ifndef sbpl_collision_attached_bodies_collision_model_h
#define sbpl_collision_attached_bodies_collision_model_h

namespace sbpl {
namespace collision {

class AttachedBodiesCollisionModelImpl;

class AttachedBodiesCollisionModel
{
public:

    AttachedBodiesCollisionModel(RobotCollisionState* state);

    ~AttachedBodiesCollisionModel();

    /// \brief Attach a body to the collision model
    /// \param shapes The shapes making up the body
    /// \param transforms The offsets from the attached link for each shape
    /// \param link_name The link to attach to
    bool attachBody(
        const std::string& id,
        const std::vector<shapes::ShapeConstPtr>& shapes,
        const Affine3dVector& transforms,
        const std::string& link_name,
        bool create_voxels_model,
        bool create_spheres_model);
    bool detachBody(const std::string& id);

    size_t attachedBodyCount() const;
    bool   hasAttachedBody(const std::string& id) const;
    int    attachedBodyIndex(const std::string& id) const;
    auto   attachedBodyName(int abidx) const -> const std::string&;
    int    attachedBodyLinkIndex(int abidx) const;

    auto attachedBodyIndices(const std::string& link_name) const ->
            const std::vector<int>&;
    auto attachedBodyIndices(int lidx) const -> const std::vector<int>&;

    auto attachedBodyTransform(const std::string& id) const ->
            const Eigen::Affine3d&;
    auto attachedBodyTransform(int abidx) const -> const Eigen::Affine3d&;

    bool attachedBodyTransformDirty(const std::string& id) const;
    bool attachedBodyTransformDirty(int abidx) const;

    bool updateAttachedBodyTransform(const std::string& id);
    bool updateAttachedBodyTransform(int abidx);

    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto sphereState(int ssidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(int ssidx) const;
    bool updateSphereStates();
    bool updateSphereState(int ssidx);

    auto   groupSphereStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupSphereStateIndices(int gidx) const ->
            const std::vector<int>&;

    /// \brief Return the indices of the collision voxels states NOT belonging
    ///        to this group.
    auto  groupOutsideVoxelsStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;

private:

    std::unique_ptr<AttachedBodiesCollisionModelImpl> m_impl;
};

} // namespace collision
} // namespace sbpl

#endif
