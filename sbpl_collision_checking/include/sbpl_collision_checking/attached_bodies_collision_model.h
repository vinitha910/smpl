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

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <geometric_shapes/shapes.h>

// project includes
#include <sbpl_collision_checking/base_collision_models.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/types.h>

namespace sbpl {
namespace collision {

class AttachedBodiesCollisionModelImpl;

class AttachedBodiesCollisionModel
{
public:

    AttachedBodiesCollisionModel(const RobotCollisionModel* model);

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
        bool create_voxels_model = true,
        bool create_spheres_model = true);
    bool detachBody(const std::string& id);

    /// \name Attached Bodies Model
    ///@{
    size_t attachedBodyCount() const;
    bool   hasAttachedBody(const std::string& id) const;
    int    attachedBodyIndex(const std::string& id) const;
    auto   attachedBodyName(int abidx) const -> const std::string&;
    int    attachedBodyLinkIndex(int abidx) const;

    auto attachedBodyIndices(const std::string& link_name) const
            -> const std::vector<int>&;
    auto attachedBodyIndices(int lidx) const -> const std::vector<int>&;
    ///@}

    /// \name Attached Bodies Collision Model
    ///@{
    size_t sphereModelCount() const;

    bool   hasSpheresModel(const std::string& id) const;
    bool   hasSpheresModel(int abidx) const;
    size_t spheresModelCount() const;
    auto   spheresModel(int smidx) const -> const CollisionSpheresModel&;

    bool   hasVoxelsModel(const std::string& id) const;
    bool   hasVoxelsModel(int abidx) const;
    size_t voxelsModelCount() const;
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    size_t groupCount();
    auto   group(int gidx) const -> const CollisionGroupModel&;
    bool   hasGroup(const std::string& group_name) const;
    int    groupIndex(const std::string& group_name) const;
    auto   groupName(int gidx) const -> const std::string&;

    auto groupLinkIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto groupLinkIndices(int gidx) const -> const std::vector<int>&;
    ///@}

private:

    std::unique_ptr<AttachedBodiesCollisionModelImpl> m_impl;
};

} // namespace collision
} // namespace sbpl

#endif
