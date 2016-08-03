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

// system includes
#include <leatherman/print.h>

// project includes
#include <sbpl_collision_checking/base_collision_states.h>

namespace sbpl {
namespace collision {

std::ostream& operator<<(std::ostream& o, const CollisionSphereState& css)
{
    o << "{ model: " << css.model << ", parent_state: " << css.parent_state <<
            ", pos: (" << css.pos.x() << ", " << css.pos.y() << ", " <<
            css.pos.z() << ") }";
    return o;
}

void CollisionSphereStateTree::buildFrom(CollisionSpheresState* parent_state)
{
    m_tree.resize(parent_state->model->spheres.size());
    for (size_t i = 0; i < m_tree.size(); ++i) {
        const CollisionSphereModel& model = parent_state->model->spheres[i];
        CollisionSphereState& state = m_tree[i];
        state.model = &model;
        state.parent_state = parent_state;
        state.left = &m_tree[0] + std::distance(parent_state->model->spheres.root(), model.left);
        state.right = &m_tree[0] + std::distance(parent_state->model->spheres.root(), model.right);
    }
}

CollisionSphereStateTree::CollisionSphereStateTree(
    const CollisionSphereStateTree& o)
{
    m_tree = o.m_tree;
    for (size_t i = 0; i < m_tree.size(); ++i) {
        m_tree[i].left = &m_tree[0] + std::distance(
                o.root(), (const CollisionSphereState*)o.m_tree[i].left);
        m_tree[i].right = &m_tree[0] + std::distance(
                o.root(), (const CollisionSphereState*)o.m_tree[i].right);
    }
}

CollisionSphereStateTree& CollisionSphereStateTree::operator=(
    const CollisionSphereStateTree& rhs)
{
    if (this != &rhs) {
        m_tree = rhs.m_tree;
        for (size_t i = 0; i < m_tree.size(); ++i) {
            m_tree[i].left = &m_tree[0] + std::distance(
                    rhs.root(), (const CollisionSphereState*)rhs.m_tree[i].left);
            m_tree[i].right = &m_tree[0] + std::distance(
                    rhs.root(), (const CollisionSphereState*)rhs.m_tree[i].right);
        }
    }
    return *this;
}

std::ostream& operator<<(std::ostream& o, const CollisionSphereStateTree& tree)
{
    o << tree.m_tree;
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionSpheresState& css)
{
    o << "{ model: " << css.model << ", spheres: " << css.spheres << " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionVoxelsState& cvs)
{
    o << "{ model: " << cvs.model << ", voxels: [" << cvs.voxels.size() << "] }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionGroupState& cgs)
{
    o << "{ model: " << cgs.model << ", sphere_indices: " <<
            cgs.spheres_indices << ", voxels_indices: " << cgs.voxels_indices <<
            " }";
    return o;
}

} // namespace collision
} // namespace sbpl
