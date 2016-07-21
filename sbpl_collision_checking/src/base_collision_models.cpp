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

std::ostream& operator<<(std::ostream& o, const CollisionSphereModel& csm)
{
    o << "{ name: " << csm.name << ", center: (" << csm.center.x() << ", " <<
            csm.center.y() << ", " << csm.center.z() << "), radius: " <<
            csm.radius << ", priority: " << csm.priority << " }";
    return o;
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
