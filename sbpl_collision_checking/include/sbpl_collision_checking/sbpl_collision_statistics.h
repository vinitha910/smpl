////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the University of Pennsylvania nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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

/// \author Benjamin Cohen

#ifndef sbpl_collision_SBPLCollisionStatistics_h
#define sbpl_collision_SBPLCollisionStatistics_h

// standard includes
#include <map>
#include <string>

// system includes
#include <kdl/frames.hpp>

// project includes
#include <sbpl_collision_checking/sbpl_collision_model.h>

namespace sbpl {
namespace collision {

class SBPLCollisionStatistics
{
public:

    SBPLCollisionStatistics(Group* group);

    ~SBPLCollisionStatistics(){};

    void logSphereCollision(
        Sphere* s,
        int& x,
        int& y,
        int& z,
        unsigned char& dist_temp);

    void resetSphereCollisionLogs();

    void printSphereCollisionStats(std::string text);

private:

    Group* group_;

    /** \brief log the number of collisions per collision sphere **/
    std::map<Sphere*, int> col_sph_map_;

    /** \brief log the number of collisions per cell **/
    std::map<KDL::Vector, int> col_cell_map_;
};

} // namespace collision
} // namespace sbpl

#endif
