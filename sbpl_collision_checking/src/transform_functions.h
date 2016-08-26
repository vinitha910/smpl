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

#ifndef sbpl_collision_transform_functions_h
#define sbpl_collision_transform_functions_h

#include <Eigen/Dense>

#define SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS 1

namespace sbpl {
namespace collision {

////////////////////////////////////////
// Compute*JointTransform Definitions //
////////////////////////////////////////

Affine3 ComputeRevoluteJointTransform(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputeRevoluteJointTransformX(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputeRevoluteJointTransformY(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputeRevoluteJointTransformZ(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputeContinuousJointTransform(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputePrismaticJointTransform(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputeFloatingJointTransform(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputePlanarJointTransform(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

Affine3 ComputeFixedJointTransform(
    const Affine3d& origin,
    const Vector3& axis,
    real* jvals);

////////////////////////////////////////
// Compute*JointTransform Definitions //
////////////////////////////////////////

inline
Affine3 ComputeRevoluteJointTransform(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
    return o * AngleAxis(jvals[0], axis);
}

inline
Affine3 ComputeRevoluteJointTransformX(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
#if SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS
    Affine3 t;
    real cth = std::cos(jvals[0]);
    real sth = std::sin(jvals[0]);
    t(0,0) = o(0,0);
    t(1,0) = o(1,0);
    t(2,0) = o(2,0);
    t(3,0) = 0.0;

    t(0,1) = cth * o(0,1) + sth * o(0,2);
    t(1,1) = cth * o(1,1) + sth * o(1,2);
    t(2,1) = cth * o(2,1) + sth * o(2,2);
    t(3,1) = 0.0;

    t(0,2) = cth * o(0,2) - sth * o(0,1);
    t(1,2) = cth * o(1,2) - sth * o(1,1);
    t(2,2) = cth * o(2,2) - sth * o(2,1);
    t(3,0) = 0.0;

    t(0,3) = o(0,3);
    t(1,3) = o(1,3);
    t(2,3) = o(2,3);
    t(3,3) = 1.0;
    return t;
#else
    return ComputeRevoluteJointTransform(o, axis, jvals);
#endif
}

inline
Affine3 ComputeRevoluteJointTransformY(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
#if SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS
    Affine3 t;
    real cth = std::cos(jvals[0]);
    real sth = std::sin(jvals[0]);
    t(0,0) = cth * o(0,0) - sth * o(0,2);
    t(1,0) = cth * o(1,0) - sth * o(1,2);
    t(2,0) = cth * o(2,0) - sth * o(2,2);
    t(3,0) = 0.0;

    t(0,1) = o(0,1);
    t(1,1) = o(1,1);
    t(2,1) = o(2,1);
    t(3,1) = 0.0;

    t(0,2) = sth * o(0,0) + cth * o(0,2);
    t(1,2) = sth * o(1,0) + cth * o(1,2);
    t(2,2) = sth * o(2,0) + cth * o(2,2);
    t(3,0) = 0.0;

    t(0,3) = o(0,3);
    t(1,3) = o(1,3);
    t(2,3) = o(2,3);
    t(3,3) = 1.0;
    return t;
#else
    return ComputeRevoluteJointTransform(o, axis, jvals);
#endif
}

inline
Affine3 ComputeRevoluteJointTransformZ(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
#if SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS
    Affine3 t;
    real cth = std::cos(jvals[0]);
    real sth = std::sin(jvals[0]);
    t(0,0) = o(0,0) * cth + o(0,1) * sth;
    t(1,0) = o(1,0) * cth + o(1,1) * sth;
    t(2,0) = o(2,0) * cth + o(2,1) * sth;
    t(3,0) = 0.0;

    t(0,1) = o(0,1) * cth - o(0,0) * sth;
    t(1,1) = o(1,1) * cth - o(1,0) * sth;
    t(2,1) = o(2,1) * cth - o(2,0) * sth;
    t(3,1) = 0.0;

    t(0,2) = o(0,2);
    t(1,2) = o(1,2);
    t(2,2) = o(2,2);
    t(3,0) = 0.0;

    t(0,3) = o(0,3);
    t(1,3) = o(1,3);
    t(2,3) = o(2,3);
    t(3,3) = 1.0;
    return t;
#else
    return ComputeRevoluteJointTransform(o, axis, jvals);
#endif
}

inline
Affine3 ComputeContinuousJointTransform(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
    return o * AngleAxis(jvals[0], axis);
}

inline
Affine3 ComputePrismaticJointTransform(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
    return o * Translation3(Vector3(0.0, 0.0, jvals[0]));
}

inline
Affine3 ComputeFloatingJointTransform(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
    // TODO: be mindful that quaternion values here may not be normalized
    return o *
            Translation3(Vector3(jvals[0], jvals[1], jvals[2])) *
            Quaternion(jvals[6], jvals[3], jvals[4], jvals[5]);
}

inline
Affine3 ComputePlanarJointTransform(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
    return o *
            Translation3(Vector3(jvals[0], jvals[1], 0.0)) *
            AngleAxis(jvals[2], Vector3::UnitZ());
}

inline
Affine3 ComputeFixedJointTransform(
    const Affine3d& o,
    const Vector3& axis,
    real* jvals)
{
    return o;
}

} // namespace collision
} // namespace sbpl

#endif
