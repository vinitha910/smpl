////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Benjamin Cohen
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

/// \author Benjamin Cohen

#ifndef _SBPL_DEBUG_CODES_
#define _SBPL_DEBUG_CODES_

namespace sbpl_arm_planner {

enum DebugCode
{
    SUCCESS,
    COLLISION_BETWEEN_ARMS,
    RIGHT_ARM_IN_COLLISION,
    LEFT_ARM_IN_COLLISION,
    ATTACHED_OBJECT_IN_COLLISION,
    RIGHT_IK_FAIL_IK_SEARCH_SUCCESS,
    RIGHT_IK_FAIL_IK_SEARCH_FAIL,
    LEFT_IK_FAIL_IK_SEARCH_SUCCESS,
    LEFT_IK_FAIL_IK_SEARCH_FAIL,
    INVALID_RIGHT_SHOULDER_PAN_ANGLE,
    INVALID_RIGHT_SHOULDER_PITCH_ANGLE,
    INVALID_RIGHT_UPPER_ARM_ROLL_ANGLE,
    INVALID_RIGHT_ELBOW_FLEX_ANGLE,
    INVALID_RIGHT_FOREARM_ROLL_ANGLE,
    INVALID_RIGHT_WRIST_PITCH_ANGLE,
    INVALID_RIGHT_WRIST_ROLL_ANGLE,
    INVALID_LEFT_SHOULDER_PAN_ANGLE,
    INVALID_LEFT_SHOULDER_PITCH_ANGLE,
    INVALID_LEFT_UPPER_ARM_ROLL_ANGLE,
    INVALID_LEFT_ELBOW_FLEX_ANGLE,
    INVALID_LEFT_FOREARM_ROLL_ANGLE,
    INVALID_LEFT_WRIST_PITCH_ANGLE,
    INVALID_LEFT_WRIST_ROLL_ANGLE,
    NUM_DEBUG_CODES
};

/*
  static const char* DebugCodeNames[] =
        {"success",
        "right ik fail ik search success",
        "right ik fail ik search fail",
        "left ik fail ik search success",
        "left ik fail ik search fail",
        "invalid right shoulder pan",
        "invalid right shoulder pitch",
        "invalid right upper arm roll",
        "invalid right elbow flex",
        "invalid right forearm roll",
        "invalid right wrist pitch",
        "invalid right wrist roll",
        "invalid left shoulder pan",
        "invalid left shoulder pitch",
        "invalid left upper arm roll",
        "invalid left elbow flex",
        "invalid left forearm roll",
        "invalid left wrist pitch",
        "invalid left wrist roll",
        "collision between arms",
        "right arm in collision",
        "left arm in collision",
        "attached object in collision"};
 */

} // namespace sbpl_arm_planner

#endif

