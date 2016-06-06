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

#ifndef sbpl_manip_pr2_action_set_h
#define sbpl_manip_pr2_action_set_h

// system includes
#include <sbpl_arm_planner/action_set.h>

namespace sbpl_arm_planner {

class EnvironmentROBARM3D;

enum MotionPrimitiveType {
  LONG_DISTANCE,
  SHORT_DISTANCE,
  SNAP_TO_RPY,
  SNAP_TO_XYZ_RPY,
  NUMBER_OF_MPRIM_TYPES
};

class ActionSet
{
  public:
    ActionSet();

    ~ActionSet(){};

    bool init(std::string filename, EnvironmentROBARM3D *env);

    bool getActionSet(const RobotState &parent, std::vector<Action> &actions);

    void print();

  private:

    bool use_multires_mprims_;

    bool use_ik_;

    double short_dist_mprims_thresh_m_;

    double ik_amp_dist_thresh_m_;

    EnvironmentROBARM3D *env_;

    std::vector<MotionPrimitive> mp_;

    std::vector<std::string> motion_primitive_type_names_;

    bool getMotionPrimitivesFromFile(FILE* fCfg);

    void addMotionPrim(const std::vector<double> &mprim, bool add_converse, bool short_dist_mprim);

    bool applyMotionPrimitive(const RobotState &state, MotionPrimitive &mp, Action &action);

    bool getAction(const RobotState &parent, double dist_to_goal, MotionPrimitive &mp, Action &action);
};

}
#endif

