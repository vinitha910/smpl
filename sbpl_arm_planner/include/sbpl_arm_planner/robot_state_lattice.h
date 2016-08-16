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

#ifndef sbpl_manip_robot_state_lattice
#define sbpl_manip_robot_state_lattice

// system includes
#include <sbpl/discrete_space_information/environment.h>

// project includes
#include <sbpl_arm_planner/types.h>

namespace sbpl {
namespace manip {

class RobotStateLattice : public DiscreteSpaceInformation
{
public:

    RobotStateLattice();
    virtual ~RobotStateLattice();

//    virtual bool init(RobotModel* robot) = 0;

    virtual bool setStartState(const RobotState& state) = 0;

    virtual int getStartStateID() const = 0;
    virtual int getGoalStateID() const = 0;

    virtual bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) = 0;

    /// \name Restate Required Public Functions
    ///@{
    virtual int GetGoalHeuristic(int state_id) override = 0;
    virtual int GetStartHeuristic(int state_id) override = 0;
    virtual int GetFromToHeuristic(int from_id, int to_id) override = 0;

    virtual void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override = 0;

    virtual void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override = 0;

    virtual void PrintState(
        int state_id,
        bool verbose,
        FILE* f = nullptr) override = 0;
    ///@}

    virtual void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;

    virtual int GetTrueCost(int parentID, int childID) override;

private:

    // Make all attempts to hide the set of useless functions from
    // DiscreteSpaceInformation
    virtual bool InitializeEnv(const char*) final { return false; }
    virtual bool InitializeMDPCfg(MDPConfig*) final { return false; }
    virtual int SizeofCreatedEnv() final { return 0; }
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE*) final { }
    virtual void SetAllPreds(CMDPSTATE*) final { }
    virtual void PrintEnv_Config(FILE*) final { }
};

} // namespace manip
} // namespace sbpl

#endif
