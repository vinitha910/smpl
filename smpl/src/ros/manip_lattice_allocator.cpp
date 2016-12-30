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

#include <smpl/ros/manip_lattice_allocator.h>

// system includes
#include <leatherman/print.h>

// project includes
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>

namespace sbpl {
namespace motion {

static const char* PI_LOGGER = "planning";

RobotPlanningSpacePtr ManipLatticeAllocator::allocate(
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
{
    auto pspace = std::make_shared<ManipLattice>(robot, checker, params);
    if (m_grid) {
        pspace->setVisualizationFrameId(m_grid->getReferenceFrame());
    }

    auto aspace = std::make_shared<ManipLatticeActionSpace>(pspace);
    if (!aspace->load(params->action_filename)) {
        return RobotPlanningSpacePtr();
    }

    ROS_DEBUG_NAMED(PI_LOGGER, "Action Set:");
    for (auto ait = aspace->begin(); ait != aspace->end(); ++ait) {
        ROS_DEBUG_NAMED(PI_LOGGER, "  type: %s", to_string(ait->type).c_str());
        if (ait->type == MotionPrimitive::SNAP_TO_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", aspace->useAmp(MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", aspace->ampThresh(MotionPrimitive::SNAP_TO_RPY));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", aspace->ampThresh(MotionPrimitive::SNAP_TO_XYZ));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", aspace->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY));
        } else if (ait->type == MotionPrimitive::LONG_DISTANCE ||
            ait->type == MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_DEBUG_NAMED(PI_LOGGER, "    action: %s", to_string(ait->action).c_str());
        }
    }

    // associate action space with lattice
    if (!pspace->setActionSpace(aspace)) {
        ROS_ERROR("Failed to associate action space with planning space");
        return RobotPlanningSpacePtr();
    }
    return pspace;
}

} // namespace motion
} // namespace sbpl
