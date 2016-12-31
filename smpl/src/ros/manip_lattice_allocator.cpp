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

static const char* PI_LOGGER = "simple";

RobotPlanningSpacePtr ManipLatticeAllocator::allocate(
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());
    std::string mprim_filename;
    bool use_multiple_ik_solutions = false; // TODO: config parameter for this
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_thresh;
    double rpy_snap_thresh;
    double xyzrpy_snap_thresh;
    double short_dist_mprims_thresh;

    auto it = params->params.find("discretization");
    if (it == params->params.end()) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return RobotPlanningSpacePtr();
    }
    std::map<std::string, double> disc;
    std::stringstream ss(it->second);
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        const std::string& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == disc.end()) {
            ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return RobotPlanningSpacePtr();
        }
        resolutions[vidx] = dit->second;
    }

    it = params->params.find("mprim_filename");
    if (it == params->params.end()) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'mprim_filename' not found in planning params");
        return RobotPlanningSpacePtr();
    }
    mprim_filename = it->second;

    it = params->params.find("use_xyz_snap_mprim");
    if (it == params->params.end()) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'use_xyz_snap_mprim' not found in planning params");
        return RobotPlanningSpacePtr();
    }
    use_xyz_snap_mprim = it->second == "true";

    it = params->params.find("use_rpy_snap_mprim");
    if (it == params->params.end()) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'use_rpy_snap_mprim' not found in planning params");
        return RobotPlanningSpacePtr();
    }
    use_rpy_snap_mprim = it->second == "true";

    it = params->params.find("use_xyzrpy_snap_mprim");
    if (it == params->params.end()) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'use_xyzrpy_snap_mprim' not found in planning params");
        return RobotPlanningSpacePtr();
    }
    use_xyzrpy_snap_mprim = it->second == "true";

    it = params->params.find("use_short_dist_mprims");
    if (it == params->params.end()) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'use_short_dist_mprims' not found in planning params");
        return RobotPlanningSpacePtr();
    }
    use_short_dist_mprims = it->second == "true";

    try {
        it = params->params.find("xyz_snap_dist_thresh");
        if (it == params->params.end()) {
            ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'xyz_snap_dist_thresh' not found in planning params");
            return RobotPlanningSpacePtr();
        }
        xyz_snap_thresh = std::stod(it->second);

        it = params->params.find("rpy_snap_dist_thresh");
        if (it == params->params.end()) {
            ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'rpy_snap_dist_thresh' not found in planning params");
            return RobotPlanningSpacePtr();
        }
        rpy_snap_thresh = std::stod(it->second);

        it = params->params.find("xyzrpy_snap_dist_thresh");
        if (it == params->params.end()) {
            ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'xyzrpy_snap_dist_thresh' not found in planning params");
            return RobotPlanningSpacePtr();
        }
        xyzrpy_snap_thresh = std::stod(it->second);

        it = params->params.find("short_dist_mprims_thresh");
        if (it == params->params.end()) {
            ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'short_dist_mprims_thresh' not found in planning params");
            return RobotPlanningSpacePtr();
        }
        short_dist_mprims_thresh = std::stod(it->second);
    } catch (const std::logic_error& ex) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to convert amp distance thresholds to floating-point values");
        return RobotPlanningSpacePtr();
    }

    ////////////////////
    // Initialization //
    ////////////////////

    auto pspace = std::make_shared<ManipLattice>(robot, checker, params);
    if (!pspace->init(resolutions)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice");
        return RobotPlanningSpacePtr();
    }

    if (m_grid) {
        pspace->setVisualizationFrameId(m_grid->getReferenceFrame());
    }

    auto aspace = std::make_shared<ManipLatticeActionSpace>(pspace);
    aspace->useMultipleIkSolutions(use_multiple_ik_solutions);
    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ, use_xyz_snap_mprim);
    aspace->useAmp(MotionPrimitive::SNAP_TO_RPY, use_rpy_snap_mprim);
    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, use_xyzrpy_snap_mprim);
    aspace->useAmp(MotionPrimitive::SHORT_DISTANCE, use_short_dist_mprims);
    aspace->ampThresh(MotionPrimitive::SNAP_TO_XYZ, xyz_snap_thresh);
    aspace->ampThresh(MotionPrimitive::SNAP_TO_RPY, rpy_snap_thresh);
    aspace->ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, xyzrpy_snap_thresh);
    aspace->ampThresh(MotionPrimitive::SHORT_DISTANCE, short_dist_mprims_thresh);

    if (!aspace->load(mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", mprim_filename.c_str());
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
