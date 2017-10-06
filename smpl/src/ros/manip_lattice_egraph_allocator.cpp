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

#include <smpl/ros/manip_lattice_egraph_allocator.h>

// project includes
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/graph/manip_lattice_action_space.h>

namespace sbpl {
namespace motion {

static const char* PI_LOGGER = "simple";

RobotPlanningSpacePtr ManipLatticeEgraphAllocator::allocate(
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
{
    // TODO: remove the copypasta between here and ManipLatticeAllocator
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

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return RobotPlanningSpacePtr();
    }
    std::map<std::string, double> disc;
    std::stringstream ss(disc_string);
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }
    SMPL_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        const std::string& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == disc.end()) {
            SMPL_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return RobotPlanningSpacePtr();
        }
        resolutions[vidx] = dit->second;
    }

    if (!params->getParam("mprim_filename", mprim_filename)) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Parameter 'mprim_filename' not found in planning params");
        return RobotPlanningSpacePtr();
    }

    params->param("use_multiple_ik_solutions", use_multiple_ik_solutions, false);

    params->param("use_xyz_snap_mprim", use_xyz_snap_mprim, false);
    params->param("use_rpy_snap_mprim", use_rpy_snap_mprim, false);
    params->param("use_xyzrpy_snap_mprim", use_xyzrpy_snap_mprim, false);
    params->param("use_short_dist_mprims", use_short_dist_mprims, false);

    params->param("xyz_snap_dist_thresh", xyz_snap_thresh, 0.0);
    params->param("rpy_snap_dist_thresh", rpy_snap_thresh, 0.0);
    params->param("xyzrpy_snap_dist_thresh", xyzrpy_snap_thresh, 0.0);
    params->param("short_dist_mprims_thresh", short_dist_mprims_thresh, 0.0);

    auto pspace = std::make_shared<ManipLatticeEgraph>(robot, checker, params);
    if (!pspace->init(resolutions)) {
        SMPL_ERROR("Failed to initialize Manip Lattice Egraph");
        return RobotPlanningSpacePtr();
    }

    ////////////////////
    // Initialization //
    ////////////////////

    auto aspace = std::make_shared<ManipLatticeActionSpace>(pspace.get());
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
        SMPL_ERROR("Failed to load actions from file '%s'", mprim_filename.c_str());
        return RobotPlanningSpacePtr();
    }

    // associate action space with lattice
    if (!pspace->setActionSpace(aspace)) {
        SMPL_ERROR("Failed to associate action space with planning space");
        return RobotPlanningSpacePtr();
    }

    std::string egraph_path;
    if (params->getParam("egraph_path", egraph_path)) {
        // warning printed within, allow to fail silently
        (void)pspace->loadExperienceGraph(egraph_path);
    } else {
        SMPL_WARN("No experience graph file parameter");
    }

    return pspace;
}

} // namespace motion
} // namespace sbpl
