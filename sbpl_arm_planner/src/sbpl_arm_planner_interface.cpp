////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2009, Willow Garage, Inc.
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
//     * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>

#include <assert.h>
#include <time.h>
#include <iostream>
#include <map>

#include <angles/angles.h>
#include <geometry_msgs/Pose.h>
#include <kdl/frames.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <leatherman/viz.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <sbpl_arm_planner/bfs_heuristic.h>
#include <sbpl_arm_planner/environment_robarm3d.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_manipulation_components/post_processing.h>

namespace sbpl {
namespace manip {

SBPLArmPlannerInterface::SBPLArmPlannerInterface(
    RobotModel* rm,
    CollisionChecker* cc,
    ActionSet* as,
    distance_field::PropagationDistanceField* df)
:
    m_initialized(false),
    solution_cost_(INFINITECOST),
    prm_(),
    rm_(rm),
    cc_(cc),
    df_(df),
    as_(as),
    mdp_cfg_(),
    grid_(),
    sbpl_arm_env_(),
    planner_(),
    m_heuristic(nullptr),
    m_heur_vec(),
    m_heuristics(),
    m_planner_id(),
    req_(),
    res_(),
    pscene_(),
    m_starttime(),
    m_vpub()
{
    ros::NodeHandle nh;
    m_vpub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1);
}

SBPLArmPlannerInterface::~SBPLArmPlannerInterface()
{
    if (!m_heuristic) {
        delete m_heuristic;
    }

    for (Heuristic* heur : m_heur_vec) {
        if (heur) {
            delete heur;
        }
    }

    m_heuristics.clear();
    m_heur_vec.clear();
}

bool SBPLArmPlannerInterface::init()
{
    if (!initializeParamsFromParamServer()) {
        return false;
    }

    if (!checkParams(prm_)) {
        return false;
    }

    if (!initializePlannerAndEnvironment()) {
        return false;
    }

    m_initialized = true;
    ROS_INFO("The SBPL arm planner node initialized succesfully");
    return true;
}

bool SBPLArmPlannerInterface::init(const PlanningParams& params)
{
    ROS_INFO("Initializing SBPL Arm Planner Interface");
    ROS_INFO("  Planning Frame: %s", params.planning_frame_.c_str());
    ROS_INFO("  Group Name: %s", params.group_name_.c_str());
    ROS_INFO("  Num Joints: %d", params.num_joints_);
    ROS_INFO("  Planning Joints: %s", to_string(params.planning_joints_).c_str());
    ROS_INFO("  Coord Values: %s", to_string(params.coord_vals_).c_str());
    ROS_INFO("  Coord Deltas: %s", to_string(params.coord_delta_).c_str());

    ROS_INFO("  Use Multiple IK Solutions: %s", params.use_multiple_ik_solutions_ ? "true" : "false");

    ROS_INFO("  Use BFS Heuristic: %s", params.use_bfs_heuristic_ ? "true" : "false");
    ROS_INFO("  Planning Link Sphere Radius: %0.3f", params.planning_link_sphere_radius_);

    ROS_INFO("  Planner Name: %s", params.planner_name_.c_str());
    ROS_INFO("  Epsilon: %0.3f", params.epsilon_);
    ROS_INFO("  Allowed Time: %0.3f", params.allowed_time_);
    ROS_INFO("  Search Mode: %s", params.search_mode_ ? "true" : "false");

    ROS_INFO("  Shortcut Path: %s", params.shortcut_path_ ? "true" : "false");
    ROS_INFO("  Interpolate Path: %s", params.interpolate_path_ ? "true" : "false");
    ROS_INFO("  Waypoint Time: %0.3f", params.waypoint_time_);

    prm_ = params;

    if (!checkParams(prm_)) {
        return false;
    }

    if (!initializePlannerAndEnvironment()) {
        return false;
    }

    m_initialized = true;
    ROS_INFO("The SBPL arm planner node initialized succesfully.");
    return true;
}

bool SBPLArmPlannerInterface::initializeParamsFromParamServer()
{
    if (!prm_.init()) {
        return false;
    }
    return true;
}

bool SBPLArmPlannerInterface::initializePlannerAndEnvironment()
{
    grid_.reset(new OccupancyGrid(df_));
    grid_->setReferenceFrame(prm_.planning_frame_);
    sbpl_arm_env_.reset(new EnvironmentROBARM3D(
            grid_.get(), rm_, cc_, as_, &prm_));

    if (!as_->init(sbpl_arm_env_.get(), prm_.use_multiple_ik_solutions_)) {
        ROS_ERROR("Failed to initialize the action set.");
        return false;
    }

    m_heur.reset(new BfsHeuristic(sbpl_arm_env_.get(), grid_, &prm_));

    if (!sbpl_arm_env_->initEnvironment(m_heur.get())) {
        ROS_ERROR("initEnvironment failed");
        return false;
    }


    ROS_INFO("Initialized sbpl arm planning environment.");
    return true;
}

bool SBPLArmPlannerInterface::solve(
    const moveit_msgs::PlanningSceneConstPtr& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    clearMotionPlanResponse(res);

    if (!m_initialized) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (req.planner_id != m_planner_id) {
        if (!reinitPlanner(req.planner_id)) {
            res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
            return false;
        }
    }

    if (!planning_scene) {
        ROS_ERROR("Planning scene is null");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (req.goal_constraints.empty()) {
        ROS_WARN("No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    if (!canServiceRequest(req, res)) {
        return false;
    }

    ROS_INFO("Got octomap in %s frame", planning_scene->world.octomap.header.frame_id.c_str());
    ROS_INFO("Current prm_.planning_frame_ is %s", prm_.planning_frame_.c_str());

    // preprocess
    clock_t t_preprocess = clock();
    double preprocess_time = (clock() - t_preprocess) / (double)CLOCKS_PER_SEC;

    // plan
    clock_t t_plan = clock();
    res.trajectory_start = planning_scene->robot_state;
    prm_.allowed_time_ = req.allowed_planning_time;
    ROS_INFO("Allowed Time (s): %0.3f", prm_.allowed_time_);

    if (req.goal_constraints.front().position_constraints.size() > 0) {
        ROS_INFO("Planning to position!");
        if (!planToPosition(req, res)) {
            return false;
        }
    }
    else if (req.goal_constraints.front().joint_constraints.size() > 0) {
        ROS_INFO("Planning to joint configuration!");
        if (!planToConfiguration(req, res)) {
            return false;
        }
    }
    else {
        ROS_ERROR("Both position and joint constraints empty!");
        return false;
    }

    res_ = res;
    double plan_time = (clock() - t_plan) / (double)CLOCKS_PER_SEC;
    ROS_INFO("t_plan: %0.3fsec  t_preprocess: %0.3fsec", plan_time, preprocess_time);
    return true;
}

bool SBPLArmPlannerInterface::checkParams(
    const PlanningParams& params) const
{
    if (params.allowed_time_ < 0.0) {
        return false;
    }

    if (params.waypoint_time_ < 0.0) {
        return false;
    }

    if (params.num_joints_ != (int)params.planning_joints_.size()) {
        return false;
    }

    // TODO: check for frame in robot model?
    if (params.planning_frame_.empty()) {
        return false;
    }

    // TODO: check for group in robot model/collision checker
    if (params.group_name_.empty()) {
        return false;
    }

    // TODO: check for search algorithm availability
    if (params.planner_name_.empty()) {
        return false;
    }

    // TODO: check for existence of planning joints in robot model

    if (params.epsilon_ < 1.0) {
        return false;
    }

    if (params.num_joints_ != (int)params.coord_vals_.size()) {
        return false;
    }

    if (params.num_joints_ != (int)params.coord_delta_.size()) {
        return false;
    }

    if (params.cost_multiplier_ < 0) {
        return false;
    }

    if (params.cost_per_cell_ < 0) {
        return false;
    }

    if (params.cost_per_meter_ < 0) {
        return false;
    }

    if (params.cost_per_second_ < 0) {
        return false;
    }

    if (params.time_per_cell_ < 0.0) {
        return false;
    }

    return true;
}

bool SBPLArmPlannerInterface::setStart(const sensor_msgs::JointState& state)
{
    ROS_INFO("Setting start configuration");
    std::vector<double> initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state, prm_.planning_joints_, initial_positions, missing))
    {
        ROS_ERROR("Start state is missing planning joints: %s", to_string(missing).c_str());
        return false;
    }

    ROS_INFO("New Start Configuration");
    ROS_INFO("  Joint Variables: %s", to_string(initial_positions).c_str());

    if (sbpl_arm_env_->setStartConfiguration(initial_positions) == 0) {
        ROS_ERROR("Environment failed to set start state. Not Planning.");
        return false;
    }

    const int start_id = sbpl_arm_env_->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set");
        return false;
    }

    if (planner_->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state. Not Planning.");
        return false;
    }

    return true;
}

bool SBPLArmPlannerInterface::setGoalConfiguration(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO("Setting goal configuration");

    std::vector<double> sbpl_angle_goal(7, 0);
    std::vector<double> sbpl_angle_tolerance(7,0.05); //~3 degrees tolerance by default

    if (goal_constraints.joint_constraints.size() < 7) {
        ROS_WARN("All 7 arm joint constraints must be specified for goal!");
        return false;
    }
    if (goal_constraints.joint_constraints.size() > 7) {
        ROS_WARN("%d joint constraints specified! Using the first 7!", (int)goal_constraints.joint_constraints.size());
        return false;
    }

    const size_t num_angle_constraints = std::min(
            goal_constraints.joint_constraints.size(), sbpl_angle_goal.size());
    for (size_t i = 0; i < num_angle_constraints; i++) {
        const auto& joint_constraint = goal_constraints.joint_constraints[i];
        sbpl_angle_goal[i] = joint_constraint.position;
        sbpl_angle_tolerance[i] =
               0.5 * abs(joint_constraint.tolerance_above) +
               0.5 * abs(joint_constraint.tolerance_below);
        ROS_INFO("Joint %zu [%s]: goal position: %.3f, goal tolerance: %.3f", i, goal_constraints.joint_constraints[i].joint_name.c_str(), sbpl_angle_goal[i], sbpl_angle_tolerance[i]);
    }

    // set sbpl environment goal
    if (!sbpl_arm_env_->setGoalConfiguration(
            sbpl_angle_goal, sbpl_angle_tolerance))
    {
        ROS_ERROR("Failed to set goal state. Exiting.");
        return false;
    }

    // TODO: set heuristic goal

    // set planner goal
    const int goal_id = sbpl_arm_env_->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (planner_->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set goal state. Exiting.");
        return false;
    }

    return true;
}

bool SBPLArmPlannerInterface::setGoalPosition(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO("Setting goal position");

    geometry_msgs::Pose goal_pose;
    geometry_msgs::Vector3 offset;
    if (!extractGoalPoseFromGoalConstraints(
            goal_constraints, goal_pose, offset))
    {
        ROS_WARN("Failed to extract goal pose from goal constraints");
        return false;
    }

    std::vector<std::vector<double>> sbpl_goal(1, std::vector<double>(7, 0.0));

    // currently only supports one goal
    sbpl_goal[0][0] = goal_pose.position.x;
    sbpl_goal[0][1] = goal_pose.position.y;
    sbpl_goal[0][2] = goal_pose.position.z;

    // TODO: do we need to handle gimbal lock in any special way here?
    leatherman::getRPY(goal_pose.orientation, sbpl_goal[0][3], sbpl_goal[0][4], sbpl_goal[0][5]);

    // true => 6-dof goal, false => 3-dof
    sbpl_goal[0][6] = (double)true;

    std::vector<std::vector<double>> sbpl_goal_offset(1, std::vector<double>(3, 0.0));
    sbpl_goal_offset[0][0] = offset.x;
    sbpl_goal_offset[0][1] = offset.y;
    sbpl_goal_offset[0][2] = offset.z;

    // allowable tolerance from goal
    std::vector<std::vector<double>> sbpl_tolerance(1, std::vector<double>(6, 0.0));
    if (!extractGoalToleranceFromGoalConstraints(goal_constraints, &sbpl_tolerance[0][0])) {
        ROS_WARN("Failed to extract goal tolerance from goal constraints");
        return false;
    }

    ROS_INFO("New Goal");
    ROS_INFO("    frame: %s", prm_.planning_frame_.c_str());
    ROS_INFO("    pose: (x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f)",
            sbpl_goal[0][0], sbpl_goal[0][1], sbpl_goal[0][2],
            sbpl_goal[0][3], sbpl_goal[0][4], sbpl_goal[0][5]);
    ROS_INFO("    quaternion: (%0.3f, %0.3f, %0.3f, %0.3f)",
            goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z);
    ROS_INFO("    offset: (%0.3f, %0.3f, %0.3f)", sbpl_goal_offset[0][0], sbpl_goal_offset[0][1], sbpl_goal_offset[0][2]);
    ROS_INFO("    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)",
            sbpl_tolerance[0][0], sbpl_tolerance[0][1], sbpl_tolerance[0][2],
            sbpl_tolerance[0][3], sbpl_tolerance[0][4], sbpl_tolerance[0][5]);

    if (m_planner_id == "MHA*") {
        // set the goal for the heuristic
        ROS_INFO("Setting the heuristic goal for %zu heuristics", m_heur_vec.size());
        for (Heuristic* heur : m_heur_vec) {
            BfsHeuristic* bheur = dynamic_cast<BfsHeuristic*>(heur);
            if (!bheur) {
                continue;
            }
            if (!bheur->setGoal(sbpl_goal[0][0], sbpl_goal[0][1], sbpl_goal[0][2])) {
                ROS_ERROR("Failed to set heuristic goal");
            }
        }
    }

    // set sbpl environment goal
    if (!sbpl_arm_env_->setGoalPosition(sbpl_goal, sbpl_goal_offset, sbpl_tolerance)) {
        ROS_ERROR("Failed to set goal state");
        return false;
    }

    // set sbpl heuristic goal
    if (!m_heur->setGoal(sbpl_goal[0][0], sbpl_goal[0][1], sbpl_goal[0][2])) {
        ROS_ERROR("Failed to set goal for the heuristic");
        return false;
    }

    // set sbpl planner goal
    const int goal_id = sbpl_arm_env_->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (planner_->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set goal state. Exiting.");
        return false;
    }

    return true;
}

bool SBPLArmPlannerInterface::planKinematicPath(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    if (!m_initialized) {
        ROS_ERROR("Hold up a second...the planner isn't initialized yet. Try again in a second or two.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!planToPosition(req, res)) {
        return false;
    }

    return true;
}

bool SBPLArmPlannerInterface::plan(trajectory_msgs::JointTrajectory& traj)
{
    ROS_WARN("Planning!!!!!");
    bool b_ret = false;
    std::vector<int> solution_state_ids;

    //reinitialize the search space
    planner_->force_planning_from_scratch();

    //plan
    ReplanParams replan_params(prm_.allowed_time_);
    replan_params.initial_eps = prm_.epsilon_;
    replan_params.final_eps = 1.0;
    replan_params.dec_eps = 0.2;
    replan_params.return_first_solution = false;
    // replan_params.max_time = prm_.allowed_time_;
    replan_params.repair_time = 1.0;
    b_ret = planner_->replan(&solution_state_ids, replan_params, &solution_cost_);

    //check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN("Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_INFO("Path Length: %zu, Initial Epsilon: %0.3f, Final Epsilon: %0.3f, Solution Cost: %d",
                solution_state_ids.size(), planner_->get_initial_eps(), planner_->get_final_epsilon(), solution_cost_);

        if (!sbpl_arm_env_->convertStateIDPathToJointTrajectory(
                solution_state_ids, traj))
        {
            ROS_ERROR("Failed to convert state id trajectory to joint trajectory");
            return false;
        }
    }
    return b_ret;
}

bool SBPLArmPlannerInterface::planToPosition(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    const std::vector<moveit_msgs::Constraints>& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    m_starttime = clock();
    req_ = req;

    // transform goal pose into reference_frame

    // only acknowledge the first constraint
    const moveit_msgs::Constraints& goal_constraints = goal_constraints_v.front();

    if (!setGoalPosition(goal_constraints)) {
        ROS_ERROR("Failed to set goal position.");
        res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state.joint_state)) {
        ROS_ERROR("Failed to set initial configuration of robot.");
        res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(res.trajectory.joint_trajectory)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions).", prm_.allowed_time_, planner_->get_n_expands());
        res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

//    ROS_INFO("Planner Trajectory:");
//    leatherman::printJointTrajectory(res.trajectory.joint_trajectory, "planner_path");

    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    postProcessPath(res.trajectory.joint_trajectory);

    visualizePath(res.trajectory_start, res.trajectory);

    res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

bool SBPLArmPlannerInterface::planToConfiguration(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    const std::vector<moveit_msgs::Constraints>& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    m_starttime = clock();
    req_ = req;

    // only acknowledge the first constraint
    const moveit_msgs::Constraints& goal_constraints = goal_constraints_v.front();

    if (!setStart(req.start_state.joint_state)) {
        ROS_ERROR("Failed to set initial configuration of robot.");
        res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!setGoalConfiguration(goal_constraints)) {
        ROS_ERROR("Failed to set goal position.");
        res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!plan(res.trajectory.joint_trajectory)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds).", prm_.allowed_time_);
        res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

//    ROS_INFO("Planner Trajectory:");
//    leatherman::printJointTrajectory(res.trajectory.joint_trajectory, "planner_path");

    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    postProcessPath(res.trajectory.joint_trajectory);

    visualizePath(res.trajectory_start, res.trajectory);

    res.planning_time = ((clock() - m_starttime) / (double)CLOCKS_PER_SEC);
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

bool SBPLArmPlannerInterface::canServiceRequest(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    // check for an empty start state
    // TODO: generalize this to "missing necessary state information"
    if (req.start_state.joint_state.position.empty()) {
        ROS_ERROR("No start state given. Unable to plan.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    if (req.goal_constraints.empty()) {
        ROS_ERROR("Goal constraints are empty. Expecting at least one goal constraints with pose and orientation constraints");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    // check if position & orientation constraints is empty
    const moveit_msgs::Constraints& goal_constraints =
            req.goal_constraints.front();

    if ((
            goal_constraints.position_constraints.empty() ||
            goal_constraints.orientation_constraints.empty()
        ) &&
        goal_constraints.joint_constraints.empty())
    {
        ROS_ERROR("Position or orientation constraint is empty.");
        ROS_ERROR("Joint constraint is empty.");
        ROS_ERROR("Expecting a 6D end effector pose constraint or 7D joint constraint. Exiting.");
        return false;
    }

    // check if there is more than one goal constraint
    if (goal_constraints.position_constraints.size() > 1 ||
        goal_constraints.orientation_constraints.size() > 1)
    {
        ROS_WARN("The planning request message contains %zd position and %zd orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", goal_constraints.position_constraints.size(), goal_constraints.orientation_constraints.size());
    }

    return true;
}

std::map<std::string, double> SBPLArmPlannerInterface::getPlannerStats()
{
    std::map<std::string, double> stats;
    stats["initial solution planning time"] = planner_->get_initial_eps_planning_time();
    stats["initial epsilon"] = planner_->get_initial_eps();
    stats["initial solution expansions"] = planner_->get_n_expands_init_solution();
    stats["final epsilon planning time"] = planner_->get_final_eps_planning_time();
    stats["final epsilon"] = planner_->get_final_epsilon();
    stats["solution epsilon"] = planner_->get_solution_eps();
    stats["expansions"] = planner_->get_n_expands();
    stats["solution cost"] = solution_cost_;
    return stats;
}

visualization_msgs::MarkerArray
SBPLArmPlannerInterface::getCollisionModelTrajectoryMarker()
{
    return getCollisionModelTrajectoryVisualization(
            res_.trajectory_start, res_.trajectory);
}

visualization_msgs::MarkerArray
SBPLArmPlannerInterface::getCollisionModelTrajectoryVisualization(
    const moveit_msgs::RobotState& ref_state,
    const moveit_msgs::RobotTrajectory& res_traj) const
{
    visualization_msgs::MarkerArray ma, ma1;
    std::vector<std::vector<double>> traj;

    if (res_traj.joint_trajectory.points.empty()) {
        ROS_ERROR("No trajectory found to visualize yet. Plan a path first.");
        return ma;
    }

    traj.resize(res_traj.joint_trajectory.points.size());
    double cinc = 1.0/double(res_traj.joint_trajectory.points.size());
    for (size_t i = 0; i < res_traj.joint_trajectory.points.size(); ++i) {
        traj[i].resize(res_traj.joint_trajectory.points[i].positions.size());
        for (size_t j = 0; j < res_traj.joint_trajectory.points[i].positions.size(); j++) {
            traj[i][j] = res_traj.joint_trajectory.points[i].positions[j];
        }

        ma1 = cc_->getCollisionModelVisualization(traj[i]);

        for (size_t j = 0; j < ma1.markers.size(); ++j) {
            ma1.markers[j].color.r = 0.1;
            ma1.markers[j].color.g = cinc * double(res_traj.joint_trajectory.points.size() - (i + 1));
            ma1.markers[j].color.b = cinc * double(i);
        }
        ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
    }

    for (size_t i = 0; i < ma.markers.size(); ++i) {
        ma.markers[i].ns = "trajectory";
        ma.markers[i].id = i;
    }

    return ma;
}

bool SBPLArmPlannerInterface::addBfsHeuristic(
    const std::string& name,
    distance_field::PropagationDistanceField* df,
    double radius)
{
    if (!df) {
        ROS_ERROR("Null distance field");
        return false;
    }

    if (m_heuristics.find(name) != m_heuristics.end()) {
        ROS_ERROR("Already have heuristic named '%s'", name.c_str());
        return false;
    }

    auto entry = m_heuristics.insert(std::make_pair(
            name,
            new BfsHeuristic(
                    sbpl_arm_env_.get(),
                    OccupancyGridConstPtr(new OccupancyGrid(df)),
                    &prm_)));

    m_heur_vec.push_back(entry.first->second);

    if (m_planner_id == "MHA*") {
        reinitMhaPlanner();
    }

    ROS_INFO("Added bfs heuristic '%s'", name.c_str());
    return true;
}

bool SBPLArmPlannerInterface::removeHeuristic(
    const std::string& name)
{
    auto it = m_heuristics.find(name);
    if (it == m_heuristics.end()) {
        ROS_ERROR("No heuristic named '%s'", name.c_str());
        return false;
    }

    Heuristic* heur = it->second;

    // remove from the name -> heur map
    m_heuristics.erase(it);

    // remove from the heuristic vector
    auto vit = std::find(m_heur_vec.begin(), m_heur_vec.end(), heur);
    if (vit != m_heur_vec.end()) {
        m_heur_vec.erase(vit);
    }

    delete heur;

    if (m_planner_id == "MHA*") {
        reinitMhaPlanner();
    }

    ROS_INFO("Removed bfs heuristic '%s'", name.c_str());
    return true;
}

visualization_msgs::MarkerArray
SBPLArmPlannerInterface::getGoalVisualization() const
{
    const moveit_msgs::Constraints& goal_constraints = req_.goal_constraints.front();
    if (goal_constraints.position_constraints.empty()) {
        ROS_WARN("Failed to get visualization marker for goals because no position constraints found.");
        return visualization_msgs::MarkerArray();
    }

    // compute space needed for goal poses
    int num_goal_pos_constraints = 0;
    for (int i = 0; i < goal_constraints.position_constraints.size(); ++i) {
        const moveit_msgs::PositionConstraint& pos_constraint =
                goal_constraints.position_constraints[i];
        num_goal_pos_constraints += pos_constraint.constraint_region.primitive_poses.size();
    }
    std::vector<std::vector<double>> poses(
            num_goal_pos_constraints, std::vector<double>(6, 0));

    for (size_t i = 0; i < goal_constraints.position_constraints.size(); ++i) {
        const moveit_msgs::PositionConstraint& pos_constraint =
                goal_constraints.position_constraints[i];
        const moveit_msgs::BoundingVolume constraint_region =
                pos_constraint.constraint_region;
        for (size_t j = 0; j < constraint_region.primitive_poses.size(); ++j) {
            const geometry_msgs::Pose& prim_pose =
                    constraint_region.primitive_poses[j];

            size_t idx = i * pos_constraint.constraint_region.primitive_poses.size() + j;

            poses[idx][0] = prim_pose.position.x;
            poses[idx][1] = prim_pose.position.y;
            poses[idx][2] = prim_pose.position.z;

            if (goal_constraints.orientation_constraints.size() > i) {
                leatherman::getRPY(
                        goal_constraints.orientation_constraints[i].orientation,
                        poses[idx][3], poses[idx][4], poses[idx][5]);
            }
            else
            {
                poses[idx][3] = 0;
                poses[idx][4] = 0;
                poses[idx][5] = 0;
            }
        }
    }
    return viz::getPosesMarkerArray(poses, goal_constraints.position_constraints[0].header.frame_id, "goals", 0);
}

visualization_msgs::MarkerArray
SBPLArmPlannerInterface::getExpansionsVisualization() const
{
    visualization_msgs::MarkerArray ma;
    std::vector<std::vector<double>> expanded_states;
    sbpl_arm_env_->getExpandedStates(expanded_states);
    if (!expanded_states.empty()) {
        std::vector<std::vector<double>> colors(2, std::vector<double>(4, 0));
        colors[0][0] = 1;
        colors[0][3] = 1;
        colors[1][1] = 1;
        colors[1][3] = 1;
        ROS_ERROR("Expansions visualization %d expands, %s frame", int(expanded_states.size()), prm_.planning_frame_.c_str());
        ma = viz::getCubesMarkerArray(expanded_states, 0.01, colors, prm_.planning_frame_, "expansions", 0);
    }
    return ma;
}

visualization_msgs::MarkerArray SBPLArmPlannerInterface::getVisualization(
    const std::string& type) const
{
    if (type == "goal") {
        return getGoalVisualization();
    }
    else if (type == "expansions") {
        return getExpansionsVisualization();
    }
    else if (type =="bfs_walls") {
        const BfsHeuristic* bfs_heur = dynamic_cast<const BfsHeuristic*>(m_heur.get());
        if (bfs_heur) {
            return bfs_heur->getWallsVisualization();
        }
    }
    else if (type == "bfs_values") {
        const BfsHeuristic* bfs_heur = dynamic_cast<const BfsHeuristic*>(m_heur.get());
        if (bfs_heur) {
            return bfs_heur->getValuesVisualization();
        }
    }

    return visualization_msgs::MarkerArray();
}

bool SBPLArmPlannerInterface::extractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    geometry_msgs::Pose& goal_pose,
    geometry_msgs::Vector3& offset) const
{
    if (constraints.position_constraints.empty() ||
        constraints.orientation_constraints.empty())
    {
        ROS_WARN("Conversion from goal constraints to goal pose requires at least one position and one orientation constraint");
        return false;
    }

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?
    const moveit_msgs::PositionConstraint& position_constraint = constraints.position_constraints.front();
    const moveit_msgs::OrientationConstraint& orientation_constraint = constraints.orientation_constraints.front();

    if (position_constraint.constraint_region.primitive_poses.empty()) {
        ROS_WARN("Conversion from goal constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    const shape_msgs::SolidPrimitive& bounding_primitive = position_constraint.constraint_region.primitives.front();
    const geometry_msgs::Pose& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Affine3d T_planning_eef = // T_planning_off * T_off_eef;
            Eigen::Translation3d(
                    primitive_pose.position.x,
                    primitive_pose.position.y,
                    primitive_pose.position.z) *
            Eigen::Quaterniond(
                    primitive_pose.orientation.w,
                    primitive_pose.orientation.x,
                    primitive_pose.orientation.y,
                    primitive_pose.orientation.z);
    Eigen::Vector3d eef_pos(T_planning_eef.translation());

    goal_pose.position.x = eef_pos.x();
    goal_pose.position.y = eef_pos.y();
    goal_pose.position.z = eef_pos.z();

//    goal_pose.position = primitive_pose.position;
    goal_pose.orientation = orientation_constraint.orientation;

    offset = position_constraint.target_point_offset;
    return true;
}

bool SBPLArmPlannerInterface::extractGoalToleranceFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    double* tol)
{
    if (!goal_constraints.position_constraints.empty() &&
        !goal_constraints.position_constraints.front()
                .constraint_region.primitives.empty())
    {
        const moveit_msgs::PositionConstraint& position_constraint =
                goal_constraints.position_constraints.front();
        const shape_msgs::SolidPrimitive& constraint_primitive =
                position_constraint.constraint_region.primitives.front();
        const std::vector<double>& dims = constraint_primitive.dimensions;
        switch (constraint_primitive.type) {
        case shape_msgs::SolidPrimitive::BOX:
            tol[0] = dims[shape_msgs::SolidPrimitive::BOX_X];
            tol[1] = dims[shape_msgs::SolidPrimitive::BOX_Y];
            tol[2] = dims[shape_msgs::SolidPrimitive::BOX_Z];
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            tol[0] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            tol[0] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CONE:
            tol[0] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            break;
        }
    }
    else {
        tol[0] = tol[1] = tol[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        const std::vector<moveit_msgs::OrientationConstraint>& orientation_constraints = goal_constraints.orientation_constraints;
        const moveit_msgs::OrientationConstraint& orientation_constraint = orientation_constraints.front();
        tol[3] = orientation_constraint.absolute_x_axis_tolerance;
        tol[4] = orientation_constraint.absolute_y_axis_tolerance;
        tol[5] = orientation_constraint.absolute_z_axis_tolerance;
    }
    else {
        tol[3] = tol[4] = tol[5] = 0.0;
    }
    return true;
}

void SBPLArmPlannerInterface::clearMotionPlanResponse(
    moveit_msgs::MotionPlanResponse& res) const
{
    res.trajectory_start.joint_state;
    res.trajectory_start.multi_dof_joint_state;
    res.trajectory_start.attached_collision_objects;
    res.trajectory_start.is_diff = false;
    res.group_name = "";
    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.joint_trajectory.header.frame_id = "";
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.multi_dof_joint_trajectory.header.seq = 0;
    res.trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.multi_dof_joint_trajectory.header.frame_id = "";
    res.trajectory.multi_dof_joint_trajectory.joint_names.clear();
    res.trajectory.multi_dof_joint_trajectory.points.clear();
    res.planning_time = 0.0;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
}

void SBPLArmPlannerInterface::clearGraphStateToPlannerStateMap()
{
    if (!sbpl_arm_env_) {
        return;
    }

    std::vector<int*>& state_id_to_index = sbpl_arm_env_->StateID2IndexMapping;
    for (int* mapping : state_id_to_index) {
        for (int i = 0; i < NUMOFINDICES_STATEID2IND; ++i) {
            mapping[i] = -1;
        }
    }
}

bool SBPLArmPlannerInterface::reinitPlanner(const std::string& planner_id)
{
    if (planner_id == "ARA*") {
        return reinitAraPlanner();
    }
    else if (planner_id == "MHA*") {
        return reinitMhaPlanner();
    }
    else if (planner_id == "LARA*") {
        return reinitLaraPlanner();
    }
    else if (planner_id == "LMHA*") {
        return false; // stub name for lazy mha*
    }
    else if (planner_id == "AD*") {
        return false; // stub name for anytime d*
    }
    else if (planner_id == "R*") {
        return false; // stub name for r*
    }
    else {
        return false;
    }
}

bool SBPLArmPlannerInterface::reinitAraPlanner()
{
    clearGraphStateToPlannerStateMap();

    planner_.reset(new ARAPlanner(sbpl_arm_env_.get(), true));
    planner_->set_initialsolution_eps(prm_.epsilon_);
    planner_->set_search_mode(prm_.search_mode_);

    m_planner_id = "ARA*";

    return true;
}

bool SBPLArmPlannerInterface::reinitMhaPlanner()
{
    clearGraphStateToPlannerStateMap();

    if (!m_heuristic) {
        m_heuristic = new EmbeddedHeuristic(sbpl_arm_env_.get());

        auto entry = m_heuristics.insert(std::make_pair(
                "embedded_heuristic", new EmbeddedHeuristic(sbpl_arm_env_.get())));
        m_heur_vec.push_back(entry.first->second);
    }

    MHAPlanner* mha = new MHAPlanner(
            sbpl_arm_env_.get(),
            m_heuristic,
            m_heur_vec.data(),
            m_heur_vec.size());

    mha->set_initial_mha_eps(2.0);

    planner_.reset(mha);
    planner_->set_initialsolution_eps(prm_.epsilon_);
    planner_->set_search_mode(prm_.search_mode_);

    m_planner_id = "MHA*";

    return true;
}

bool SBPLArmPlannerInterface::reinitLaraPlanner()
{
    clearGraphStateToPlannerStateMap();

    planner_.reset(new LazyARAPlanner(sbpl_arm_env_.get(), true));
    planner_->set_initialsolution_eps(prm_.epsilon_);
    planner_->set_search_mode(prm_.search_mode_);

    m_planner_id = "LARA*";

    return true;
}

void SBPLArmPlannerInterface::profilePath(
    trajectory_msgs::JointTrajectory& traj) const
{
    traj.points[0].time_from_start.fromSec(prm_.waypoint_time_);
    for (size_t i = 1; i < traj.points.size(); i++) {
        const double prev_time_s = traj.points[i - 1].time_from_start.toSec();
        traj.points[i].time_from_start.fromSec(prev_time_s + prm_.waypoint_time_);
    }
}

void SBPLArmPlannerInterface::postProcessPath(
    trajectory_msgs::JointTrajectory& traj) const
{
    profilePath(traj);

    // shortcut path
    if (prm_.shortcut_path_) {
        trajectory_msgs::JointTrajectory straj;
        if (!interpolateTrajectory(cc_, traj.points, straj.points)) {
            ROS_WARN("Failed to interpolate planned trajectory with %zu waypoints before shortcutting.", traj.points.size());
            trajectory_msgs::JointTrajectory orig_traj = traj;
            shortcutTrajectory(cc_, orig_traj.points, traj.points);
        }
        else {
            shortcutTrajectory(cc_, straj.points, traj.points);
        }
    }

    // interpolate path
    if (prm_.interpolate_path_) {
        trajectory_msgs::JointTrajectory itraj = traj;
        if (!interpolateTrajectory(cc_, itraj.points, traj.points)) {
            ROS_WARN("Failed to interpolate trajectory");
        }
    }

    if (prm_.print_path_) {
        leatherman::printJointTrajectory(traj, "path");
    }
}

void SBPLArmPlannerInterface::visualizePath(
    const moveit_msgs::RobotState& traj_start,
    const moveit_msgs::RobotTrajectory& traj) const
{
    ROS_INFO("Visualizing trajectory");
    m_vpub.publish(getCollisionModelTrajectoryVisualization(traj_start, traj));
    ros::Duration(1.0).sleep();
}

} // namespace manip
} // namespace sbpl
