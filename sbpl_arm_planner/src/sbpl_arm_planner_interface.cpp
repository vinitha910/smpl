/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <visualization_msgs/Marker.h>

clock_t starttime;

using namespace sbpl_arm_planner;

SBPLArmPlannerInterface::SBPLArmPlannerInterface(
  RobotModel *rm,
  CollisionChecker *cc,
  ActionSet* as,
  distance_field::PropagationDistanceField* df)
:
  nh_("~"),
  planner_(NULL),
  sbpl_arm_env_(NULL),
  cc_(NULL),
  grid_(NULL),
  prm_(NULL)
{
  rm_ = rm;
  cc_ = cc;
  as_ = as;
  df_ = df;
  planner_initialized_ = false;
}

SBPLArmPlannerInterface::~SBPLArmPlannerInterface()
{
  if(planner_ != NULL)
    delete planner_;
  if(sbpl_arm_env_ != NULL)
    delete sbpl_arm_env_;
  if(prm_ != NULL)
    delete prm_;
}

bool SBPLArmPlannerInterface::init()
{
  if(!initializePlannerAndEnvironment())
    return false;

  planner_initialized_ = true;
  ROS_INFO_PRETTY("The SBPL arm planner node initialized succesfully.");
  return true;
}

bool SBPLArmPlannerInterface::initializePlannerAndEnvironment()
{
  prm_ = new sbpl_arm_planner::PlanningParams();
  if(!prm_->init())
    return false;

  grid_ = new sbpl_arm_planner::OccupancyGrid(df_);
  sbpl_arm_env_ = new sbpl_arm_planner::EnvironmentROBARM3D(grid_, rm_, cc_, as_, prm_);

  if(!sbpl_arm_env_)
    return false;

  if(!as_->init(sbpl_arm_env_))
  {
    ROS_ERROR_PRETTY("Failed to initialize the action set.");
    return false;
  }
  //as_->print();

  //initialize environment
  planner_ = new ARAPlanner(sbpl_arm_env_, true);

  //initialize arm planner environment
  if(!sbpl_arm_env_->initEnvironment())
  {
    ROS_ERROR_PRETTY("ERROR: initEnvironment failed");
    return false;
  }

  //initialize MDP
  if(!sbpl_arm_env_->InitializeMDPCfg(&mdp_cfg_))
  {
    ROS_ERROR_PRETTY("ERROR: InitializeMDPCfg failed");
    return false;
  }

  //set epsilon
  planner_->set_initialsolution_eps(100.0);

  //set search mode (true - settle with first solution)
  planner_->set_search_mode(prm_->search_mode_);
  ROS_INFO_PRETTY("Initialized sbpl arm planning environment.");
  return true;
}

bool SBPLArmPlannerInterface::solve(const moveit_msgs::PlanningSceneConstPtr& planning_scene,
                                    const moveit_msgs::GetMotionPlan::Request &req,
                                    moveit_msgs::GetMotionPlan::Response &res)
{
  if(!planner_initialized_)
    return false;

  ROS_INFO("Got octomap in %s frame", planning_scene->world.octomap.header.frame_id.c_str());
  ROS_INFO("Current prm_->planning_frame_ is %s", prm_->planning_frame_.c_str());
  // preprocess
  clock_t t_preprocess = clock();
  //prm_->planning_frame_ = planning_scene->world.octomap.header.frame_id;
  grid_->setReferenceFrame(prm_->planning_frame_);
  // TODO: set kinematics to planning frame
  double preprocess_time = (clock() - t_preprocess) / (double)CLOCKS_PER_SEC;

  // plan
  clock_t t_plan = clock();
  res.motion_plan_response.trajectory_start = planning_scene->robot_state;

  if(req.motion_plan_request.goal_constraints.size() == 0){
    ROS_ERROR_PRETTY("No goal constraints in request!");
    return false;
  } 
  if(req.motion_plan_request.goal_constraints.front().position_constraints.size() > 0){
    ROS_INFO_PRETTY("Planning to position!");
    if(!planToPosition(req,res))
      return false;
  } else if(req.motion_plan_request.goal_constraints.front().joint_constraints.size() > 0){
    ROS_INFO_PRETTY("Planning to joint configuration!");
    if(!planToConfiguration(req,res))
      return false;
  } else {
    ROS_ERROR_PRETTY("Both position and joint constraints empty!");
    return false;
  }

  res_ = res;
  double plan_time = (clock() - t_plan) / (double)CLOCKS_PER_SEC;
  ROS_INFO_PRETTY("t_plan: %0.3fsec  t_preprocess: %0.3fsec", plan_time, preprocess_time);
  return true;
}

bool SBPLArmPlannerInterface::setStart(const sensor_msgs::JointState &state)
{
  std::vector<double> initial_positions;
  if(!leatherman::getJointPositions(state, prm_->planning_joints_, initial_positions))
  {
    ROS_ERROR_PRETTY("Start state does not contain the positions of the planning joints.");
    return false;
  }

  if(sbpl_arm_env_->setStartConfiguration(initial_positions) == 0)
  {
    ROS_ERROR_PRETTY("Environment failed to set start state. Not Planning.");
    return false;
  }
  if(planner_->set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR_PRETTY("Failed to set start state. Not Planning.");
    return false;
  }
  ROS_INFO_PRETTY("start: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", initial_positions[0],initial_positions[1],initial_positions[2],initial_positions[3],initial_positions[4],initial_positions[5],initial_positions[6]);
  return true;
}

bool SBPLArmPlannerInterface::setGoalConfiguration(const moveit_msgs::Constraints& goal_constraints)
{
  std::vector <double> sbpl_angle_goal(7, 0);
  std::vector <double> sbpl_angle_tolerance(7,0.05); //~3 degrees tolerance by default

  if (goal_constraints.joint_constraints.size() < 7)
  {
      ROS_WARN_PRETTY("All 7 arm joint constraints must be specified for goal!");
      return false;
  }
  if (goal_constraints.joint_constraints.size() > 7)
  {
      ROS_WARN_PRETTY("%d joint constraints specified! Using the first 7!", (int)goal_constraints.joint_constraints.size());
      return false;
  }
  for(int i = 0; i < (int)std::min(goal_constraints.joint_constraints.size(), sbpl_angle_goal.size()); i++){
    sbpl_angle_goal[i] = goal_constraints.joint_constraints[i].position;
    sbpl_angle_tolerance[i] = 0.5*abs(goal_constraints.joint_constraints[i].tolerance_above) + 0.5*abs(goal_constraints.joint_constraints[i].tolerance_below);
    ROS_INFO_PRETTY("Joint %d [%s]: goal position: %.3f, goal tolerance: %.3f", i, goal_constraints.joint_constraints[i].joint_name.c_str(), sbpl_angle_goal[i], sbpl_angle_tolerance[i]);
  }

  // set sbpl environment goal
  if (!sbpl_arm_env_->setGoalConfiguration(sbpl_angle_goal, sbpl_angle_tolerance))
  {
    ROS_ERROR_PRETTY("Failed to set goal state. Exiting.");
    return false;
  }

  //set planner goal
  if (planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR_PRETTY("Failed to set goal state. Exiting.");
    return false;
  }

  return true;
}

bool SBPLArmPlannerInterface::setGoalPosition(const moveit_msgs::Constraints& goal_constraints)
{
  std::vector<std::vector <double> > sbpl_goal(1, std::vector<double> (11,0));  //Changed to include Quaternion
  std::vector<std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  if (goal_constraints.position_constraints.empty() || goal_constraints.orientation_constraints.empty())
  {
      ROS_WARN_PRETTY("Cannot convert goal constraints without position constraints into goal pose");
      return false;
  }

  if (goal_constraints.position_constraints.front().constraint_region.primitive_poses.empty())
  {
      ROS_WARN_PRETTY("At least one primitive shape pose for goal position constraint regions is required");
      return false;
  }

  geometry_msgs::Pose goal_pose;
  extractGoalPoseFromGoalConstraints(goal_constraints, goal_pose);
  double tolerance[6];
  extractGoalToleranceFromGoalConstraints(goal_constraints, tolerance);

  // currently only supports one goal
  sbpl_goal[0][0] = goal_pose.position.x;
  sbpl_goal[0][1] = goal_pose.position.y;
  sbpl_goal[0][2] = goal_pose.position.z;

  // convert quaternion into roll, pitch, yaw
  geometry_msgs::Quaternion goalq;
  goalq = goal_pose.orientation;
  // perturb quaternion if rpy will suffer from gimbal lock.
  goalq.w += 0.001; // Where is the if here? Won't this possibly perturb it into gimbal lock? - Andrew
  leatherman::getRPY(goalq, sbpl_goal[0][3], sbpl_goal[0][4], sbpl_goal[0][5]);

  // 6dof goal: true, 3dof: false
  sbpl_goal[0][6] = true;

  // orientation constraint as a quaternion
  sbpl_goal[0][7] = goalq.x;
  sbpl_goal[0][8] = goalq.y;
  sbpl_goal[0][9] = goalq.z;
  sbpl_goal[0][10] = goalq.w;

  // allowable tolerance from goal
  sbpl_tolerance[0][0] = tolerance[0];
  sbpl_tolerance[0][1] = tolerance[1];
  sbpl_tolerance[0][2] = tolerance[2];
  sbpl_tolerance[0][3] = tolerance[3];
  sbpl_tolerance[0][4] = tolerance[4];
  sbpl_tolerance[0][5] = tolerance[5];

  ROS_INFO_PRETTY("goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)  (quat: %0.3f %0.3f %0.3f %0.3f)",
           prm_->planning_frame_.c_str(), sbpl_goal[0][0], sbpl_goal[0][1], sbpl_goal[0][2], sbpl_tolerance[0][0], sbpl_goal[0][3], sbpl_goal[0][4], sbpl_goal[0][5], sbpl_tolerance[0][1],
           goal_constraints.orientation_constraints[0].orientation.x, goal_constraints.orientation_constraints[0].orientation.y, goal_constraints.orientation_constraints[0].orientation.z, goal_constraints.orientation_constraints[0].orientation.w);

  // set sbpl environment goal
  if (!sbpl_arm_env_->setGoalPosition(sbpl_goal, sbpl_tolerance))
  {
    ROS_ERROR_PRETTY("Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
    return false;
  }

  //set planner goal
  if (planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR_PRETTY("Failed to set goal state. Exiting.");
    return false;
  }

  return true;
}

bool SBPLArmPlannerInterface::planKinematicPath(
  const moveit_msgs::GetMotionPlan::Request &req,
  moveit_msgs::GetMotionPlan::Response &res)
{
  if (!planner_initialized_)
  {
    ROS_ERROR_PRETTY("Hold up a second...the planner isn't initialized yet. Try again in a second or two.");
    return false;
  }

  if (!planToPosition(req, res))
  {
    return false;
  }

  return true;
}

bool SBPLArmPlannerInterface::plan(trajectory_msgs::JointTrajectory &traj)
{
  ROS_WARN_PRETTY("Planning!!!!!");
  bool b_ret = false;
  std::vector<int> solution_state_ids;

  //reinitialize the search space
  planner_->force_planning_from_scratch();

  //plan
  ReplanParams replan_params(prm_->allowed_time_);
  replan_params.initial_eps = 100.0;
  replan_params.final_eps = 1.0;
  replan_params.dec_eps = 0.2;
  replan_params.return_first_solution = false;
  // replan_params.max_time = prm_->allowed_time_;
  replan_params.repair_time = 1.0;
  b_ret = planner_->replan(&solution_state_ids, replan_params, &solution_cost_);

  //check if an empty plan was received.
  if(b_ret && solution_state_ids.size() <= 0)
  {
    ROS_WARN_PRETTY("Path returned by the planner is empty?");
    b_ret = false;
  }

  // if a path is returned, then pack it into msg form
  if(b_ret && (solution_state_ids.size() > 0))
  {
    ROS_INFO_PRETTY("Initial Epsilon: %0.3f   Final Epsilon: %0.3f  Solution Cost: %d", planner_->get_initial_eps(),planner_->get_final_epsilon(), solution_cost_);

    if(!sbpl_arm_env_->convertStateIDPathToJointTrajectory(solution_state_ids, traj))
      return false;
  }
  return b_ret;
}

bool SBPLArmPlannerInterface::planToPosition(
  const moveit_msgs::GetMotionPlan::Request& req,
  moveit_msgs::GetMotionPlan::Response& res)
{
  starttime = clock();
  int status = 0;
  prm_->allowed_time_ = req.motion_plan_request.allowed_planning_time;
  req_ = req.motion_plan_request;

  if(!canServiceRequest(req))
    return false;

  // transform goal pose into reference_frame
  const std::vector<moveit_msgs::Constraints>& goal_constraints_v = req.motion_plan_request.goal_constraints;
  if (goal_constraints_v.empty()) {
      ROS_WARN_PRETTY("Received a motion plan request without any goal constraints");
      return false;
  }

  const moveit_msgs::Constraints& goal_constraints = goal_constraints_v.front(); // only acknowledge the first constraint

  bool use6dofgoal = true;
  if (goal_constraints.position_constraints.empty() || goal_constraints.orientation_constraints.empty()) {
      ROS_WARN_PRETTY("Received a motion plan request without position or orientation constraints on the goal constraints");
      use6dofgoal = false;
  }
  if ((!use6dofgoal) && goal_constraints.joint_constraints.empty()){
      ROS_WARN_PRETTY("Received a motion plan request without joint constraints. Giving up!");
      return false;
  }

  // set start
  ROS_INFO_PRETTY("Setting start.");

  if(!setStart(req.motion_plan_request.start_state.joint_state))
  {
    status = -1;
    ROS_ERROR_PRETTY("Failed to set initial configuration of robot.");
  }

  if(use6dofgoal){
    // set goal
    ROS_INFO_PRETTY("Setting goal 6dof goal.");
    if(!setGoalPosition(goal_constraints) && status == 0)
    {
      status = -2;
      ROS_ERROR_PRETTY("Failed to set goal position.");
    }
  } else {
    ROS_INFO_PRETTY("Setting goal 7dof goal.");
    if(!setGoalConfiguration(goal_constraints) && status == 0)
    {
      status = -2;
      ROS_ERROR_PRETTY("Failed to set goal position.");
    }
  }

  // plan
  ROS_INFO_PRETTY("Calling planner");
  if(status == 0 && plan(res.motion_plan_response.trajectory.joint_trajectory))
  {
    const moveit_msgs::PositionConstraint position_constraint = goal_constraints.position_constraints.front();
    res.motion_plan_response.trajectory.joint_trajectory.header.seq = position_constraint.header.seq;
    res.motion_plan_response.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    // fill in the waypoint times (not very intelligently)
    res.motion_plan_response.trajectory.joint_trajectory.points[0].time_from_start.fromSec(prm_->waypoint_time_);
    for(size_t i = 1; i < res.motion_plan_response.trajectory.joint_trajectory.points.size(); i++) {
      const double prev_time_s = res.motion_plan_response.trajectory.joint_trajectory.points[i - 1].time_from_start.toSec();
      res.motion_plan_response.trajectory.joint_trajectory.points[i].time_from_start.fromSec(prev_time_s + prm_->waypoint_time_);
    }

    res.motion_plan_response.planning_time = ((clock() - starttime) / (double)CLOCKS_PER_SEC);

    // shortcut path
    if(prm_->shortcut_path_)
    {
      trajectory_msgs::JointTrajectory straj;
      if(!interpolateTrajectory(cc_, res.motion_plan_response.trajectory.joint_trajectory.points, straj.points))
        ROS_WARN_PRETTY("Failed to interpolate planned trajectory with %d waypoints before shortcutting.", int(res.motion_plan_response.trajectory.joint_trajectory.points.size()));

      shortcutTrajectory(cc_, straj.points,res.motion_plan_response.trajectory.joint_trajectory.points);
    }

    // interpolate path
    if(prm_->interpolate_path_)
    {
      trajectory_msgs::JointTrajectory itraj = res.motion_plan_response.trajectory.joint_trajectory;
      interpolateTrajectory(cc_, itraj.points, res.motion_plan_response.trajectory.joint_trajectory.points);
    }

    if(prm_->print_path_) {
      leatherman::printJointTrajectory(res.motion_plan_response.trajectory.joint_trajectory, "path");
    }
  }
  else
  {
    status = -3;
    ROS_ERROR_PRETTY("Failed to plan within alotted time frame (%0.2f seconds).", prm_->allowed_time_);
  }

  if(status == 0)
    return true;

  return false;
}

bool SBPLArmPlannerInterface::planToConfiguration(
  const moveit_msgs::GetMotionPlan::Request& req,
  moveit_msgs::GetMotionPlan::Response& res)
{
  starttime = clock();
  int status = 0;
  prm_->allowed_time_ = req.motion_plan_request.allowed_planning_time;
  req_ = req.motion_plan_request;

  if(!canServiceRequest(req))
    return false;

  // transform goal pose into reference_frame
  const std::vector<moveit_msgs::Constraints>& goal_constraints_v = req.motion_plan_request.goal_constraints;
  if (goal_constraints_v.empty()) {
      ROS_WARN_PRETTY("Received a motion plan request without any goal constraints");
      return false;
  }

  const moveit_msgs::Constraints& goal_constraints = goal_constraints_v.front(); // only acknowledge the first constraint

  bool use6dofgoal = true;
  if (goal_constraints.position_constraints.empty() || goal_constraints.orientation_constraints.empty()) {
      ROS_WARN_PRETTY("Received a motion plan request without position or orientation constraints on the goal constraints");
      use6dofgoal = false;
  }
  if ((!use6dofgoal) && goal_constraints.joint_constraints.empty()){
      ROS_WARN_PRETTY("Received a motion plan request without joint constraints. Giving up!");
      return false;
  }

  // set start
  ROS_INFO_PRETTY("Setting start.");

  if(!setStart(req.motion_plan_request.start_state.joint_state))
  {
    status = -1;
    ROS_ERROR_PRETTY("Failed to set initial configuration of robot.");
  }

  if(use6dofgoal){
    // set goal
    ROS_INFO_PRETTY("Setting goal 6dof goal.");
    if(!setGoalPosition(goal_constraints) && status == 0)
    {
      status = -2;
      ROS_ERROR_PRETTY("Failed to set goal position.");
    }
  } else {
    ROS_INFO_PRETTY("Setting goal 7dof goal.");
    if(!setGoalConfiguration(goal_constraints) && status == 0)
    {
      status = -2;
      ROS_ERROR_PRETTY("Failed to set goal position.");
    }
  }

  // plan
  ROS_INFO_PRETTY("Calling planner");
  if(status == 0 && plan(res.motion_plan_response.trajectory.joint_trajectory))
  {
    const moveit_msgs::JointConstraint joint_constraint = goal_constraints.joint_constraints.front();
    res.motion_plan_response.trajectory.joint_trajectory.header.seq = 0;
    res.motion_plan_response.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    // fill in the waypoint times (not very intelligently)
    res.motion_plan_response.trajectory.joint_trajectory.points[0].time_from_start.fromSec(prm_->waypoint_time_);
    for(size_t i = 1; i < res.motion_plan_response.trajectory.joint_trajectory.points.size(); i++) {
      const double prev_time_s = res.motion_plan_response.trajectory.joint_trajectory.points[i - 1].time_from_start.toSec();
      res.motion_plan_response.trajectory.joint_trajectory.points[i].time_from_start.fromSec(prev_time_s + prm_->waypoint_time_);
    }

    res.motion_plan_response.planning_time = ((clock() - starttime) / (double)CLOCKS_PER_SEC);

    // shortcut path
    if(prm_->shortcut_path_)
    {
      trajectory_msgs::JointTrajectory straj;
      if(!interpolateTrajectory(cc_, res.motion_plan_response.trajectory.joint_trajectory.points, straj.points))
        ROS_WARN_PRETTY("Failed to interpolate planned trajectory with %d waypoints before shortcutting.", int(res.motion_plan_response.trajectory.joint_trajectory.points.size()));

      shortcutTrajectory(cc_, straj.points,res.motion_plan_response.trajectory.joint_trajectory.points);
    }

    // interpolate path
    if(prm_->interpolate_path_)
    {
      trajectory_msgs::JointTrajectory itraj = res.motion_plan_response.trajectory.joint_trajectory;
      interpolateTrajectory(cc_, itraj.points, res.motion_plan_response.trajectory.joint_trajectory.points);
    }

    if(prm_->print_path_) {
      leatherman::printJointTrajectory(res.motion_plan_response.trajectory.joint_trajectory, "path");
    }
  }
  else
  {
    status = -3;
    ROS_ERROR_PRETTY("Failed to plan within alotted time frame (%0.2f seconds).", prm_->allowed_time_);
  }

  if(status == 0)
    return true;

  return false;
}

bool SBPLArmPlannerInterface::canServiceRequest(const moveit_msgs::GetMotionPlan::Request &req)
{
  // check for an empty start state
  if(req.motion_plan_request.start_state.joint_state.position.empty())
  {
    ROS_ERROR_PRETTY("No start state given. Unable to plan.");
    return false;
  }

  if (req.motion_plan_request.goal_constraints.empty()) {
      ROS_ERROR_PRETTY("Goal constraints are empty. Expecting at least one goal constraints with pose and orientation constraints");
      return false;
  }

  // check if position & orientation constraints is empty
  const moveit_msgs::Constraints& goal_constraints = req.motion_plan_request.goal_constraints.front();

  if((goal_constraints.position_constraints.empty() || goal_constraints.orientation_constraints.empty()) && goal_constraints.joint_constraints.empty())
  {
    ROS_ERROR_PRETTY("Position or orientation constraint is empty.");
    ROS_ERROR_PRETTY("Joint constraint is empty.");
    ROS_ERROR_PRETTY("Expecting a 6D end effector pose constraint or 7D joint constraint. Exiting.");
    return false;
  }

  // check if there is more than one goal constraint
  if(goal_constraints.position_constraints.size() > 1 || goal_constraints.orientation_constraints.size() > 1) {
    ROS_WARN_PRETTY("The planning request message contains %zd position and %zd orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", goal_constraints.position_constraints.size(), goal_constraints.orientation_constraints.size());
  }

  return true;
}

std::map<std::string, double>  SBPLArmPlannerInterface::getPlannerStats()
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

visualization_msgs::MarkerArray SBPLArmPlannerInterface::getCollisionModelTrajectoryMarker()
{
  visualization_msgs::MarkerArray ma, ma1;
  std::vector<std::vector<double> > traj;

  if(res_.motion_plan_response.trajectory.joint_trajectory.points.empty())
  {
    ROS_ERROR_PRETTY("No trajectory found to visualize yet. Plan a path first.");
    return ma;
  }

  traj.resize(res_.motion_plan_response.trajectory.joint_trajectory.points.size());
  double cinc = 1.0/double(res_.motion_plan_response.trajectory.joint_trajectory.points.size());
  for(size_t i = 0; i < res_.motion_plan_response.trajectory.joint_trajectory.points.size(); ++i)
  {
    traj[i].resize(res_.motion_plan_response.trajectory.joint_trajectory.points[i].positions.size());
    for(size_t j = 0; j < res_.motion_plan_response.trajectory.joint_trajectory.points[i].positions.size(); j++)
      traj[i][j] = res_.motion_plan_response.trajectory.joint_trajectory.points[i].positions[j];

    ma1 = cc_->getCollisionModelVisualization(traj[i]);

    for(size_t j = 0; j < ma1.markers.size(); ++j)
    {
      ma1.markers[j].color.r = 0.1;
      ma1.markers[j].color.g = cinc*double(res_.motion_plan_response.trajectory.joint_trajectory.points.size()-(i+1));
      ma1.markers[j].color.b = cinc*double(i);
    }
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
  }

  for(size_t i = 0; i < ma.markers.size(); ++i)
  {
    ma.markers[i].ns = "trajectory";
    ma.markers[i].id = i;
  }

  return ma;
}

visualization_msgs::MarkerArray SBPLArmPlannerInterface::getVisualization(std::string type)
{
  visualization_msgs::MarkerArray ma;
  if(type.compare("goal") == 0)
  {
    const moveit_msgs::Constraints& goal_constraints = req_.goal_constraints.front();
    if(goal_constraints.position_constraints.empty())
    {
      ROS_WARN_PRETTY("Failed to get visualization marker for goals because no position constraints found.");
      return visualization_msgs::MarkerArray();
    }

    // compute space needed for goal poses
    int num_goal_pos_constraints = 0;
    for (int i = 0; i < goal_constraints.position_constraints.size(); ++i) {
      const moveit_msgs::PositionConstraint& pos_constraint = goal_constraints.position_constraints[i];
      num_goal_pos_constraints += pos_constraint.constraint_region.primitive_poses.size();
    }
    std::vector<std::vector<double> > poses(num_goal_pos_constraints,std::vector<double>(6,0));

    for(size_t i = 0; i < goal_constraints.position_constraints.size(); ++i)
    {
      const moveit_msgs::PositionConstraint& pos_constraint = goal_constraints.position_constraints[i];
      for (size_t j = 0; j < pos_constraint.constraint_region.primitive_poses.size(); ++j) {
        size_t idx = i * pos_constraint.constraint_region.primitive_poses.size() + j;

        poses[idx][0] = goal_constraints.position_constraints[i].constraint_region.primitive_poses[j].position.x;
        poses[idx][1] = goal_constraints.position_constraints[i].constraint_region.primitive_poses[j].position.y;
        poses[idx][2] = goal_constraints.position_constraints[i].constraint_region.primitive_poses[j].position.z;

        if(goal_constraints.orientation_constraints.size() > i) {
          leatherman::getRPY(goal_constraints.orientation_constraints[i].orientation, poses[idx][3], poses[idx][4], poses[idx][5]);
        }
        else
        {
          poses[idx][3] = 0;
          poses[idx][4] = 0;
          poses[idx][5] = 0;
        }
      }
    }
    ma = viz::getPosesMarkerArray(poses, goal_constraints.position_constraints[0].header.frame_id, "goals", 0);
  }
  else if(type.compare("expansions") == 0)
  {
    std::vector<std::vector<double> > expanded_states;
    sbpl_arm_env_->getExpandedStates(&(expanded_states));
    if(!expanded_states.empty())
    {
      std::vector<std::vector<double> > colors(2, std::vector<double>(4,0));
      colors[0][0] = 1;
      colors[0][3] = 1;
      colors[1][1] = 1;
      colors[1][3] = 1;
      ROS_ERROR_PRETTY("Expansions visualization %d expands, %s frame", int(expanded_states.size()), prm_->planning_frame_.c_str());
      ma = viz::getCubesMarkerArray(expanded_states, 0.01, colors, prm_->planning_frame_, "expansions", 0);
    }
  }
  else
    ma = sbpl_arm_env_->getVisualization(type);

  return ma;
}

void SBPLArmPlannerInterface::extractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    geometry_msgs::Pose& goal_pose_out) const
{
    assert(!goal_constraints.position_constraints.empty() && !goal_constraints.orientation_constraints.empty());

    const moveit_msgs::PositionConstraint& position_constraint = goal_constraints.position_constraints.front();
    const moveit_msgs::OrientationConstraint& orientation_constraint = goal_constraints.orientation_constraints.front();

    assert(!position_constraint.constraint_region.primitive_poses.empty());

    const shape_msgs::SolidPrimitive& bounding_primitive = position_constraint.constraint_region.primitives.front();
    const geometry_msgs::Pose& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    goal_pose_out.position = primitive_pose.position;
    goal_pose_out.orientation = orientation_constraint.orientation;
}

void SBPLArmPlannerInterface::extractGoalToleranceFromGoalConstraints(const moveit_msgs::Constraints& goal_constraints,  double* tolerance_out)
{
    if (!goal_constraints.position_constraints.empty() && !goal_constraints.position_constraints.front().constraint_region.primitives.empty()) {
        const moveit_msgs::PositionConstraint& position_constraint = goal_constraints.position_constraints.front();
        const shape_msgs::SolidPrimitive& constraint_primitive = position_constraint.constraint_region.primitives.front();
        switch (constraint_primitive.type) {
        case shape_msgs::SolidPrimitive::BOX:
            tolerance_out[0] = constraint_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X];
            tolerance_out[1] = constraint_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
            tolerance_out[2] = constraint_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            tolerance_out[0] = tolerance_out[1] = tolerance_out[2] = constraint_primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            tolerance_out[0] = tolerance_out[1] = tolerance_out[2] = constraint_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CONE:
            tolerance_out[0] = tolerance_out[1] = tolerance_out[2] = constraint_primitive.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS];
            break;
        }
    }
    else {
        tolerance_out[0] = tolerance_out[1] = tolerance_out[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        const moveit_msgs::OrientationConstraint& orientation_constraint = goal_constraints.orientation_constraints.front();
        tolerance_out[3] = orientation_constraint.absolute_x_axis_tolerance;
        tolerance_out[4] = orientation_constraint.absolute_y_axis_tolerance;
        tolerance_out[5] = orientation_constraint.absolute_z_axis_tolerance;
    }
    else {
        tolerance_out[3] = tolerance_out[4] = tolerance_out[5] = 0.0;
    }
}

