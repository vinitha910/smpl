////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2012, Benjamin Cohen
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

// standard includes
#include <stdlib.h>
#include <string>
#include <vector>

// system includes
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <sbpl_arm_planner/arm_planner_interface.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <sbpl_pr2_robot_model/pr2_kdl_robot_model.h>
#include <sbpl_pr2_robot_model/ubr1_kdl_robot_model.h>
#include <visualization_msgs/MarkerArray.h>

void fillConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

//    goals.position_constraints[0].position.x = pose[0];
//    goals.position_constraints[0].position.y = pose[1];
//    goals.position_constraints[0].position.z = pose[2];

    leatherman::rpyToQuatMsg(pose[3], pose[4], pose[5], goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

moveit_msgs::CollisionObject getCollisionCube(
    geometry_msgs::Pose pose,
    std::vector<double>& dims,
    std::string frame_id,
    std::string id)
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

std::vector<moveit_msgs::CollisionObject> getCollisionCubes(
    std::vector<std::vector<double> > &objects,
    std::vector<std::string> &object_ids,
    std::string frame_id)
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(getCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

std::vector<moveit_msgs::CollisionObject> getCollisionObjects(
    std::string filename,
    std::string frame_id)
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    char* file = new char[filename.length()+1];
    filename.copy(file, filename.length(),0);
    file[filename.length()] = '\0';
    FILE* fCfg = fopen(file, "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return getCollisionCubes(objects, object_ids, frame_id);
}

bool getInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state)
{
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() == 0) {
            return false;
        }
        std::cout << xlist << std::endl;
        for (int i = 0; i < xlist.size(); ++i) {
            state.joint_state.name.push_back(std::string(xlist[i]["name"]));

            if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                state.joint_state.position.push_back(double(xlist[i]["position"]));
            }
            else {
                ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                    int pos = xlist[i]["position"];
                    state.joint_state.position.push_back(double(pos));
                }
            }
        }
    }
    else {
        ROS_ERROR("initial_configuration/joint_state is not on the param server.");
        return false;
    }

    //multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }

        if (xlist.size() != 0) {
            geometry_msgs::Pose pose;
            state.multi_dof_joint_state.header.frame_id = std::string(xlist[0]["frame_id"]);
            state.multi_dof_joint_state.joint_names.resize(xlist.size());
            state.multi_dof_joint_state.transforms.resize(xlist.size());
            for (int i = 0; i < xlist.size(); ++i) {
                state.multi_dof_joint_state.joint_names[i] = "world_pose";
                pose.position.x = xlist[i]["x"];
                pose.position.y = xlist[i]["y"];
                pose.position.z = xlist[i]["z"];
                leatherman::rpyToQuatMsg(xlist[i]["roll"], xlist[i]["pitch"], xlist[i]["yaw"], pose.orientation);
                state.multi_dof_joint_state.transforms[i].translation.x = pose.position.x;
                state.multi_dof_joint_state.transforms[i].translation.y = pose.position.y;
                state.multi_dof_joint_state.transforms[i].translation.z = pose.position.z;
                state.multi_dof_joint_state.transforms[i].rotation.w = pose.orientation.w;
                state.multi_dof_joint_state.transforms[i].rotation.x = pose.orientation.x;
                state.multi_dof_joint_state.transforms[i].rotation.y = pose.orientation.y;
                state.multi_dof_joint_state.transforms[i].rotation.z = pose.orientation.z;
            }
        }
    }
    return true;
}

bool ParsePlanningJointGroup(
    const ros::NodeHandle& nh,
    std::vector<std::string>& joint_names)
{
    XmlRpc::XmlRpcValue xlist;
    if (!nh.getParam("planning/planning_joints", xlist)) {
        return false;
    }
    std::string joint_list = std::string(xlist);
    std::stringstream joint_name_stream(joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        joint_names.push_back(jname);
    }
    return true;
}

std::unique_ptr<sbpl::manip::RobotModel>
SetupRobotModel(
    const ros::NodeHandle& nh,
    const std::string& urdf,
    const std::string& group_name,
    const std::vector<std::string>& planning_joints,
    const std::string& planning_link,
    const std::string& planning_frame)
{
    std::unique_ptr<sbpl::manip::RobotModel> rm_ptr;

    sbpl::manip::KDLRobotModel* rm = nullptr;
    KDL::Frame f;
    if (group_name == "right_arm") {
        sbpl::manip::PR2KDLRobotModel* pr2_rm = new sbpl::manip::PR2KDLRobotModel();
        // Set the current transform from the planning frame to the kinematics frame
        f.p.x(-0.05);
        f.p.y(1.0);
        f.p.z(0.789675);
        f.M = KDL::Rotation::Quaternion(0,0,0,1);
        pr2_rm->setKinematicsToPlanningTransform(f, planning_frame);
        rm = pr2_rm;
    }
    else if (group_name == "arm") {
        sbpl::manip::UBR1KDLRobotModel* ubr1_rm = new sbpl::manip::UBR1KDLRobotModel();
        // Set the current transform from the planning frame to the kinematics frame
        f.p.x(-0.05);
        f.p.y(0.0);
        f.p.z(0.26);
        f.M = KDL::Rotation::Quaternion(0,0,0,1);
        ubr1_rm->setKinematicsToPlanningTransform(f, planning_frame);
        rm = ubr1_rm;
    }
    else {
        std::string kinematics_frame, chain_tip_link;
        if (!nh.getParam("kinematics_frame", kinematics_frame) ||
            !nh.getParam("chain_tip_link", chain_tip_link))
        {
            ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
            return false;
        }
        rm  = new sbpl::manip::KDLRobotModel(kinematics_frame, chain_tip_link);
    }

    if (!rm->init(urdf, planning_joints)) {
        ROS_ERROR("Failed to initialize robot model.");
        delete rm;
        rm = nullptr;
    }

    rm->setPlanningLink(planning_link);

    rm_ptr.reset(rm);
    return rm_ptr;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sbpl_arm_planner_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    sleep(1);

    ros::spinOnce();
    ros::Publisher ma_pub = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_marker_array", 100);

    // everyone needs to know the name of the planning frame for reasons...
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    /////////////////
    // Robot Model //
    /////////////////

    // retrieve the urdf
    std::string urdf;
    if (!nh.getParam("robot_description", urdf)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    // retrieve the planning group name, joints, and link; this is used to setup
    // the specific robot we are planning for and is coupled with the group
    // specified in the configuration for the collision checker
    std::string group_name;
    if (!ph.getParam("group_name", group_name)) {
        ROS_ERROR("Failed to retrieve param 'group_name' from the param server");
        return 1;
    }

    std::vector<std::string> planning_joints;
    if (!ParsePlanningJointGroup(ph, planning_joints)) {
        ROS_ERROR("Failed to parse planning joint group");
        return 1;
    }

    std::string planning_link;
    if (!ph.getParam("planning_link", planning_link)) {
        ROS_ERROR("Failed to retrieve param 'planning_link' from the param server");
        return 1;
    }

    std::vector<double> start_angles(planning_joints.size(), 0);
    if (planning_joints.size() < 7) {
        ROS_ERROR("Found %zu planning joints on the param server. I usually expect at least 7 joints...", planning_joints.size());
    }

    auto rm = SetupRobotModel(
            ph, urdf, group_name, planning_joints, planning_link, planning_frame);

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    const double df_size_x = 3.0;
    const double df_size_y = 3.0;
    const double df_size_z = 3.0;
    const double df_res = 0.02;
    const double df_origin_x = -0.75;
    const double df_origin_y = -1.25;
    const double df_origin_z = 1.0;
    const double max_distance = 0.2;

    auto df = std::make_shared<distance_field::PropagationDistanceField>(
            df_size_x, df_size_y, df_size_z,
            df_res,
            df_origin_x, df_origin_y, df_origin_z,
            max_distance);
    df->reset();

    sbpl::OccupancyGrid grid(df);
    grid.setReferenceFrame(planning_frame);

    ///////////////////////
    // Collision Checker //
    ///////////////////////

    sbpl::collision::CollisionModelConfig cc_conf;
    if (!sbpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    sbpl::collision::CollisionSpaceBuilder builder;
    auto cc = builder.build(&grid, urdf, cc_conf, group_name, planning_joints);
    if (!cc) {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    ////////////////
    // Action Set //
    ////////////////

    std::string action_set_filename;
    if (!ph.getParam("action_set_filename", action_set_filename)) {
        ROS_ERROR("Failed to retrieve param 'action_set_filename' from the param server");
        return 1;
    }

    ////////////////////////
    // Planning Interface //
    ////////////////////////

    // planner interface
    sbpl::manip::MotionPlannerInterface planner(rm.get(), cc.get(), &grid);

    sbpl::manip::PlanningParams params;
    params.action_file = action_set_filename;
    if (!planner.init(params)) {
        ROS_ERROR("Failed to initialize Arm Planner Interface");
        return 1;
    }

    ///////////////////////
    // Environment Setup //
    ///////////////////////

    std::string object_filename;
    ph.param<std::string>("object_filename", object_filename, "");

    // collision objects
    moveit_msgs::PlanningScenePtr scene(new moveit_msgs::PlanningScene);
    if (!object_filename.empty()) {
        scene->world.collision_objects = getCollisionObjects(object_filename, planning_frame);
    }

    // fill start state
    if (!getInitialConfiguration(ph, scene->robot_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 0;
    }
    scene->robot_state.joint_state.header.frame_id = planning_frame;

    // set planning scene
    cc->setPlanningScene(*scene);

    //////////////
    // Planning //
    //////////////

    std::vector<double> pose(6, 0);
    std::vector<double> goal(6, 0);
    ph.param("goal/x", goal[0], 0.0);
    ph.param("goal/y", goal[1], 0.0);
    ph.param("goal/z", goal[2], 0.0);
    ph.param("goal/roll", goal[3], 0.0);
    ph.param("goal/pitch", goal[4], 0.0);
    ph.param("goal/yaw", goal[5], 0.0);

    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    // fill start state
    req.start_state = scene->robot_state;

    // fill goal state
    req.goal_constraints.resize(1);
    fillConstraint(goal, planning_frame, req.goal_constraints[0]);
    req.allowed_planning_time = 60.0;

    // plan
    ROS_INFO("Calling solve...");
    if (!planner.solve(scene, req, res)) {
        ROS_ERROR("Failed to plan.");
    }
    else {
        ma_pub.publish(planner.getCollisionModelTrajectoryMarker());
    }

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    std::vector<std::string> statistic_names = {
            "initial solution planning time",
            "initial epsilon",
            "initial solution expansions",
            "final epsilon planning time",
            "final epsilon",
            "solution epsilon",
            "expansions",
            "solution cost"
    };
    std::map<std::string, double> planning_stats = planner.getPlannerStats();

    ROS_INFO("Planning statistics");
    for (const auto& statistic : statistic_names) {
        auto it = planning_stats.find(statistic);
        if (it != planning_stats.end()) {
            ROS_INFO("    %s: %0.3f", statistic.c_str(), it->second);
        }
        else {
            ROS_WARN("Did not find planning statistic \"%s\"", statistic.c_str());
        }
    }

    // visualizations
    ros::spinOnce();
    ma_pub.publish(cc->getVisualization("bounds"));
    ma_pub.publish(cc->getVisualization("distance_field"));
    ma_pub.publish(planner.getVisualization("goal"));
    ma_pub.publish(planner.getVisualization("expansions"));
    ma_pub.publish(cc->getVisualization("collision_objects"));
    ma_pub.publish(cc->getCollisionModelVisualization(start_angles));

    ros::spinOnce();

    sleep(1);
    ROS_INFO("Done");
    return 0;
}
