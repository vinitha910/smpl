////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
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
//     * Neither the name of the University of Pennsylvania nor the names of its
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

#include <iostream>

#include <sbpl_collision_checking/test_sbpl_collision_space.h>

using namespace sbpl_arm_planner;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_sbpl_collision_checking");
    ros::NodeHandle nh_, ph_("~");
    int num_checks = 100;
    std::string group_name = "right_arm";

    if (argc > 1) {
        num_checks = atoi(argv[1]);
    }

    // set up the grid
    OccupancyGrid* grid_ =
            new OccupancyGrid(1.7, 1.9, 2.0, 0.01, -0.6, -1.25, -0.05, 0.40);

    // create the collision space
    sbpl::collision::CollisionSpace* cspace_ = new sbpl::collision::CollisionSpace(grid_);
    //cspace_->setPlanningJoints(prms_.planning_joints_);
    cspace_->init(group_name);

    // get map
    moveit_msgs::CollisionMapConstPtr map =
            ros::topic::waitForMessage<moveit_msgs::CollisionMap>(
                    "collision_map_occ");
    grid_->updateFromCollisionMap(*map);

    std::vector<double> angles(7,0);
    unsigned char dist = 100;

    ROS_INFO("Starting %d collision checks.", num_checks);
    clock_t start = clock();
    for (int i = 0; i < num_checks; ++i) {
        if (!cspace_->checkCollision(angles, false, false, dist)) {
            ROS_INFO("In Collision");
            break;
        }
    }
    double total_time = (clock()-start)/double(CLOCKS_PER_SEC);
    ROS_INFO("       total time: %0.8fsec", total_time);
    ROS_INFO("   time per check: %0.8fsec", total_time/double(num_checks));
    ROS_INFO("checks per second: %0.8fsec", 1.0 / (total_time/double(num_checks)));

    ROS_INFO("[sanity check] # calls: %d   # calls not in collision: %d", cspace_->num_collision_checks_, cspace_->num_false_collision_checks_);

    delete cspace_;
    delete grid_;
    return 0;
}

