////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
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
#include <iostream>

// project includes
#include <sbpl_collision_checking/test_sbpl_collision_space.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_sbpl_collision_checking");
    ros::NodeHandle nh_;
    ros::NodeHandle ph_("~");
    int num_checks = 100;
    std::string group_name = "right_arm";

    if (argc > 1) {
        num_checks = atoi(argv[1]);
    }

    // set up the grid
    double size_x = 1.7;
    double size_y = 1.9;
    double size_z = 2.0;
    double res = 0.01;
    double origin_x = -0.6;
    double origin_y = -1.25;
    double origin_z = -0.05;
    double max_dist = 0.40;
    sbpl::OccupancyGrid grid(
            size_x, size_y, size_z,
            res,
            origin_x, origin_y, origin_z,
            max_dist);

    // create the collision space
    sbpl::collision::CollisionSpace cspace(&grid);

    // cspace->setPlanningJoints(prms_.planning_joints_);
    if (!cspace.init(group_name)) {
        return 1;
    }

    // get map
    moveit_msgs::CollisionMapConstPtr map =
            ros::topic::waitForMessage<moveit_msgs::CollisionMap>(
                    "collision_map_occ");
    grid.updateFromCollisionMap(*map);

    std::vector<double> angles(7,0);
    unsigned char dist = 100;

    ROS_INFO("Starting %d collision checks.", num_checks);
    clock_t start = clock();
    for (int i = 0; i < num_checks; ++i) {
        if (!cspace.checkCollision(angles, false, false, dist)) {
            ROS_INFO("In Collision");
            break;
        }
    }
    double total_time = (clock() - start) / double(CLOCKS_PER_SEC);
    ROS_INFO("       total time: %0.8fsec", total_time);
    ROS_INFO("   time per check: %0.8fsec", total_time/double(num_checks));
    ROS_INFO("checks per second: %0.8fsec", 1.0 / (total_time/double(num_checks)));

    ROS_INFO("[sanity check] # calls: %d   # calls not in collision: %d", cspace.num_collision_checks_, cspace->num_false_collision_checks_);

    return 0;
}

