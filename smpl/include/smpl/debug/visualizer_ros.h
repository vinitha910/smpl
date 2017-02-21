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

#ifndef SMPL_VISUALIZER_ROS_H
#define SMPL_VISUALIZER_ROS_H

// standard includes
#include <stdlib.h>
#include <unordered_set>

// system includes
#include <boost/regex.hpp>
#include <ros/ros.h>

// project includes
#include <smpl/debug/visualize.h>

namespace sbpl {

class VisualizerROS : public viz::VisualizerBase
{
public:

    VisualizerROS(
        const ros::NodeHandle& nh = ros::NodeHandle(),
        size_t queue_size = 100);

    void visualize(
        sbpl::viz::levels::Level level,
        const visualization_msgs::MarkerArray& markers);

private:

    ros::NodeHandle m_nh;
    ros::Publisher m_pub;

//    std::unordered_set<std::string> m_disabled;
    std::vector<boost::regex> m_disabled;

    std::vector<int> m_match_index;

    visualization_msgs::MarkerArray m_enabled;
};

} // namespace sbpl

#endif

