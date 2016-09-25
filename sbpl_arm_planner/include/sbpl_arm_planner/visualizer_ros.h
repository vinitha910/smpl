#ifndef sbpl_ros_visualizer_h
#define sbpl_ros_visualizer_h

// standard includes
#include <stdlib.h>

// system includes
#include <ros/ros.h>

// project includes
#include <sbpl_arm_planner/visualize.h>

namespace sbpl {
namespace ros {

class VisualizerROS : public viz::VisualizerBase
{
public:

    VisualizerROS(size_t queue_size = 100);

    void visualize(
        sbpl::viz::levels::Level level,
        const visualization_msgs::MarkerArray& markers);

private:

    ::ros::NodeHandle m_nh;
    ::ros::Publisher m_pub;
};

} // namespace ros
} // namespace sbpl

#endif

