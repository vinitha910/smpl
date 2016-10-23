#ifndef sbpl_ros_visualizer_h
#define sbpl_ros_visualizer_h

// standard includes
#include <stdlib.h>
#include <unordered_set>

// system includes
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

    std::unordered_set<std::string> m_disabled;

    visualization_msgs::MarkerArray m_enabled;
};

} // namespace sbpl

#endif

