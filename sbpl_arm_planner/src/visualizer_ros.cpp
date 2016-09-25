#include <sbpl_arm_planner/visualizer_ros.h>

namespace sbpl {
namespace ros {

VisualizerROS::VisualizerROS(size_t queue_size)
{
    m_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", queue_size);
}

void VisualizerROS::visualize(
    sbpl::viz::levels::Level level,
    const visualization_msgs::MarkerArray& markers)
{
    m_pub.publish(markers);
}

} // namespace ros
} // namespace sbpl
