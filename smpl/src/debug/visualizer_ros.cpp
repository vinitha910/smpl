#include <smpl/debug/visualizer_ros.h>

namespace sbpl {

VisualizerROS::VisualizerROS(
    const ros::NodeHandle& nh,
    size_t queue_size)
:
    m_nh(nh)
{
    m_pub = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", queue_size);

    XmlRpc::XmlRpcValue disabled_value;
    if (nh.getParam("disabled_namespaces", disabled_value)) {
        if (disabled_value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < disabled_value.size(); ++i) {
                XmlRpc::XmlRpcValue& e = disabled_value[i];
                if (e.getType() != XmlRpc::XmlRpcValue::TypeString) {
                    ROS_WARN("'disabled_namespaces' element is not a string. skipping.");
                    continue;
                }
                m_disabled.emplace_back((std::string&)e);
            }
        }
        else {
            ROS_WARN("param 'disabled_namespaces' is not a vector");
        }
    }
}

void VisualizerROS::visualize(
    sbpl::viz::levels::Level level,
    const visualization_msgs::MarkerArray& markers)
{
    size_t exclude_count = 0;
    m_match_index.assign(markers.markers.size(), -1);
    boost::smatch sm;
    for (size_t i = 0; i < markers.markers.size(); ++i) {
        const visualization_msgs::Marker& m = markers.markers[i];
        for (size_t j = 0; j < m_disabled.size(); ++j) {
            const boost::regex& r = m_disabled[j];
            if (boost::regex_match(m.ns, sm, r)) {
                m_match_index[i] = j;
                ++exclude_count;
                break;
            }
        }
    }

    if (exclude_count == 0) {
        // include all markers
        m_pub.publish(markers);
    }
    else if (exclude_count == markers.markers.size()) {
        // exclude all markers
        return;
    }
    else {
        m_enabled.markers.clear();
        for (size_t i = 0; i < markers.markers.size(); ++i) {
            const visualization_msgs::Marker& m = markers.markers[i];
            if (m_match_index[i] == -1) {
                m_enabled.markers.push_back(m);
            }
        }
        m_pub.publish(m_enabled);
    }
}

} // namespace sbpl
