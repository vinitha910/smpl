#include <cmath>
#include <thread>

#include <ros/ros.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>

void VisualizeCube()
{
    auto beginning = ros::Time::now();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        auto now = ros::Time::now();

        auto make_cube_marker = [&]() -> visualization_msgs::MarkerArray {
            ROS_INFO("make cube marker");
            // make cube marker
            visualization_msgs::MarkerArray ma;
            visualization_msgs::Marker cube_marker;
            cube_marker.header.stamp = now;
            cube_marker.header.frame_id = "world";
            cube_marker.ns = "cube";
            cube_marker.id = 0;
            cube_marker.type = visualization_msgs::Marker::CUBE;
            cube_marker.action = visualization_msgs::Marker::ADD;
            cube_marker.pose.orientation.w = 1.0;
            cube_marker.pose.position.x = std::sin((now - beginning).toSec());
            cube_marker.scale.x = cube_marker.scale.y = cube_marker.scale.z = 1.0;
            cube_marker.color.r = cube_marker.color.a = 1.0;
            cube_marker.lifetime = ros::Duration(0);
            ma.markers.push_back(cube_marker);
            return ma;
        };

        SV_SHOW_INFO(make_cube_marker());

        loop_rate.sleep();
    }
}

void VisualizeSphere()
{
    auto beginning = ros::Time::now();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        auto now = ros::Time::now();

        auto make_sphere_marker = [&]() -> visualization_msgs::MarkerArray {
            ROS_INFO("make sphere marker");
            // make sphere marker
            visualization_msgs::MarkerArray ma;
            visualization_msgs::Marker sphere_marker;
            sphere_marker.header.stamp = now;
            sphere_marker.header.frame_id = "world";
            sphere_marker.ns = "sphere";
            sphere_marker.id = 0;
            sphere_marker.type = visualization_msgs::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::Marker::ADD;
            sphere_marker.pose.orientation.w = 1.0;
            sphere_marker.pose.position.y = std::sin((now - beginning).toSec());
            sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 1.0;
            sphere_marker.color.g = sphere_marker.color.a = 1.0;
            sphere_marker.lifetime = ros::Duration(0);
            ma.markers.push_back(sphere_marker);
            return ma;
        };

        SV_SHOW_INFO(make_sphere_marker());

        loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "debug_vis_demo");
    ros::NodeHandle nh;

    sbpl::VisualizerROS visualizer;
    sbpl::viz::set_visualizer(&visualizer);

    std::thread cube_vis_thread(VisualizeCube);
    std::thread sphere_vis_thread(VisualizeSphere);

    ros::Rate loop_rate(1.0);
    bool enabled = true;
    while (ros::ok()) {
        if (enabled) {
            sbpl::viz::set_visualization_level(SV_ROOT_VIZ_NAME, sbpl::viz::levels::Warn);
        } else {
            sbpl::viz::set_visualization_level(SV_ROOT_VIZ_NAME, sbpl::viz::levels::Info);
        }
        enabled = !enabled;

        loop_rate.sleep();
    }

    cube_vis_thread.join();
    sphere_vis_thread.join();

    sbpl::viz::unset_visualizer();
    return 0;
}
