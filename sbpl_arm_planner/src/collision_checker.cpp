#include <sbpl_arm_planner/collision_checker.h>

#include <ros/console.h>

namespace sbpl {
namespace manip {

CollisionChecker::CollisionChecker()
{
}

CollisionChecker::~CollisionChecker()
{
}

visualization_msgs::MarkerArray
CollisionChecker::getCollisionModelVisualization(
    const std::vector<double> &angles)
{
    ROS_ERROR("Function is not filled in.");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray CollisionChecker::getVisualization(
    const std::string& type)
{
    ROS_ERROR("Function is not filled in.");
    return visualization_msgs::MarkerArray();
}

} // namespace manip
} // namespace sbpl
