#include <sbpl_manipulation_components/collision_checker.h>

namespace sbpl_arm_planner {

CollisionChecker::CollisionChecker()
{
}

CollisionChecker::~CollisionChecker()
{
}

visualization_msgs::MarkerArray CollisionChecker::getCollisionModelVisualization(const std::vector<double> &angles)
{
  ROS_ERROR("Function is not filled in.");
  return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray CollisionChecker::getVisualization(std::string type)
{
  ROS_ERROR("Function is not filled in.");
  return visualization_msgs::MarkerArray();
}

}
