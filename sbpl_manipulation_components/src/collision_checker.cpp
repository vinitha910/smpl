#include <sbpl_manipulation_components/collision_checker.h>

namespace sbpl_arm_planner {

CollisionChecker::CollisionChecker()
{
}

CollisionChecker::~CollisionChecker()
{
}

bool CollisionChecker::isStateValid(const std::vector<double> &angles, bool verbose, bool visualize, double &dist)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::isStateToStateValid(const std::vector<double> &angles0, const std::vector<double> &angles1, int path_length, int num_checks, double &dist)
{
  ROS_ERROR("Function is not filled in.");
  return false;
}

bool CollisionChecker::interpolatePath(const std::vector<double> &start, const std::vector<double> &end, const std::vector<double> &inc, std::vector<std::vector<double> > &path)
{
  ROS_ERROR("Function is not filled in.");
  return false;
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
