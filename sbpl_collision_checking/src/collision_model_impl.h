#ifndef CollisionModelImpl_h
#define CollisionModelImpl_h

#include <map>
#include <string>
#include <vector>
#include <kdl/kdl.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MultiDOFJointState.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/group.h>

namespace sbpl
{
namespace manipulation
{

class CollisionModelImpl
{
public:

    CollisionModelImpl();
    ~CollisionModelImpl();

    bool init(const std::string &urdf_string);

    bool initAllGroups();
    void getGroupNames(std::vector<std::string> &names);

    void getDefaultGroupSpheres(std::vector<sbpl_arm_planner::Sphere*> &spheres);
    bool setDefaultGroup(const std::string &group_name);
    bool computeDefaultGroupFK(const std::vector<double> &angles, std::vector<std::vector<KDL::Frame>> &frames);

    sbpl_arm_planner::Group* getGroup(const std::string &name);
    void getVoxelGroups(std::vector<sbpl_arm_planner::Group*> &vg);
    bool getJointLimits(
            const std::string &group_name,
            const std::string &joint_name,
            double &min_limit,
            double &max_limit,
            bool &continuous);
    bool getFrameInfo(const std::string &name, const std::string &group_name, int &chain, int &segment);
    std::string getReferenceFrame(const std::string &group_name);
    void setOrderOfJointPositions(const std::vector<std::string> &joint_names, const std::string &group_name);
    bool doesLinkExist(const std::string &name, const std::string &group_name);

    bool computeGroupFK(
            const std::vector<double> &angles,
            sbpl_arm_planner::Group *group,
            std::vector<std::vector<KDL::Frame>> &frames);

    void setJointPosition(const std::string &name, double position);
    bool setWorldToModelTransform(const moveit_msgs::RobotState &state, const std::string &world_frame);

    void printGroups();
    void printDebugInfo(const std::string &group_name);

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    boost::shared_ptr<urdf::Model> urdf_;

    robot_model_loader::RobotModelLoaderPtr rm_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    std::map<std::string, sbpl_arm_planner::Group*> group_config_map_;
    sbpl_arm_planner::Group *dgroup_;

    bool initURDF(const std::string &urdf_string);
    bool initRobotModel(const std::string &urdf_string);
    bool readGroups();
};

} // namespace manipulation
} // namespace sbpl

#endif
