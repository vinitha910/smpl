#ifndef _MOTION_PRIMITIVE_
#define _MOTION_PRIMITIVE_

#include <vector>
#include <sstream>
#include <ros/console.h>

namespace sbpl_arm_planner {

typedef std::vector<double> RobotState;

typedef std::vector<RobotState> Action;

struct MotionPrimitive
{
    enum Type
    {
        LONG_DISTANCE = -1,
        SNAP_TO_RPY = 0, // NOTE: start at 0 to use successive types as indices
        SNAP_TO_XYZ,
        SNAP_TO_XYZ_RPY,
        SHORT_DISTANCE,
        NUMBER_OF_MPRIM_TYPES
    };

    Type type;
    int id;
    Action action;

    void print() const;
};

inline
void MotionPrimitive::print() const
{ 
    ROS_INFO("type: %d  id: %d  nsteps: %d ", type, id, int(action.size()));
    std::stringstream os;
    for (std::size_t j = 0; j < action.size(); ++j) {
        os.str("");
        os << "[step: " << int(j+1) << "/" << int(action.size()) << "] ";
        for (std::size_t k = 0; k < action[j].size(); ++k) {
            os << std::setw(4) << std::setprecision(3) << std::fixed << action[j][k] << " ";
        }
        ROS_INFO_STREAM(os.str());
    }
}

} // namespace sbpl_arm_planner

#endif
