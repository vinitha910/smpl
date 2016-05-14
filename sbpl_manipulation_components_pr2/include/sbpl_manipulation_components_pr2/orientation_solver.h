/// \author Gokul Subramanian, MSE in Robotics & CIS 2011
/// \maintainer (Maintained by Benjamin Cohen)
/// \date 4/14/2010

#ifndef sbpl_manip_orientation_solver_h
#define sbpl_manip_orientation_solver_h

// standard includes
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

namespace sbpl {
namespace manip {

class RPYSolver
{
  public:

    RPYSolver(double wrist_pitch_min_limit, double wrist_pitch_max_limit);

    /// \param rpy
    /// \param start Joint variable vector of the start
    /// \param forearm_roll_link_pose { x, y, z, r, p, y } of the forearm
    /// \param endeff_link_pose { x, y, z, r, p, y } of the end effector
    /// \param solution_num 1 or 2 to choose the forearm roll orientation
    /// \param solution Joint variable vector of the solution
    bool computeRPYOnly(
        const std::vector<double>& rpy,
        const std::vector<double>& start,
        const std::vector<double>& forearm_roll_link_pose,
        const std::vector<double>& endeff_link_pose,
        int solution_num,
        std::vector<double>& solution) const;

  private:

    double wrist_pitch_min_limit_;
    double wrist_pitch_max_limit_;

    /// \brief Calculate the angles required for the pr2 wrist to achieve a
    ///     desired orientation
    ///
    /// Function calculates the forearm roll, wrist pitch and wrist roll for the
    /// PR2 robot hand which is required to attain a certain yaw, pitch and roll
    /// configuration in the world frame, starting at a certain yaw, pitch and
    /// roll configuration. Do not use for other robots unless the joints are of
    /// the same nature and the reference frame conventions are the same.
    ///
    /// \param[out] output { success, theta_1, theta_2, theta_3 }
    /// \param phi yaw of the forearm
    /// \param theta pitch of the forearm
    /// \param psi roll of the forearm
    /// \param yaw1 yaw of the end effector
    /// \param pitch1 pitch of the end effector
    /// \param roll1 roll of the end effector
    /// \param yaw2 desired yaw of the end effector
    /// \param pitch2 desired pitch of the end effector
    /// \param roll2 desired pitch of the end effector
    /// \param attempt 1 or 2 to choose the forearm roll orientation
    void orientationSolver(
        double* output,
        double phi, double theta, double psi,
        double yaw1, double pitch1, double roll1,
        double yaw2, double pitch2, double roll2,
        int attempt) const;
};

} // namespace manip
} // namespace sbpl

#endif

