////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the University of Pennsylvania nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen

#ifndef sbpl_collision_Group_h
#define sbpl_collision_Group_h

#include <algorithm>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

struct Sphere
{
    std::string name;
    KDL::Vector v;
    double radius;
    int priority;
    int kdl_chain;
    int kdl_segment;
};

struct Voxels
{
    int kdl_chain;
    int kdl_segment;
    std::vector<KDL::Vector> v;
};

struct Link
{
    int type;   // spheres or voxels
    int i_chain_;
    Voxels voxels_;
    std::string name_;
    std::string root_name_;
    std::vector<Sphere> spheres_;
};

class Group
{
public:

    Group(std::string name);
    
    ~Group();

    bool init(boost::shared_ptr<urdf::Model> urdf);

    void print();

    bool getParams(
        const CollisionGroupConfig& grp,
        const std::vector<CollisionSphereConfig>& spheres);

    std::string getName();

    const std::string& getReferenceFrame() const { return root_name_; }

    void getSpheres(std::vector<Sphere*>& spheres);
    
    bool computeFK(
        const std::vector<double>& angles,
        int chain,
        int segment,
        KDL::Frame& frame);

    bool computeFK(
        const std::vector<double>& angles,
        std::vector<std::vector<KDL::Frame>>& frames);

    void setOrderOfJointPositions(const std::vector<std::string>& joint_names);

    void setJointPosition(const std::string& name, double position);

    bool getFrameInfo(const std::string& name, int& chain, int& segment);

    void printSpheres();

    void printDebugInfo();

    void setGroupToWorldTransform(const KDL::Frame& f);

    KDL::Frame getGroupToWorldTransform();

    bool init_;
    enum { SPHERES, VOXELS } type_;
    std::string tip_name_;
    std::vector<Link> links_;

private:

    boost::shared_ptr<urdf::Model> urdf_;

    std::string name_;
    std::string root_name_;
    KDL::Frame T_root_to_world_;

    std::vector<KDL::Chain> chains_;
    std::vector<KDL::ChainFkSolverPos_recursive*> solvers_;
    std::vector<KDL::JntArray> joint_positions_;
    std::vector<std::vector<int>> frames_;
    std::vector<std::vector<std::string>> jntarray_names_;
    std::vector<std::vector<int>> angles_to_jntarray_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> order_of_input_angles_;
    std::vector<Sphere*> spheres_;

    bool initSpheres();

    bool initVoxels();
  
    bool initKinematics();

    bool getLinkVoxels(std::string name, std::vector<KDL::Vector>& voxels);
};

inline void Group::setGroupToWorldTransform(const KDL::Frame& f)
{
    T_root_to_world_ = f;
}

inline KDL::Frame Group::getGroupToWorldTransform()
{
    return T_root_to_world_;
}

} // namespace collision
} // namespace sbpl

#endif
