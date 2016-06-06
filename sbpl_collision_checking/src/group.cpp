////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
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
/// \author Andrew Dornbush

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shapes.h>
#include <leatherman/print.h>
#include <sbpl_geometry_utils/Voxelizer.h>

// project includes
#include <sbpl_collision_checking/group.h>

#define RES 0.02

namespace sbpl {
namespace collision {

bool sortSphere(const Sphere* a, const Sphere* b)
{
    return a->priority < b->priority;
}

Group::Group() :
    init_(false),
    type_(INVALID),
    tip_name_(),
    links_(),
    urdf_(),
    name_(),
    root_name_(),
    m_T_model_group(KDL::Frame::Identity()),
    chains_(),
    solvers_(),
    jntarray_names_(),
    joint_positions_(),
    frames_(),
    angles_to_jntarray_(),
    order_of_input_angles_(),
    spheres_()
{
}

Group::~Group()
{
    for (std::size_t i = 0; i < solvers_.size(); i++) {
        if (solvers_[i] != NULL) {
            delete solvers_[i];
            solvers_[i] = NULL;
        }
    }
}

bool Group::init(
    boost::shared_ptr<urdf::Model> urdf,
    const CollisionGroupConfig& config,
    const std::vector<CollisionSphereConfig>& sphere_config)
{
    name_ = config.name;

    if (!getParams(config, sphere_config)) {
        ROS_ERROR("Failed to get parameters for group");
        return false;
    }

    urdf_ = urdf;
    if (!initKinematics()) {
        return false;
    }

    if (type_ == Group::VOXELS) {
        init_ = initVoxels();
    }
    else {
        init_ = initSpheres();
    }

    return init_;
}

bool Group::initKinematics()
{
    if (!decomposeModelIntoChains()) {
        ROS_ERROR("Failed to decompose model into set of kinematic chains");
        return false;
    }

    if (!initKinematicChains()) {
        ROS_ERROR("Failed to initialize kinematic chains");
        return false;
    }

    return true;
}

bool Group::decomposeModelIntoChains()
{
    bool unincluded_links = true;
    int cnt = 0;
    std::vector<int> link_included(links_.size(),-1);
    KDL::Chain chain;
    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(*urdf_, tree)) {
        ROS_ERROR("Failed to parse tree from robot description.");
        return false;
    }

    // loop until all links are included in a single kdl chain
    while (unincluded_links && cnt < 100) {
        ROS_DEBUG("--------------------------------------");
        ROS_DEBUG("-------- %s:  %d ------------------", name_.c_str(), cnt);
        ROS_DEBUG("--------------------------------------");
        cnt++;
        std::vector<int> num_links_per_tip(links_.size(), 0);

        // compute # of links each link would include if tip of chain
        for (size_t i = 0; i < links_.size(); ++i) {
            // link i is already included in a chain
            if (link_included[i] > -1) {
                continue;
            }

            // link i is same as root link, set as identity
            if (root_name_.compare(links_[i].root_name_) == 0) {
                ROS_ERROR("The group root matches the link root. Creating an empty chain. { root: %s, tip: %s }", root_name_.c_str(), links_[i].root_name_.c_str());
                num_links_per_tip[i]++;
            }

            // create chain with link i as the tip
            if (!tree.getChain(root_name_, links_[i].root_name_, chain)) {
                ROS_ERROR("Error: Failed to fetch the KDL chain. Exiting. (root: %s, tip: %s)", root_name_.c_str(), links_[i].root_name_.c_str());
                continue;
            }

            // count number of links included in this chain
            for (size_t j = 0; j < links_.size(); ++j) {
                // link j is already included in a chain
                if (link_included[j] > -1) {
                    continue;
                }

                // check if link is included in this chain
                int seg;
                if (leatherman::getSegmentIndex(chain, links_[j].root_name_, seg)) {
                    num_links_per_tip[i]++;
                }
            }
        }

        // get chain tip that would include the most links
        int i_max = 0;
        for (size_t i = 0; i < num_links_per_tip.size(); ++i) {
            ROS_DEBUG("[%d]  chain_tip: %25s  included_links %d", int(i), links_[i].root_name_.c_str(), num_links_per_tip[i]);
            if (num_links_per_tip[i] > num_links_per_tip[i_max]) {
                i_max = i;
            }
        }
        ROS_DEBUG("[cnt %d] Creating a chain for %s group with %s as the tip.", cnt, name_.c_str(), links_[i_max].root_name_.c_str());

        // create chain with link i_max as the tip
        if (!tree.getChain(root_name_, links_[i_max].root_name_, chain)) {
            ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting. (root: %s, tip: %s)", root_name_.c_str(), links_[i_max].root_name_.c_str());
            continue;
        }

        // add chain to the group
        chains_.push_back(chain);

        // mark links that are included in this chain
        int included_links = 0;
        for (size_t i = 0; i < links_.size(); ++i) {
            // link i is already included in a different chain
            if (link_included[i] > -1) {
                included_links++;
                continue;
            }

            if (root_name_.compare(links_[i].root_name_) == 0) {
                ROS_ERROR("Checking which links are included in the single link chain.");
                link_included[i] = chains_.size()-1;
                links_[i].i_chain_ = chains_.size()-1;
                included_links++;
                ROS_DEBUG("[one_link-chain: %s] [%d] includes: %s", links_[i_max].root_name_.c_str(), included_links, links_[i].root_name_.c_str());
            }

            // check if link i is included in this chain
            int seg;
            if (leatherman::getSegmentIndex(chains_.back(), links_[i].root_name_, seg)) {
                link_included[i] = chains_.size()-1;
                links_[i].i_chain_ = chains_.size()-1;
                included_links++;
                ROS_DEBUG("[chain: %s] [%d] includes: %s", links_[i_max].root_name_.c_str(), included_links, links_[i].root_name_.c_str());
            }
        }

        if (included_links == int(links_.size())) {
            unincluded_links = false;
        }

        ROS_DEBUG("Completed %d loops of the while loop (included_links = %d)", cnt, included_links);
    }

    for (size_t i = 0; i < link_included.size(); ++i) {
        ROS_DEBUG("included link: %25s  link_root: %25s  chain: %d", links_[i].name_.c_str(), links_[i].root_name_.c_str(), link_included[i]);
    }

    if (cnt >= 100) {
        return false;
    }

    return true;
}

bool Group::initKinematicChains()
{
    // initialize the FK solvers
    solvers_.resize(chains_.size());
    for (size_t i = 0; i < chains_.size(); ++i) {
        solvers_[i] = new KDL::ChainFkSolverPos_recursive(chains_[i]);
        ROS_DEBUG("[%s] Instantiated a forward kinematics solver for chain #%d for the %s with %d joints.", name_.c_str(), int(i), name_.c_str(), chains_[i].getNrOfJoints());
    }

    // get order of joints for each chain
    jntarray_names_.resize(chains_.size());
    for (size_t i = 0; i < chains_.size(); ++i) {
        for (size_t j = 0; j < chains_[i].getNrOfSegments(); ++j) {
            if (chains_[i].getSegment(j).getJoint().getTypeName().compare("None") != 0) {
                jntarray_names_[i].push_back(chains_[i].getSegment(j).getJoint().getName());
            }
        }
    }

    // debug output
    ROS_DEBUG("[%s] Order of Joints in Joint Array input to the FK Solver:", name_.c_str());
    for (size_t i = 0; i < jntarray_names_.size(); ++i) {
        for (size_t j = 0; j < jntarray_names_[i].size(); ++j) {
            ROS_DEBUG("[%s] [chain %d] %d: %s", name_.c_str(), int(i), int(j), jntarray_names_[i][j].c_str());
        }
    }

    // initialize the sizes of the JntArrays
    joint_positions_.resize(chains_.size());
    for (size_t i = 0; i < chains_.size(); ++i) {
        joint_positions_[i].resize(jntarray_names_[i].size());
        KDL::SetToZero(joint_positions_[i]);
    }

    ROS_DEBUG("Initialized %d chains for the %s group.", int(chains_.size()), name_.c_str());
    return true;
}

bool Group::getParams(
    const CollisionGroupConfig& group_config,
    const std::vector<CollisionSphereConfig>& spheres_config)
{
    name_ = group_config.name;

    if (group_config.type == "voxels") {
        type_ = Group::VOXELS;
    }
    else if (group_config.type == "spheres") {
        type_ = Group::SPHERES;
    }
    else {
        ROS_ERROR("Illegal group type. (voxels or spheres)");
        return false;
    }

    root_name_ = group_config.root_name;
    tip_name_ = group_config.tip_name;

    Link link;
    for (int j = 0; j < (int)group_config.collision_links.size(); j++) {
        const CollisionLinkConfig& link_config = group_config.collision_links[j];
        link.name_ = link_config.name;
        link.root_name_ = link_config.root;
        link.spheres_.clear();

        if (type_ == Group::SPHERES) {
            const std::vector<std::string>& sphere_names = link_config.spheres;
            for (const std::string& sphere_name : sphere_names) {
                // find the sphere corresponding to this name
                auto sit = std::find_if(
                        spheres_config.begin(), spheres_config.end(),
                        [&](const CollisionSphereConfig& config)
                        {
                            return config.name == sphere_name;
                        });

                if (sit == spheres_config.end()) {
                    ROS_ERROR("Failed to find sphere %s in the sphere list.", sphere_name.c_str());
                    return false;
                }

                Sphere sphere;
                sphere.name = sit->name;
                sphere.v.x(sit->x);
                sphere.v.y(sit->y);
                sphere.v.z(sit->z);
                sphere.priority = sit->priority;
                sphere.radius = sit->radius;
                link.spheres_.push_back(sphere);
            }
        }
        links_.push_back(link);
    }
    return true;
}

void Group::printSpheres() const
{
    if (!init_) {
        ROS_ERROR("Failed to print the collision spheres because the %s group is not initialized.", name_.c_str());
        return;
    }

    ROS_INFO("\n%s", name_.c_str());
    for (size_t i = 0; i < spheres_.size(); ++i) {
        ROS_INFO("[%s] x: %0.3f  y:%0.3f  z:%0.3f  radius: %0.3f  priority: %d", spheres_[i]->name.c_str(), spheres_[i]->v.x(), spheres_[i]->v.y(), spheres_[i]->v.z(), spheres_[i]->radius, spheres_[i]->priority);
    }
    ROS_INFO(" ");
}

bool Group::initSpheres()
{
    assert(type_ == Group::SPHERES);

    // assign the kdl segment numbers to each sphere
    for (size_t i = 0; i < links_.size(); ++i) {
        int seg = 0;
        if (!leatherman::getSegmentIndex(chains_[links_[i].i_chain_], links_[i].root_name_, seg)) {
            return false;
        }

        for (size_t j = 0; j < links_[i].spheres_.size(); ++j) {
            links_[i].spheres_[j].kdl_segment = seg + 1;
            links_[i].spheres_[j].kdl_chain = links_[i].i_chain_;
        }
    }

    // fill the group's list of all the spheres
    for (size_t i = 0; i < links_.size(); ++i) {
        for (size_t j = 0; j < links_[i].spheres_.size(); ++j) {
            spheres_.push_back(&(links_[i].spheres_[j]));
        }
    }

    // sort the spheres by priority
    sort(spheres_.begin(), spheres_.end(), sortSphere);

    // populate the frames vector that stores the segments in each chain
    frames_.resize(chains_.size());
    for (size_t i = 0; i < spheres_.size(); ++i) {
        if (std::find(
                frames_[spheres_[i]->kdl_chain].begin(),
                frames_[spheres_[i]->kdl_chain].end(),
                spheres_[i]->kdl_segment) ==
                frames_[spheres_[i]->kdl_chain].end())
        {
            frames_[spheres_[i]->kdl_chain].push_back(spheres_[i]->kdl_segment);
        }
    }

    // debug output
    ROS_DEBUG("[%s] Frames:", name_.c_str());
    for (size_t i = 0; i < frames_.size(); ++i) {
        for (size_t j = 0; j < frames_[i].size(); ++j) {
            ROS_DEBUG("[%s] [chain %d] segment: %d", name_.c_str(), int(i), frames_[i][j]);
        }
    }

    return true;
}

bool Group::initVoxels()
{
    assert(type_ == Group::VOXELS);

    // get link voxels and assign the kdl segment numbers to each link
    for (size_t i = 0; i < links_.size(); ++i) {
        Link& link = links_[i];
        if (!getLinkVoxels(link.root_name_, link.voxels_.v)) {
            ROS_ERROR("Failed to retrieve voxels for link '%s' in group '%s'", link.root_name_.c_str(), name_.c_str());
            return false;
        }
        ROS_DEBUG("Retrieved %d voxels for link '%s'", int(link.voxels_.v.size()), link.root_name_.c_str());
        int seg = 0;
        if (!leatherman::getSegmentIndex(chains_[link.i_chain_], link.root_name_, seg)) {
            ROS_ERROR("When retrieving group voxels, getSegmentIndex() failed. The group root must be the same as the link root. {root: %s, tips: %s}", root_name_.c_str(),  link.root_name_.c_str());
            seg = -1;
            //return false;
        }

        link.voxels_.kdl_segment = seg + 1;
        link.voxels_.kdl_chain = link.i_chain_;
    }

    // populate the frames vector that stores the segments in each chain
    frames_.resize(chains_.size());
    for (size_t i = 0; i < links_.size(); ++i) {
        const Link& link = links_[i];
        frames_[link.voxels_.kdl_chain].push_back(link.voxels_.kdl_segment);
    }

    // debug output
    ROS_DEBUG("[%s] Frames:", name_.c_str());
    for (size_t i = 0; i < frames_.size(); ++i) {
        for (size_t j = 0; j < frames_[i].size(); ++j) {
            ROS_DEBUG("[%s] [chain %d] segment: %d", name_.c_str(), int(i), frames_[i][j]);
        }
    }

    return true;
}

bool Group::computeFK(
    const std::vector<double> &angles,
    int chain,
    int segment,
    KDL::Frame &frame)
{
    if (segment == 0) {
        ROS_ERROR("segment is 0!");
        frame = KDL::Frame::Identity();
    }
    else {
        // sort elements of input angles into proper positions in the JntArray
        for (size_t i = 0; i < angles.size(); ++i) {
            if (angles_to_jntarray_[chain][i] == -1) {
                continue;
            }
            joint_positions_[chain](angles_to_jntarray_[chain][i]) = angles[i];
        }

        if (solvers_[chain]->JntToCart(joint_positions_[chain], frame, segment) < 0) {
            ROS_ERROR("JntToCart returned < 0. Exiting.");
            return false;
        }
    }

    frame = m_T_model_group * frame;
    return true;
}

bool Group::computeFK(
    const std::vector<double>& angles,
    std::vector<std::vector<KDL::Frame>>& frames)
{
    frames.resize(chains_.size());
    for (int i = 0; i < int(frames_.size()); ++i) {
        frames[i].resize(chains_[i].getNrOfSegments()+1);
        for (size_t j = 0; j < frames_[i].size(); ++j) {
            ROS_DEBUG("chain: %d   frame_index: %d  frame: %d  size_of_frames_vector: %d", i, int(j), int(frames_[i][j]), int(frames[i].size()));
            if (!computeFK(angles, i, frames_[i][j], frames[i][frames_[i][j]])) {
                return false;
            }
        }
    }
    return true;
}

void Group::setOrderOfJointPositions(const std::vector<std::string>& joint_names)
{
    // store the desired order of the input angles for debug information
    order_of_input_angles_ = joint_names;

    // for each joint, find its proper index in the JntArray for each chain's solver
    angles_to_jntarray_.resize(chains_.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        bool matched = false; // kinda useless

        ROS_DEBUG("[%s] [%d] %s", name_.c_str(), int(i), joint_names[i].c_str());
        for (size_t k = 0; k < chains_.size(); ++k) {
            angles_to_jntarray_[k].resize(joint_names.size(), -1);
            for (size_t j = 0; j < jntarray_names_[k].size(); ++j) {
                if (joint_names[i].compare(jntarray_names_[k][j]) == 0) {
                    angles_to_jntarray_[k][i] = j;
                    matched = true;
                    break;
                }
            }
        }
        if (!matched) {
            ROS_ERROR("%s was not found in either chain. Why do you send it to the forward kinematics solver?", joint_names[i].c_str());
        }
    }

    for (size_t i = 0; i < angles_to_jntarray_.size(); ++i) {
        for (size_t j = 0; j < angles_to_jntarray_[i].size(); ++j) {
            ROS_DEBUG("[%s] [chain %d] joint: %s  index: %d", name_.c_str(), int(i), joint_names[j].c_str(), angles_to_jntarray_[i][j]);
        }
    }
}

void Group::setJointPosition(const std::string& name, double position)
{
    for (std::size_t i = 0; i < jntarray_names_.size(); ++i) {
        for (std::size_t j = 0; j < jntarray_names_[i].size(); ++j) {
            if (name == jntarray_names_[i][j]) {
                joint_positions_[i](j) = position;
                break;
            }
        }
    }
}

bool Group::getLinkVoxels(
    const std::string& name,
    std::vector<KDL::Vector>& voxels)
{
    boost::shared_ptr<const urdf::Link> link = urdf_->getLink(name);

    if (!link) {
        ROS_ERROR("Failed to find link '%s' in URDF.", name.c_str());
        return false;
    }
    if (!link->collision) {
        ROS_ERROR("Failed to find collision field for link '%s' in URDF.", link->name.c_str());
        return false;
    }
    if (!link->collision->geometry) {
        ROS_ERROR("Failed to find geometry for link '%s' in URDF. (group: %s)", name.c_str(), link->collision->group_name.c_str());
        return false;
    }

    geometry_msgs::Pose p;

    boost::shared_ptr<const urdf::Geometry> geom = link->collision->geometry;
    p.position.x = link->collision->origin.position.x;
    p.position.y = link->collision->origin.position.y;
    p.position.z = link->collision->origin.position.z;
    link->collision->origin.rotation.getQuaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);

    Eigen::Affine3d pose;
    tf::poseMsgToEigen(p, pose);

    voxels.clear();

    std::vector<Eigen::Vector3d> v;
    if (geom->type == urdf::Geometry::MESH) {
        geometry_msgs::Vector3 scale;
        scale.x = 1; scale.y = 1; scale.z = 1;
        std::vector<geometry_msgs::Point> mesh_vertices;
        std::vector<int> triangles;
        urdf::Mesh* mesh = (urdf::Mesh*)geom.get();
        if (!leatherman::getMeshComponentsFromResource(
                mesh->filename, scale, triangles, mesh_vertices))
        {
            ROS_ERROR("Failed to get mesh from file. (%s)", mesh->filename.c_str());
            return false;
        }
        ROS_DEBUG("mesh: %s  triangles: %u  vertices: %u", name.c_str(), int(triangles.size()), int(mesh_vertices.size()));

        std::vector<Eigen::Vector3d> vertices(mesh_vertices.size());
        for (size_t vidx = 0; vidx < mesh_vertices.size(); ++vidx) {
            const geometry_msgs::Point& vertex = mesh_vertices[vidx];
            vertices[vidx] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
        }

        sbpl::VoxelizeMesh(vertices, triangles, pose, RES, v, false);
        ROS_DEBUG("mesh: %s  voxels: %u", name.c_str(), int(v.size()));
    }
    else if (geom->type == urdf::Geometry::BOX) {
        std::vector<Eigen::Vector3d> v;
        urdf::Box* box = (urdf::Box*)geom.get();
        sbpl::VoxelizeBox(box->dim.x, box->dim.y, box->dim.z, pose, RES, v, false);
        ROS_INFO("box: %s  voxels: %u   {dims: %0.2f %0.2f %0.2f}", name.c_str(), int(v.size()), box->dim.x, box->dim.y, box->dim.z);
    }
    else if (geom->type == urdf::Geometry::CYLINDER) {
        std::vector<Eigen::Vector3d> v;
        urdf::Cylinder* cyl = (urdf::Cylinder*)geom.get();
        sbpl::VoxelizeCylinder(
                cyl->radius, cyl->length, pose, RES, v, true);
        ROS_INFO("cylinder: %s  voxels: %u", name.c_str(), int(v.size()));
    }
    else if (geom->type == urdf::Geometry::SPHERE) {
        std::vector<Eigen::Vector3d> v;
        urdf::Sphere* sph = (urdf::Sphere*)geom.get();
        sbpl::VoxelizeSphere(sph->radius, pose, RES, v, true);
        ROS_INFO("sphere: %s  voxels: %u", name.c_str(), int(v.size()));
    }
    else {
        ROS_ERROR("Failed to get voxels for link '%s'.", name.c_str());
        return false;
    }

    voxels.resize(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        voxels[i].x(v[i][0]);
        voxels[i].y(v[i][1]);
        voxels[i].z(v[i][2]);
    }

    if (voxels.empty()) {
        ROS_ERROR("Problem voxeling '%s' link. It resulted in 0 voxels.", name.c_str());
    }

    return true;
}

bool Group::getFrameInfo(const std::string& name, int& chain, int& segment) const
{
    for (size_t i = 0; i < chains_.size(); ++i) {
        for (size_t k = 0; k < chains_[i].getNrOfSegments(); ++k) {
            if (chains_[i].getSegment(k).getName().compare(name) == 0) {
                chain = i;
                segment = k + 1;
                return true;
            }
        }
    }
    return false;
}

void Group::print() const
{
    if (!init_) {
        ROS_ERROR("Failed to print %s group information because has not yet been initialized.", name_.c_str());
        return;
    }

    ROS_INFO("name: %s", name_.c_str());
    ROS_INFO("type: %d", type_);
    ROS_INFO("root name: %s", root_name_.c_str());
    ROS_INFO("collision links: ");
    for (std::size_t i = 0; i < links_.size(); ++i) {
        ROS_INFO("  name: %s", links_[i].name_.c_str());
        ROS_INFO("  root: %s", links_[i].root_name_.c_str());
        ROS_INFO(" chain: %d", links_[i].i_chain_);
        if (type_ == Group::SPHERES) {
            ROS_INFO(" spheres: %d", int(links_[i].spheres_.size()));
            for (std::size_t j = 0; j < links_[i].spheres_.size(); ++j) {
                ROS_INFO("  [%s] x: %0.3f y: %0.3f z: %0.3f radius: %0.3f priority: %d chain: %d segment: %d", links_[i].spheres_[j].name.c_str(), links_[i].spheres_[j].v.x(), links_[i].spheres_[j].v.y(), links_[i].spheres_[j].v.z(), links_[i].spheres_[j].radius, links_[i].spheres_[j].priority, links_[i].spheres_[j].kdl_chain, links_[i].spheres_[j].kdl_segment);
            }
        }
        else if (type_ == Group::VOXELS) {
            ROS_INFO(" voxels: %d chain: %d segment: %d", int(links_[i].voxels_.v.size()), links_[i].voxels_.kdl_chain, links_[i].voxels_.kdl_segment);
            for (std::size_t j = 0; j < links_[i].voxels_.v.size(); ++j) {
                ROS_DEBUG("  [%d] x: %0.3f y: %0.3f z: %0.3f", int(j), links_[i].voxels_.v[j].x(), links_[i].voxels_.v[j].y(), links_[i].voxels_.v[j].z());
            }
        }
        if(i < links_.size()-1)
        ROS_INFO(" ---");
    }
    ROS_INFO(" ");
    if (type_ == Group::SPHERES) {
        ROS_INFO("sorted spheres: ");
        for (std::size_t j = 0; j < spheres_.size(); ++j) {
            ROS_INFO("  [%s] x: %0.3f  y:%0.3f  z:%0.3f  radius: %0.3f  priority: %d", spheres_[j]->name.c_str(), spheres_[j]->v.x(), spheres_[j]->v.y(), spheres_[j]->v.z(), spheres_[j]->radius, spheres_[j]->priority);
        }
        ROS_INFO(" ");
    }
    ROS_INFO("kinematic chain(s): ");
    for (std::size_t j = 0; j < chains_.size(); ++j) {
        leatherman::printKDLChain(chains_[j], "chain " + boost::lexical_cast<std::string>(j));
    }
    ROS_INFO(" ");
}

void Group::printDebugInfo() const
{
    ROS_INFO("[name] %s", name_.c_str());
    ROS_INFO("[chains] %d", int(chains_.size()));
    ROS_INFO("[solvers] %d", int(solvers_.size()));
    ROS_INFO("[joint_positions] %d", int(joint_positions_.size()));
    ROS_INFO("[frames] %d", int(frames_.size()));
    for (size_t i = 0; i < frames_.size(); ++i) {
        ROS_INFO("[frames] [%d] %d", int(i), int(frames_[i].size()));
    }
    ROS_INFO("[jntarray_names] %d", int(jntarray_names_.size()));
    for (size_t i = 0; i < jntarray_names_.size(); ++i) {
        ROS_INFO("[jntarray_names] [%d] %d", int(i), int(jntarray_names_[i].size()));
    }
    ROS_INFO("[angles_to_jntarray] %d", int(angles_to_jntarray_.size()));
    for (size_t i = 0; i < angles_to_jntarray_.size(); ++i) {
        ROS_INFO("[angles_to_jntarray] [%d] %d", int(i), int(angles_to_jntarray_[i].size()));
    }
}

} // namespace collision
} // namespace sbpl
