#include <sbpl_arm_planner/multi_frame_bfs_heuristic.h>

#include <leatherman/viz.h>

#include <bfs3d/BFS_3D.h>

namespace sbpl {
namespace manip {

MultiFrameBfsHeuristic::MultiFrameBfsHeuristic(
    ManipLattice* env,
    const OccupancyGridConstPtr& grid,
    const PlanningParams* params)
:
    ManipHeuristic(env, grid, params),
    m_bfs(),
    m_ee_bfs()
{
    syncGridAndBfs();
}

MultiFrameBfsHeuristic::~MultiFrameBfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool MultiFrameBfsHeuristic::setGoal(const GoalConstraint& goal)
{
    int ogx, ogy, ogz;
    m_grid->worldToGrid(
            goal.tgt_off_pose[0], goal.tgt_off_pose[1], goal.tgt_off_pose[2],
            ogx, ogy, ogz);

    int plgx, plgy, plgz;
    m_grid->worldToGrid(
            goal.pose[0], goal.pose[1], goal.pose[2],
            plgx, plgy, plgz);

    ROS_INFO("Setting the Two-Point BFS heuristic goals (%d, %d, %d), (%d, %d, %d)", ogx, ogy, ogz, plgx, plgy, plgz);

    if (!m_bfs->inBounds(ogx, ogy, ogz) ||
        !m_ee_bfs->inBounds(plgx, plgy, plgz))
    {
        ROS_ERROR("Heuristic goal is out of BFS bounds");
        return false;
    }

    m_bfs->run(ogx, ogy, ogz);
    m_ee_bfs->run(plgx, plgy, plgz);
    return true;
}

double MultiFrameBfsHeuristic::getMetricGoalDistance(
    double x, double y, double z)
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * m_grid->getResolution();
    }
    else {
        return (double)m_bfs->getDistance(gx, gy, gz) * m_grid->getResolution();
    }
}

int MultiFrameBfsHeuristic::GetGoalHeuristic(int state_id)
{
    return getGoalHeuristic(state_id, true);
}

int MultiFrameBfsHeuristic::GetStartHeuristic(int state_id)
{
    ROS_WARN_ONCE("MultiFrameBfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int MultiFrameBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (m_manip_env->isGoal(to_id)) {
        return GetGoalHeuristic(from_id);
    }
    else {
        ROS_WARN_ONCE("MultiFrameBfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

visualization_msgs::MarkerArray
MultiFrameBfsHeuristic::getWallsVisualization() const
{
    std::vector<geometry_msgs::Point> points;
    int dimX, dimY, dimZ;
    m_grid->getGridSize(dimX, dimY, dimZ);
    for (int z = 0; z < dimZ; z++) {
        for (int y = 0; y < dimY; y++) {
            for (int x = 0; x < dimX; x++) {
                if (m_bfs->isWall(x, y, z)) {
                    geometry_msgs::Point p;
                    m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
                    points.push_back(p);
                }
            }
        }
    }

    ROS_INFO("BFS Visualization contains %zu points", points.size());

    std_msgs::ColorRGBA color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    visualization_msgs::Marker cubes_marker = viz::getCubesMarker(
            points,
            m_grid->getResolution(),
            color,
            m_grid->getReferenceFrame(),
            "bfs_walls",
            0);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(std::move(cubes_marker));
    return ma;
}

visualization_msgs::MarkerArray
MultiFrameBfsHeuristic::getValuesVisualization() const
{
    visualization_msgs::MarkerArray ma;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    int dimX, dimY, dimZ;
    m_grid->getGridSize(dimX, dimY, dimZ);

    // factor in the ee bfs values? This doesn't seem to make a whole lot of
    // sense since the color would be derived from colocated cell values
    const bool factor_ee = false;

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = getGoalHeuristic(m_manip_env->getStartStateID(), factor_ee);

    const int edge_cost = m_params->cost_per_cell_;

    int max_cost = (int)(1.1 * start_heur);

    // ...and this will also flush the bfs...

    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;
    for (int z = 0; z < dimZ; ++z) {
        for (int y = 0; y < dimY; ++y) {
            for (int x = 0; x < dimX; ++x) {
                // skip cells without valid distances from the start
                if (m_bfs->isWall(x, y, z) || m_bfs->isUndiscovered(x, y, z)) {
                    continue;
                }

                int d = edge_cost * m_bfs->getDistance(x, y, z);
                int eed = factor_ee ? edge_cost * m_ee_bfs->getDistance(x, y, z) : 0;
                double cost_pct = (double)combine_costs(d, eed) / (double)(max_cost);

                if (cost_pct > 1.0) {
                    continue;
                }

                double hue = 300.0 - 300.0 * cost_pct;
                double sat = 1.0;
                double val = 1.0;
                double r, g, b;
                leatherman::HSVtoRGB(&r, &g, &b, hue, sat, val);

                std_msgs::ColorRGBA color;
                color.r = (float)r;
                color.g = (float)g;
                color.b = (float)b;
                color.a = 1.0f;

                auto clamp = [](double d, double lo, double hi) {
                    if (d < lo)
                        return lo;
                    else if (d > hi)
                        return hi;
                    else
                        return d;
                };

                color.r = clamp(color.r, 0.0f, 1.0f);
                color.g = clamp(color.g, 0.0f, 1.0f);
                color.b = clamp(color.b, 0.0f, 1.0f);

                geometry_msgs::Point p;
                m_grid->gridToWorld(x, y, z, p.x, p.y, p.z);
                points.push_back(p);

                colors.push_back(color);
            }
        }
    }

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = m_grid->getReferenceFrame();
    marker.ns = "bfs_values";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5 * m_grid->getResolution();
    marker.scale.y = 0.5 * m_grid->getResolution();
    marker.scale.z = 0.5 * m_grid->getResolution();
//    marker.color;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    marker.points = std::move(points);
    marker.colors = std::move(colors);
    marker.text = "";
    marker.mesh_use_embedded_materials = false;

    ma.markers.push_back(std::move(marker));
    return ma;
}

int MultiFrameBfsHeuristic::getGoalHeuristic(int state_id, bool use_ee) const
{
    const EnvROBARM3DHashEntry_t* state = m_manip_env->getHashEntry(state_id);
    if (state) {
        if (state->stateID == m_manip_env->getGoalStateID()) {
            return 0;
        }

        std::vector<double> pose;
        RobotModel* robot_model = m_manip_env->getRobotModel();
        if (!robot_model->computePlanningLinkFK(state->state, pose)) {
            ROS_ERROR("Failed to compute FK for planning link (state = %d)", state->stateID);
            return Infinity;
        }

        int eex[3];
        m_grid->worldToGrid(pose[0], pose[1], pose[2], eex[0], eex[1], eex[2]);
        const int ee_heur = getBfsCostToGoal(*m_ee_bfs, eex[0], eex[1], eex[2]);

        int h = getBfsCostToGoal(
                *m_bfs, state->xyz[0], state->xyz[1], state->xyz[2]);

        if (use_ee) {
            h = combine_costs(h, ee_heur);
        }
        return h;
    }
    else {
        return 0;
    }
}

void MultiFrameBfsHeuristic::syncGridAndBfs()
{
    int xc, yc, zc;
    m_grid->getGridSize(xc, yc, zc);
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    m_ee_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int z = 0; z < zc; ++z) {
        for (int y = 0; y < yc; ++y) {
            for (int x = 0; x < xc; ++x) {
                const double radius = m_params->planning_link_sphere_radius_;
                if (m_grid->getDistance(x, y, z) <= radius) {
                    m_bfs->setWall(x, y, z);
                    m_ee_bfs->setWall(x, y, z);
                    ++wall_count;
                }
            }
        }
    }

    ROS_INFO("%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int MultiFrameBfsHeuristic::getBfsCostToGoal(
    const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else {
        return m_params->cost_per_cell_ * bfs.getDistance(x, y, z);
    }
}

int MultiFrameBfsHeuristic::combine_costs(int c1, int c2) const
{
    return c1 + c2;
}

} // namespace manip
} // namespace sbpl
