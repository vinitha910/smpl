#include <smpl/debug/visualize.h>

#include <fstream>
#include <mutex>
#include <unordered_map>

namespace sbpl {
namespace visual {

namespace impl { // debug visualization level manager implementation

// void* in this context is a pointer to some implement-defined struct that
// manages the state of a named visualization

struct DebugViz {
    Level level = Level::Info;
};

static std::unordered_map<std::string, DebugViz> g_visualizations;
static bool g_initialized = false;

const char *to_cstring(Level level) {
    switch (level) {
    case Level::Debug:
        return "DEBUG";
    case Level::Info:
        return "INFO";
    case Level::Warn:
        return "WARN";
    case Level::Error:
        return "ERROR";
    case Level::Fatal:
        return "FATAL";
    default:
        return "";
    }
}

bool ParseVisualizationConfigLine(
    const std::string& line,
    std::vector<std::string>& split,
    Level& level)
{
    split.clear();

    size_t last = 0;
    size_t next;
    bool done = false;
    while (!done) {
        next = line.find_first_of(".=", last);
        if (next == std::string::npos) { // no remaining . or =
            split.clear();
            done = true;
        } else if (line[next] == '.') {
            if (last == next) {
                // should be a word between successive .'s or ='s
                split.clear();
                done = true;
            } else {
                split.push_back(line.substr(last, next - last));
                last = next + 1;
            }
        } else if (line[next] == '=') {
            if (last == next) {
                split.clear();
            } else {
                split.push_back(line.substr(last, next - last));
                // parse the right hand side
                std::string rhs(line.substr(next + 1));
                if (rhs == "DEBUG") {
                    level = Level::Debug;
                } else if (rhs == "INFO") {
                    level = Level::Info;
                } else if (rhs == "WARN") {
                    level = Level::Warn;
                } else if (rhs == "ERROR") {
                    level = Level::Error;
                } else if (rhs == "FATAL") {
                    level = Level::Fatal;
                } else {
                    split.clear();
                }
            }
            done = true;
        }
    }

    return !split.empty();
}

void Initialize()
{
    if (g_initialized) {
        return;
    }

    const char *config_path = getenv("SMPL_VISUALIZE_CONFIG_FILE");
    if (config_path) {
        std::ifstream f(config_path);
        if (f.is_open()) {
            std::string line;
            while (f.good()) {
                std::getline(f, line);

                std::vector<std::string> split;
                Level level;
                if (ParseVisualizationConfigLine(line, split, level)) {
                    std::string name = split.front();
                    for (size_t i = 1; i < split.size(); ++i) {
                        name = name + "." + split[i];
                    }
                    DebugViz viz;
                    viz.level = level;
                    g_visualizations[name] = viz;
                }
            }
        }
    }

    g_initialized = true;
}

void* GetHandle(const std::string& name)
{
    Initialize();
    return &g_visualizations[name];
}

bool IsEnabledFor(void* handle, Level level)
{
    Initialize();
    return static_cast<DebugViz*>(handle)->level <= level;
}

void GetVisualizations(std::unordered_map<std::string, Level>& visualizations)
{
    Initialize();
    visualizations.clear();
    for (auto& entry : g_visualizations) {
        visualizations.insert(std::make_pair(entry.first, entry.second.level));
    }
}

bool SetVisualizationLevel(const std::string& name, Level level)
{
    Initialize();
    auto& vis = g_visualizations[name];
    if (vis.level != level) {
        vis.level = level;
        return true;
    }
    return false;
}

} // namespace impl

// global singly-linked list of locations, for batch updates when debug
// visualization levels have changed
static VizLocation* g_loc_head = nullptr;
static VizLocation* g_loc_tail = nullptr;

// thread-safe initialization of debug visualization locations
static std::mutex g_locations_mutex;

// global visualizer
static VisualizerBase* g_visualizer;

// guards calls to global visualizer's visualize() method
static std::mutex g_viz_mutex;

void CheckLocationEnabledNoLock(VizLocation* loc)
{
    loc->enabled = impl::IsEnabledFor(loc->handle, loc->level);
}

void InitializeVizLocation(
    VizLocation* loc,
    const std::string& name,
    Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);

    if (loc->initialized) {
        return;
    }

    loc->handle = impl::GetHandle(name);
    loc->level = level;

    if (!g_loc_head) {
        // list is empty
        assert(!g_loc_tail);
        g_loc_head = loc;
        g_loc_tail = loc;
    } else {
        g_loc_tail->next = loc;
        g_loc_tail = loc;
    }

    CheckLocationEnabledNoLock(loc);

    loc->initialized = true;
}

void NotifyLevelsChanged()
{
    for (VizLocation *loc = g_loc_head; loc; loc = loc->next) {
        CheckLocationEnabledNoLock(loc);
    }
}

// Manage the global visualizer. NOT thread-safe!
void set_visualizer(VisualizerBase* visualizer)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    g_visualizer = visualizer;
}

void unset_visualizer()
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    g_visualizer = nullptr;
}

VisualizerBase* visualizer()
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    return g_visualizer;
}

void get_visualizations(std::unordered_map<std::string, Level>& visualizations)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    impl::GetVisualizations(visualizations);
}

bool set_visualization_level(const std::string& name, Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    bool changed = impl::SetVisualizationLevel(name, level);
    if (changed) {
        NotifyLevelsChanged();
    }
    return changed;
}

void visualize(Level level, const visual::Marker& marker)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, marker);
}

void visualize(Level level, const std::vector<visual::Marker>& markers)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, markers);
}

void visualize(Level level, const visualization_msgs::Marker& marker)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, marker);
}

void visualize(Level level, const visualization_msgs::MarkerArray& markers)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, markers);
}

void ConvertMarkerMsgToMarker(
    const visualization_msgs::Marker& mm,
    Marker& m)
{
    Eigen::Affine3d pose(
            Eigen::Translation3d(
                    mm.pose.position.x,
                    mm.pose.position.y,
                    mm.pose.position.z) *
            Eigen::Quaterniond(
                    mm.pose.orientation.w,
                    mm.pose.orientation.x,
                    mm.pose.orientation.y,
                    mm.pose.orientation.z));
    m.pose =  pose;

    auto convert_points = [](
        const std::vector<geometry_msgs::Point>& points)
        -> std::vector<Eigen::Vector3d>
    {
        std::vector<Eigen::Vector3d> new_points(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            auto& point = points[i];
            new_points[i] = Eigen::Vector3d(point.x, point.y, point.z);
        }
        return new_points;
    };

    switch (mm.type) {
    case visualization_msgs::Marker::ARROW:
        m.shape = Arrow{ mm.scale.x, mm.scale.y };
        break;
    case visualization_msgs::Marker::CUBE:
        m.shape = Cube{ mm.scale.x, mm.scale.y, mm.scale.z };
        break;
    case visualization_msgs::Marker::SPHERE:
        m.shape = Ellipse{ mm.scale.x, mm.scale.y, mm.scale.z };
        break;
    case visualization_msgs::Marker::CYLINDER:
        m.shape = Cylinder{ mm.scale.x, mm.scale.z };
        break;
    case visualization_msgs::Marker::LINE_STRIP:
        m.shape = LineStrip{ convert_points(mm.points) };
        break;
    case visualization_msgs::Marker::LINE_LIST:
        m.shape = LineList{ convert_points(mm.points) };
        break;
    case visualization_msgs::Marker::CUBE_LIST:
        m.shape = CubeList{ convert_points(mm.points), mm.scale.x };
        break;
    case visualization_msgs::Marker::SPHERE_LIST:
        m.shape = SphereList{ convert_points(mm.points), mm.scale.x };
        break;
    case visualization_msgs::Marker::POINTS:
        m.shape = PointList{ convert_points(mm.points) };
        break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
        m.shape = BillboardText{ mm.text };
        break;
    case visualization_msgs::Marker::MESH_RESOURCE:
        m.shape = MeshResource{ mm.text };
        break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
        m.shape = TriangleList{ convert_points(mm.points) };
        break;
    }

    m.frame_id = mm.header.frame_id;
    m.ns = mm.ns;
    m.lifetime = mm.lifetime.toSec();
    m.id = mm.id;
    m.action = (Marker::Action)(int)mm.action;
    if (mm.mesh_use_embedded_materials) {
        m.flags |= Marker::MESH_USE_EMBEDDED_MATERIALS;
    }
    if (mm.frame_locked) {
        m.flags |= Marker::FRAME_LOCKED;
    }
}

void ConvertMarkerToMarkerMsg(
    const Marker& m,
    visualization_msgs::Marker& mm)
{
    mm.header.frame_id = m.frame_id;
    mm.ns = m.ns;
    mm.id = m.id;
    switch (type(m.shape)) {
    case SHAPE_EMPTY:
        mm.type = visualization_msgs::Marker::SPHERE;
        mm.scale.x = mm.scale.y = mm.scale.z = 0.1;
        break;
    case SHAPE_ARROW:
        mm.type = visualization_msgs::Marker::ARROW;
        mm.scale.x = boost::get<Arrow>(m.shape).length;
        mm.scale.y = mm.scale.z = boost::get<Arrow>(m.shape).width;
        break;
    case SHAPE_CUBE:
        mm.type = visualization_msgs::Marker::CUBE;
        mm.scale.x = boost::get<Cube>(m.shape).length;
        mm.scale.y = boost::get<Cube>(m.shape).width;
        mm.scale.z = boost::get<Cube>(m.shape).height;
        break;
    case SHAPE_SPHERE:
        mm.type = visualization_msgs::Marker::SPHERE;
        mm.scale.x = mm.scale.y = mm.scale.z = 2.0 * boost::get<Sphere>(m.shape).radius;
        break;
    case SHAPE_ELLIPSE:
        mm.type = visualization_msgs::Marker::SPHERE;
        mm.scale.x = boost::get<Ellipse>(m.shape).axis_x;
        mm.scale.y = boost::get<Ellipse>(m.shape).axis_y;
        mm.scale.z = boost::get<Ellipse>(m.shape).axis_z;
        break;
    case SHAPE_CYLINDER:
        mm.scale.x = boost::get<Cylinder>(m.shape).radius;
        mm.scale.y = boost::get<Cylinder>(m.shape).radius;
        mm.scale.z = boost::get<Cylinder>(m.shape).height;
        mm.type = visualization_msgs::Marker::CYLINDER;
        break;
    case SHAPE_LINE_LIST:
        mm.type = visualization_msgs::Marker::LINE_LIST;
        mm.scale.x = 0.02; // line width
        break;
    case SHAPE_LINE_STRIP:
        mm.type = visualization_msgs::Marker::LINE_STRIP;
        mm.scale.x = 0.02; // line width
        break;
    case SHAPE_CUBE_LIST:
        mm.type = visualization_msgs::Marker::CUBE_LIST;
        mm.scale.x = mm.scale.y = mm.scale.z = boost::get<CubeList>(m.shape).size;
        break;
    case SHAPE_SPHERE_LIST:
        mm.type = visualization_msgs::Marker::SPHERE_LIST;
        mm.scale.x = mm.scale.y = mm.scale.z = 2.0 * boost::get<SphereList>(m.shape).radius;
        break;
    case SHAPE_POINT_LIST:
        mm.type = visualization_msgs::Marker::POINTS;
        mm.scale.x = mm.scale.y = 0.02; // point width and height
        break;
    case SHAPE_BILLBOARD_TEXT:
        mm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mm.text = boost::get<BillboardText>(m.shape).text;
        mm.scale.z = 0.1; // height of 'A'
        break;
    case SHAPE_MESH_RESOURCE:
        mm.type = visualization_msgs::Marker::MESH_RESOURCE;
        mm.mesh_resource = boost::get<MeshResource>(m.shape).uri;
        mm.scale.x = mm.scale.y = mm.scale.z = 1.0;
        break;
    case SHAPE_TRIANGLE_LIST:
        mm.type = visualization_msgs::Marker::TRIANGLE_LIST;
        mm.scale.x = mm.scale.y = mm.scale.z = 1.0;
        break;
    default:
        break;
    }

    switch (m.action) {
    case Marker::ACTION_ADD:
        mm.action = visualization_msgs::Marker::ADD;
        break;
    case Marker::ACTION_MODIFY:
        mm.action = visualization_msgs::Marker::MODIFY;
        break;
    case Marker::ACTION_DELETE:
        mm.action = visualization_msgs::Marker::DELETE;
        break;
    case Marker::ACTION_DELETE_ALL:
        mm.action = 3;
        break;
    }

    mm.pose.position.x = m.pose.position.x();
    mm.pose.position.y = m.pose.position.y();
    mm.pose.position.z = m.pose.position.z();
    mm.pose.orientation.w = m.pose.orientation.w();
    mm.pose.orientation.x = m.pose.orientation.x();
    mm.pose.orientation.y = m.pose.orientation.y();
    mm.pose.orientation.z = m.pose.orientation.z();

    if (m.color.which() == 0) {
        auto& color = boost::get<Color>(m.color);
        mm.color.r = color.r;
        mm.color.g = color.g;
        mm.color.b = color.b;
        mm.color.a = color.a;
    } else {
        auto& colors = boost::get<std::vector<Color>>(m.color);
        mm.colors.resize(colors.size());
        for (size_t i = 0; i < colors.size(); ++i) {
            std_msgs::ColorRGBA c;
            c.r = colors[i].r;
            c.g = colors[i].g;
            c.b = colors[i].b;
            c.a = colors[i].a;
            mm.colors.push_back(c);
        }
    }

    mm.frame_locked = (m.flags & Marker::FRAME_LOCKED);
    mm.mesh_use_embedded_materials = (m.flags & Marker::MESH_USE_EMBEDDED_MATERIALS);
}

void VisualizerBase::visualize(
    Level level,
    const visualization_msgs::Marker& mm)
{
    visual::Marker m;
    ConvertMarkerMsgToMarker(mm, m);
    visualize(level, m);
}

void VisualizerBase::visualize(
    Level level,
    const visualization_msgs::MarkerArray& markers)
{
    for (auto& m : markers.markers) {
        visualize(level, m);
    }
}

void VisualizerBase::visualize(
    Level level,
    const std::vector<visual::Marker>& markers)
{
    for (auto& m : markers) {
        visualize(level, m);
    }
}

void VisualizerBase::visualize(
    Level level,
    const visual::Marker& marker)
{
}

} // namespace viz
} // namespace sbpl
