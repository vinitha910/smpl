#include <smpl/debug/visualize.h>

#include <mutex>
#include <unordered_map>

namespace sbpl {
namespace viz {

namespace impl { // debug visualization level manager implementation

// void* in this context is a pointer to some implement-defined struct that
// manages the state of a named visualization

struct DebugViz {
    levels::Level level = levels::Info;
};

std::unordered_map<std::string, DebugViz> g_visualizations;

void* GetHandle(const std::string& name)
{
    return &g_visualizations[name];
}

bool IsEnabledFor(void* handle, levels::Level level)
{
    return static_cast<DebugViz*>(handle)->level <= level;
}

void GetVisualizations(std::unordered_map<std::string, levels::Level>& visualizations)
{
    visualizations.clear();
    for (auto& entry : g_visualizations) {
        visualizations.insert(std::make_pair(entry.first, entry.second.level));
    }
}

bool SetVisualizationLevel(const std::string& name, levels::Level level)
{
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
    levels::Level level)
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
        int size = 0;
        for (VizLocation* loc = g_loc_head; loc; loc = loc->next) {
            ++size;
        }
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

void get_visualizations(std::unordered_map<std::string, levels::Level>& visualizations)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    impl::GetVisualizations(visualizations);
}

bool set_visualization_level(const std::string& name, levels::Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    bool changed = impl::SetVisualizationLevel(name, level);
    if (changed) {
        NotifyLevelsChanged();
    }
    return changed;
}

void visualize(
    levels::Level level,
    const visualization_msgs::MarkerArray& markers)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, markers);
}

} // namespace viz
} // namespace sbpl
