#include <smpl/console/detail/console_std.h>

// standard includes
#include <stdarg.h>
#include <string.h>
#include <iostream>
#include <mutex>
#include <unordered_map>

#include <boost/program_options.hpp>

// project includes
#include <smpl/console/ansi.h>
#include <smpl/console/nonstd.h>

namespace sbpl {
namespace console {

bool g_initialized = false;

static bool g_unbuffered = false;
static bool g_colored = false;
static bool g_show_locations = false;

static std::mutex g_init_mutex;
static std::unordered_map<std::string, Level> g_log_levels;

void initialize()
{
    std::unique_lock<std::mutex> lock(g_init_mutex);

    if (g_initialized) {
        return;
    }

    // options of the form --name.name.name=level

    const char* config_filepath = getenv("SMPL_CONSOLE_CONFIG_FILE");
    if (!config_filepath) {
        g_initialized = true;
        return;
    }

    std::cout << "Initialize console system from " << config_filepath << std::endl;

    namespace po = boost::program_options;

    po::options_description ops;
    ops.add_options()
            ("format.unbuffered", po::value<bool>(&g_unbuffered)->default_value(false))
            ("format.colored", po::value<bool>(&g_colored)->default_value(false))
            ("format.show_locations", po::value<bool>(&g_show_locations)->default_value(false))
            ;

    // map from logger names to log levels
    bool allow_unregistered = true;
    auto pops = po::parse_config_file<char>(
            config_filepath, ops, allow_unregistered);

    po::variables_map vm;
    po::store(pops, vm);
    po::notify(vm);

    printf("unbuffered: %d\n", (int)g_unbuffered);
    printf("colored: %d\n", (int)g_colored);
    printf("show_locations: %d\n", (int)g_show_locations);

    std::cout << "unregistered options:" << std::endl;
    for (auto& op : pops.options) {
        if (op.unregistered) {
            std::cout << "  " << op.string_key << "=" << op.value.size() << ':' << op.value << std::endl;
        }
    }

    std::cout << "Initialized console system" << std::endl;
    g_initialized = true;
}

void InitializeLogLocation(
    LogLocation* loc,
    const std::string& name,
    Level level)
{
    if (loc->initialized) {
        return;
    }

    loc->level = level;
    loc->enabled = level > LEVEL_DEBUG;
    loc->initialized = true;
}

void print(Level level, const char* filename, int line, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    FILE* f;
    if (level >= LEVEL_ERROR) {
        f = stderr;
    } else {
        f = stdout;
    }

    if (g_colored) {
        switch (level) {
        case LEVEL_DEBUG:
            fprintf(f, "%s", codes::green);
            break;
        case LEVEL_INFO:
            fprintf(f, "%s", codes::white);
            break;
        case LEVEL_WARN:
            fprintf(f, "%s", codes::yellow);
            break;
        case LEVEL_ERROR:
            fprintf(f, "%s", codes::red);
            break;
        case LEVEL_FATAL:
            fprintf(f, "%s", codes::red);
            break;
        default:
            break;
        }
    }

    switch (level) {
    case LEVEL_DEBUG:
        fprintf(f, "[DEBUG] ");
        break;
    case LEVEL_INFO:
        fprintf(f, "[INFO]  ");
        break;
    case LEVEL_WARN:
        fprintf(f, "[WARN]  ");
        break;
    case LEVEL_ERROR:
        fprintf(f, "[ERROR] ");
        break;
    case LEVEL_FATAL:
        fprintf(f, "[FATAL] ");
        break;
    default:
        break;
    }

    vfprintf(f, fmt, args);

    // print file and line
    if (g_show_locations) {
        const char* base = strrchr(filename, '\\');
        if (base) {
            fprintf(f, " [%s:%d]", base + 1, line);
        } else {
            fprintf(f, " [%s:%d]", filename, line);
        }
    }

    if (g_colored) {
        fprintf(f, "%s", codes::reset);
    }

    fprintf(f, "\n");

    if (g_unbuffered) {
        fflush(f);
    }

    va_end(args);
}

void print(Level level, const char* filename, int line, const std::stringstream& ss)
{
    auto& o = (level >= LEVEL_ERROR) ? std::cerr : std::cout;

    if (g_colored) {
        switch (level) {
        case LEVEL_DEBUG:
            o << green;
            break;
        case LEVEL_INFO:
            o << white;
            break;
        case LEVEL_WARN:
            o << yellow;
            break;
        case LEVEL_ERROR:
            o << red;
            break;
        case LEVEL_FATAL:
            o << red;
            break;
        default:
            break;
        }
    }

    switch (level) {
    case LEVEL_DEBUG:
        o << "[DEBUG] ";
        break;
    case LEVEL_INFO:
        o << "[INFO]  ";
        break;
    case LEVEL_WARN:
        o << "[WARN]  ";
        break;
    case LEVEL_ERROR:
        o << "[ERROR] ";
        break;
    case LEVEL_FATAL:
        o << "[FATAL] ";
        break;
    default:
        break;
    }

    o << ss.str();

    if (g_show_locations) {
        o << " [" << filename << ':' << line << ']';
    }

    if (g_colored) {
        o << reset;
    }

    o << '\n';
    if (g_unbuffered) {
        o << std::flush;
    }
}

} // namespace console
} // namespace sbpl
