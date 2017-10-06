#include <smpl/console/console.h>

// standard includes
#include <stdarg.h>
#include <string.h>
#include <iostream>

// project includes
#include <smpl/console/ansi.h>

#ifndef SMPL_CONSOLE_LOG_FILE_AND_LINE
#define SMPL_CONSOLE_LOG_FILE_AND_LINE 0
#endif

#ifndef SMPL_CONSOLE_COLORIZE_OUTPUT
#define SMPL_CONSOLE_COLORIZE_OUTPUT 0
#endif

#ifndef SMPL_CONSOLE_BUFFERED
#define SMPL_CONSOLE_BUFFERED 1
#endif

namespace sbpl {
namespace console {

void InitializeLogLocation(
    LogLocation* loc,
    const std::string& name,
    Level level)
{

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

#if SMPL_CONSOLE_COLORIZE_OUTPUT
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
#endif

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
#if SMPL_CONSOLE_LOG_FILE_AND_LINE
    const char* base = strrchr(filename, '\\');
    if (base) {
        fprintf(f, " [%s:%d]", base + 1, line);
    } else {
        fprintf(f, " [%s:%d]", filename, line);
    }
#endif

#if SMPL_CONSOLE_COLORIZE_OUTPUT
    fprintf(f, "%s", codes::reset);
#endif

    fprintf(f, "\n");

#if SMPL_CONSOLE_BUFFERED
    fflush(f);
#endif

    va_end(args);
}

void print(Level level, const char* filename, int line, const std::stringstream& ss)
{
    auto& o = (level >= LEVEL_ERROR) ? std::cerr : std::cout;

#if SMPL_CONSOLE_COLORIZE_OUTPUT
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
#endif

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

#if SMPL_CONSOLE_LOG_FILE_AND_LINE
    o << " [" << filename << ':' << line << ']';
#endif

#if SMPL_CONSOLE_COLORIZE_OUTPUT
    o << reset;
#endif

    o << '\n';
#if SMPL_CONSOLE_BUFFERED
    o << std::flush;
#endif
}

} // namespace console
} // namespace sbpl
