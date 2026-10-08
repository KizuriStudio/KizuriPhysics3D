// KizuriPhysics - Core/Diagnostics.cpp
#include "Kizuri/Core/Types.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace kizuri::detail {

void KzAssertFail(const char* expr, const char* file, int line, const char* msg) {
    std::fprintf(stderr, "\n[KizuriPhysics] ASSERTION FAILED\n  expr: %s\n  file: %s:%d\n",
                 expr ? expr : "(null)", file ? file : "?", line);
    if (msg) std::fprintf(stderr, "  msg : %s\n", msg);
    std::fflush(stderr);
    std::abort();
}

void KzFatalFail(const char* file, int line, const char* msg) {
    std::fprintf(stderr, "\n[KizuriPhysics] FATAL ERROR\n  file: %s:%d\n  msg : %s\n",
                 file ? file : "?", line, msg ? msg : "(null)");
    std::fflush(stderr);
    std::abort();
}

void KzLogImpl(int level, const char* file, int line, const char* fmt, ...) {
    static const char* kLevels[] = { "TRACE", "DEBUG", "INFO", "WARN", "ERROR" };
    const char* levelName = (level >= 0 && level < 5) ? kLevels[level] : "?";
    std::fprintf(stderr, "[Kizuri/%s] %s:%d: ", levelName, file ? file : "?", line);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
    std::fprintf(stderr, "\n");
}

} // namespace kizuri::detail
