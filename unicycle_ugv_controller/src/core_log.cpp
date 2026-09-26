#include "unicycle_ugv_controller/common/core_log.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace unicycle_ugv_controller {
namespace {

void stderrSink(LogLevel level, const char* message) {
    const char* tag =
        level == LogLevel::kError ? "ERROR" : level == LogLevel::kWarn ? "WARN" : "INFO";
    std::fprintf(stderr, "[%s] %s\n", tag, message);
}

std::atomic<LogSink> g_sink{&stderrSink};
std::atomic<double> g_core_time{0.0};

}  // namespace

void setLogSink(LogSink sink) {
    g_sink.store(sink != nullptr ? sink : &stderrSink);
}

void logMessage(LogLevel level, const char* message) {
    g_sink.load()(level, message);
}

void logFormat(LogLevel level, const char* format, ...) {
    va_list args;
    va_start(args, format);
    va_list measure;
    va_copy(measure, args);
    const int length = std::vsnprintf(nullptr, 0, format, measure);
    va_end(measure);
    std::string message(length > 0 ? static_cast<size_t>(length) : 0, '\0');
    if (length > 0)
        std::vsnprintf(&message[0], message.size() + 1, format, args);
    va_end(args);
    logMessage(level, message.c_str());
}

void setCoreTime(double seconds) {
    g_core_time.store(seconds);
}

double coreTime() {
    return g_core_time.load();
}

bool logThrottleDue(double period_s, double& last_s) {
    const double now = coreTime();
    if (now - last_s < period_s) {
        return false;
    }
    last_s = now;
    return true;
}

}  // namespace unicycle_ugv_controller
