#pragma once

// Logging for the mecanum controller core, without ROS. The core formats a
// line and hands it to one process-wide sink: stderr by default, rosconsole
// when the ROS node installs its sink, or whatever an
// aggregator module installs.
//
// Throttled messages measure the core clock: the time last handed to
// MecanumUgvController::update, never a clock the core reads itself.
// ROS_*_THROTTLE measured ROS time, which is what the ROS node hands over.

#include <sstream>

namespace mecanum_ugv_controller {

enum class LogLevel { kInfo, kWarn, kError };
using LogSink = void (*)(LogLevel level, const char* message);

void setLogSink(LogSink sink);
void logMessage(LogLevel level, const char* message);
void logFormat(LogLevel level, const char* format, ...) __attribute__((format(printf, 2, 3)));

// The latest absolute time [s] handed to the core, shared by the process.
void setCoreTime(double seconds);
double coreTime();

// True at most once per `period_s` of core time for one call site (`last_s`
// is that call site's own state).
bool logThrottleDue(double period_s, double& last_s);

}  // namespace mecanum_ugv_controller

#define MECANUM_LOG_INFO(...) \
    ::mecanum_ugv_controller::logFormat(::mecanum_ugv_controller::LogLevel::kInfo, __VA_ARGS__)
#define MECANUM_LOG_WARN(...) \
    ::mecanum_ugv_controller::logFormat(::mecanum_ugv_controller::LogLevel::kWarn, __VA_ARGS__)
#define MECANUM_LOG_ERROR(...) \
    ::mecanum_ugv_controller::logFormat(::mecanum_ugv_controller::LogLevel::kError, __VA_ARGS__)

#define MECANUM_LOG_STREAM_(level, args)                                                  \
    do {                                                                                  \
        std::ostringstream mecanum_log_stream_;                                           \
        mecanum_log_stream_ << args;                                                      \
        ::mecanum_ugv_controller::logMessage((level), mecanum_log_stream_.str().c_str()); \
    } while (0)
#define MECANUM_LOG_INFO_STREAM(args) \
    MECANUM_LOG_STREAM_(::mecanum_ugv_controller::LogLevel::kInfo, args)
#define MECANUM_LOG_WARN_STREAM(args) \
    MECANUM_LOG_STREAM_(::mecanum_ugv_controller::LogLevel::kWarn, args)
#define MECANUM_LOG_ERROR_STREAM(args) \
    MECANUM_LOG_STREAM_(::mecanum_ugv_controller::LogLevel::kError, args)

#define MECANUM_LOG_THROTTLE_(period, statement)                                   \
    do {                                                                           \
        static thread_local double mecanum_log_last_ = -1.0e300;                   \
        if (::mecanum_ugv_controller::logThrottleDue((period), mecanum_log_last_)) \
            statement;                                                             \
    } while (0)
#define MECANUM_LOG_INFO_THROTTLE(period, ...) \
    MECANUM_LOG_THROTTLE_(period, MECANUM_LOG_INFO(__VA_ARGS__))
#define MECANUM_LOG_WARN_THROTTLE(period, ...) \
    MECANUM_LOG_THROTTLE_(period, MECANUM_LOG_WARN(__VA_ARGS__))
#define MECANUM_LOG_ERROR_THROTTLE(period, ...) \
    MECANUM_LOG_THROTTLE_(period, MECANUM_LOG_ERROR(__VA_ARGS__))
#define MECANUM_LOG_WARN_STREAM_THROTTLE(period, args) \
    MECANUM_LOG_THROTTLE_(period, MECANUM_LOG_WARN_STREAM(args))
#define MECANUM_LOG_ERROR_STREAM_THROTTLE(period, args) \
    MECANUM_LOG_THROTTLE_(period, MECANUM_LOG_ERROR_STREAM(args))
