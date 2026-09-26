#pragma once

// Logging for the unicycle controller core, without ROS. The core formats a
// line and hands it to one process-wide sink: stderr by default, rosconsole
// when the ROS node installs its sink (ros_log_sink.h), or whatever an
// aggregator module installs.
//
// Throttled messages measure the core clock: the time last handed to
// UnicycleUgvController::update, never a clock the core reads itself.
// ROS_*_THROTTLE measured ROS time, which is what the ROS node hands over.

#include <sstream>

namespace unicycle_ugv_controller {

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

}  // namespace unicycle_ugv_controller

#define UGV_LOG_INFO(...) \
    ::unicycle_ugv_controller::logFormat(::unicycle_ugv_controller::LogLevel::kInfo, __VA_ARGS__)
#define UGV_LOG_WARN(...) \
    ::unicycle_ugv_controller::logFormat(::unicycle_ugv_controller::LogLevel::kWarn, __VA_ARGS__)
#define UGV_LOG_ERROR(...) \
    ::unicycle_ugv_controller::logFormat(::unicycle_ugv_controller::LogLevel::kError, __VA_ARGS__)

#define UGV_LOG_STREAM_(level, args)                                                   \
    do {                                                                               \
        std::ostringstream ugv_log_stream_;                                            \
        ugv_log_stream_ << args;                                                       \
        ::unicycle_ugv_controller::logMessage((level), ugv_log_stream_.str().c_str()); \
    } while (0)
#define UGV_LOG_INFO_STREAM(args) UGV_LOG_STREAM_(::unicycle_ugv_controller::LogLevel::kInfo, args)
#define UGV_LOG_WARN_STREAM(args) UGV_LOG_STREAM_(::unicycle_ugv_controller::LogLevel::kWarn, args)
#define UGV_LOG_ERROR_STREAM(args) \
    UGV_LOG_STREAM_(::unicycle_ugv_controller::LogLevel::kError, args)

#define UGV_LOG_THROTTLE_(period, statement)                                    \
    do {                                                                        \
        static thread_local double ugv_log_last_ = -1.0e300;                    \
        if (::unicycle_ugv_controller::logThrottleDue((period), ugv_log_last_)) \
            statement;                                                          \
    } while (0)
#define UGV_LOG_INFO_THROTTLE(period, ...) UGV_LOG_THROTTLE_(period, UGV_LOG_INFO(__VA_ARGS__))
#define UGV_LOG_WARN_THROTTLE(period, ...) UGV_LOG_THROTTLE_(period, UGV_LOG_WARN(__VA_ARGS__))
#define UGV_LOG_ERROR_THROTTLE(period, ...) UGV_LOG_THROTTLE_(period, UGV_LOG_ERROR(__VA_ARGS__))
#define UGV_LOG_WARN_STREAM_THROTTLE(period, args) \
    UGV_LOG_THROTTLE_(period, UGV_LOG_WARN_STREAM(args))
#define UGV_LOG_ERROR_STREAM_THROTTLE(period, args) \
    UGV_LOG_THROTTLE_(period, UGV_LOG_ERROR_STREAM(args))
