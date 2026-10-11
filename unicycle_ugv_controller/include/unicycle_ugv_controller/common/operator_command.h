#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace unicycle_ugv_controller {

enum class OperatorCommand { kUnknown, kCustom1, kStop, kReset };

// The operator's command text, case-insensitive: "track", "tracking", "custom", "custom1" and
// "start" begin tracking, "hold" and "stop" stop it, "reset" drives to the Reset target. The ROS
// node reads the text from the command topics, the module's ROS edge from the same topics; both
// read it here.
inline OperatorCommand parseOperatorCommand(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (text == "track" || text == "tracking" || text == "custom" || text == "custom1" ||
        text == "start") {
        return OperatorCommand::kCustom1;
    }
    if (text == "hold" || text == "stop") {
        return OperatorCommand::kStop;
    }
    if (text == "reset") {
        return OperatorCommand::kReset;
    }
    return OperatorCommand::kUnknown;
}

}  // namespace unicycle_ugv_controller
