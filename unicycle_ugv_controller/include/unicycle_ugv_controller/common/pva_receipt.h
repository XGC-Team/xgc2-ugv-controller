#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace unicycle_ugv_controller {

// A diagnostic receipt has two clock readings, not a substituted source stamp.
struct PvaReceipt {
    std::uint32_t source_sec = 0;
    std::uint32_t source_nsec = 0;
    std::uint32_t received_sec = 0;
    std::uint32_t received_nsec = 0;
    std::uint32_t source_sequence = 0;
    std::uint64_t received_sequence = 0;
    // Accepted world-frame [x, y, yaw, vx, vy, ax, ay], in SI units.
    std::array<double, 7> pva{};
};

inline std::string serializePvaReceipt(const PvaReceipt& receipt) {
    if (receipt.source_nsec >= 1000000000U || receipt.received_nsec >= 1000000000U) {
        throw std::invalid_argument("PVA receipt nanoseconds are out of range");
    }
    for (double value : receipt.pva) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("PVA receipt contains a non-finite value");
        }
    }
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\"schema\":\"unicycle.pva-receipt/v1\",\"source_sec\":" << receipt.source_sec
           << ",\"source_nsec\":" << receipt.source_nsec
           << ",\"received_sec\":" << receipt.received_sec
           << ",\"received_nsec\":" << receipt.received_nsec
           << ",\"source_sequence\":" << receipt.source_sequence
           << ",\"received_sequence\":" << receipt.received_sequence << ",\"pva\":[";
    for (std::size_t i = 0; i < receipt.pva.size(); ++i) {
        if (i != 0) {
            stream << ',';
        }
        stream << receipt.pva[i];
    }
    stream << "]}";
    return stream.str();
}

}  // namespace unicycle_ugv_controller
