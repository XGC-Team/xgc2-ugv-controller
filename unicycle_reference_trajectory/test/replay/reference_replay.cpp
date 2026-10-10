// Writes the trace of the deterministic replay of the ROS-free reference
// trajectory core (reference_replay_core.h). The trace is compared by hash in
// unicycle_reference_trajectory_replay_test; this program writes it out so two
// builds, or this and a recorded one, can be compared with `cmp`.
//
// Usage: unicycle_reference_trajectory_replay OUT.txt

#include <cstdio>
#include <exception>

#include "reference_replay_core.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: unicycle_reference_trajectory_replay OUT.txt\n");
        return 2;
    }
    FILE* out = std::fopen(argv[1], "w");
    if (!out) {
        return 2;
    }
    try {
        reference_replay_core::writeTrace(out);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "replay failed: %s\n", error.what());
        std::fclose(out);
        return 1;
    }
    std::fclose(out);
    return 0;
}
