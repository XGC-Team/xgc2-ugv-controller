// The scenario harness of the Reset design as a test (RESET.md section 7): the real coordinator, the
// real executors, a physical plant, on the 235 scenarios of the TASE scene.
//
//   ResetScenarios.QuickFamily   every third scenario of the 80 scenario subset: runs with the
//                                package tests (about a minute)
//   ResetScenarios.FullFamily    the 235 scenarios, with the plant, offset, noise and outage rows of
//                                the design table; set RESET_SCENARIOS_FULL=1 (about half an hour)
//
// The data directory is RESET_TEST_DATA_DIR (ugv_reset_safety/test/data).

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <numeric>

#include "reset_harness.h"

namespace reset_harness {
namespace {

std::string dataFile(const std::string& name) {
    return std::string(RESET_TEST_DATA_DIR) + "/" + name;
}

std::string family(const std::string& name) {
    const auto cut = name.rfind('_');
    return cut != std::string::npos && std::isdigit(static_cast<unsigned char>(name.back())) ? name.substr(0, cut) : name;
}

struct Summary {
    int n{0}, completed{0}, collisions{0}, contacts_planning{0};
    std::vector<double> times;
    std::vector<double> work_ms;
    int max_reversals{0};
    std::map<std::string, int> failures;
    double max_cross{0.0};
    double worst_planning_clear{1e9};
};

Summary summarize(const std::vector<Outcome>& outcomes, const std::string& only = "") {
    Summary s;
    for (const Outcome& o : outcomes) {
        if (!only.empty() && family(o.name) != only) {
            continue;
        }
        ++s.n;
        s.completed += o.completed ? 1 : 0;
        s.collisions += o.collision ? 1 : 0;
        if (o.completed) {
            s.times.push_back(o.t_last);
        } else {
            ++s.failures[o.outcomes];
        }
        s.work_ms.insert(s.work_ms.end(), o.work_ms.begin(), o.work_ms.end());
        s.max_reversals = std::max(s.max_reversals, o.reversals);
        s.max_cross = std::max(s.max_cross, o.max_cross);
        s.worst_planning_clear = std::min(s.worst_planning_clear, o.min_planning_clear);
    }
    std::sort(s.times.begin(), s.times.end());
    std::sort(s.work_ms.begin(), s.work_ms.end());
    return s;
}

void print(const std::string& label, const std::vector<Outcome>& outcomes) {
    std::vector<std::string> families;
    for (const Outcome& o : outcomes) {
        const std::string f = family(o.name);
        if (std::find(families.begin(), families.end(), f) == families.end()) {
            families.push_back(f);
        }
    }
    families.push_back("");
    for (const std::string& f : families) {
        const Summary s = summarize(outcomes, f);
        std::printf("%-26s %-12s n=%3d completed=%3d (%3.0f%%) collisions=%d  t_med=%5.1f  t_p90=%5.1f  "
                    "plan ms p50=%5.2f p95=%6.2f  chatter<=%d  cross<=%.2f\n",
                    label.c_str(), f.empty() ? "ALL" : f.c_str(), s.n, s.completed,
                    100.0 * s.completed / std::max(1, s.n), s.collisions,
                    s.times.empty() ? 0.0 : s.times[s.times.size() / 2],
                    s.times.empty() ? 0.0 : s.times[static_cast<size_t>(0.9 * (s.times.size() - 1))],
                    s.work_ms.empty() ? 0.0 : s.work_ms[s.work_ms.size() / 2],
                    s.work_ms.empty() ? 0.0 : s.work_ms[static_cast<size_t>(0.95 * (s.work_ms.size() - 1))],
                    s.max_reversals, s.max_cross);
    }
    const Summary all = summarize(outcomes);
    if (!all.failures.empty()) {
        std::printf("%-26s unfinished outcomes:", label.c_str());
        for (const auto& f : all.failures) {
            std::printf(" %s x%d", f.first.c_str(), f.second);
        }
        std::printf("\n");
    }
}

std::vector<Outcome> runAll(const ScenarioFile& world, const Options& options, size_t stride = 1,
                            size_t offset = 0) {
    std::vector<Outcome> outcomes;
    for (size_t i = offset; i < world.scenarios.size(); i += stride) {
        Options o = options;
        o.seed += static_cast<unsigned>(i);
        outcomes.push_back(run(world, world.scenarios[i], o));
    }
    return outcomes;
}

// Safety: the physical chassis never touched anything, and the planned footprints of vehicles that
// started clear did not overlap. (The 0.10 m pair margin is the planner's design clearance; the
// tracker's deviation consumes part of it, the chassis is 0.14 m a side smaller than the envelope.)
void expectSafe(const std::vector<Outcome>& outcomes, const std::string& label) {
    for (const Outcome& o : outcomes) {
        if (std::getenv("RESET_SCENARIOS_VERBOSE") != nullptr) {
            std::printf("  %-14s %s completed=%d t=%5.1f cross=%.3f planning=%.3f chassis=%.3f chatter=%d  (planning min at %.1f s, %s)\n", o.name.c_str(),
                        o.outcomes.c_str(), o.completed, o.t_last, o.max_cross, o.min_planning_clear, o.min_clear, o.reversals,
                        o.min_planning_at, o.min_planning_pair.c_str());
            if (o.max_cross > 0.10) {
                std::printf("      max cross-track: %s\n", o.max_cross_where.c_str());
            }
        }
        EXPECT_FALSE(o.collision) << label << " " << o.name << " min clearance " << o.min_clear;
        EXPECT_GT(o.min_planning_clear, 0.0) << label << " " << o.name << ": planning footprints overlapped";
    }
}

TEST(ResetScenarios, QuickFamily) {
    const ScenarioFile world = loadScenarios(dataFile("tase_scenarios_subset80.txt"));
    const std::vector<Outcome> outcomes = runAll(world, Options(), 3);
    print("quick (known offset)", outcomes);
    expectSafe(outcomes, "quick");
    const Summary s = summarize(outcomes);
    EXPECT_GE(s.completed, static_cast<int>(0.75 * s.n)) << "the design completes about 85% within 90 s";
    // Every unfinished run carries a typed result at 90 s: nothing ends silent.
    for (const Outcome& o : outcomes) {
        EXPECT_EQ(o.outcomes.find('.'), std::string::npos) << o.name << " " << o.outcomes;
    }
}

// The SCE pair of the rostest in the closed loop: two omnidirectional vehicles head-on on one line.
TEST(ResetScenarios, OmniHeadOn) {
    ScenarioFile world;
    Scenario s;
    s.name = "omni_head_on";
    s.starts = {Eigen::Vector3d(-2.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 3.14159265)};
    s.goals = {Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(-2.0, 0.0, 3.14159265)};
    world.scenarios.push_back(s);
    Options o;
    o.mecanum = true;
    const std::vector<Outcome> outcomes = runAll(world, o);
    expectSafe(outcomes, "omni head-on");
    EXPECT_TRUE(outcomes[0].completed) << outcomes[0].outcomes;
}

TEST(ResetScenarios, FullFamily) {
    if (std::getenv("RESET_SCENARIOS_FULL") == nullptr) {
        GTEST_SKIP() << "set RESET_SCENARIOS_FULL=1 to run the 235 scenarios and the sensitivity rows";
    }
    const ScenarioFile all = loadScenarios(dataFile("tase_scenarios.txt"));
    const ScenarioFile subset = loadScenarios(dataFile("tase_scenarios_subset80.txt"));
    Options known;
    const auto full_known = runAll(all, known);
    print("full, offset known", full_known);
    expectSafe(full_known, "full known");
    Options unknown;
    unknown.teach = false;
    const auto full_unknown = runAll(all, unknown);
    print("full, offset unknown", full_unknown);
    for (const Outcome& o : full_unknown) {
        EXPECT_FALSE(o.collision) << "unknown " << o.name << " min clearance " << o.min_clear;
    }
    struct Row {
        const char* label;
        Options options;
    };
    std::vector<Row> rows;
    {
        Options o;
        o.lag_scale = 0.5;
        rows.push_back({"lag x0.5", o});
        o.lag_scale = 2.0;
        rows.push_back({"lag x2", o});
        o = Options();
        o.ell = 0.229;
        rows.push_back({"offset sign flipped", o});
        o = Options();
        o.pos_noise = 0.001;
        rows.push_back({"pose noise 1 mm", o});
        o = Options();
        o.outage_start = 12.0;
        o.outage_length = 0.8;
        rows.push_back({"coordinator outage 0.8 s", o});
        o = Options();
        o.loss = 0.1;
        rows.push_back({"bus loss 10%", o});
        o = Options();
        o.mecanum = true;
        o.ell = 0.0;
        rows.push_back({"mecanum 0.41 x 0.36", o});
    }
    for (const Row& row : rows) {
        const auto outcomes = runAll(subset, row.options);
        print(row.label, outcomes);
        for (const Outcome& o : outcomes) {
            EXPECT_FALSE(o.collision) << row.label << " " << o.name << " min clearance " << o.min_clear;
        }
    }
    const Summary s = summarize(full_known);
    EXPECT_GE(s.completed, static_cast<int>(0.80 * s.n));
}

}  // namespace
}  // namespace reset_harness
