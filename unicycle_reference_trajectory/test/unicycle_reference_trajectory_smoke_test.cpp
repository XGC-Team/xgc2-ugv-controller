#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <xgc2_math/trajectory.hpp>

TEST(UnicycleReferenceTrajectorySmoke, AnalyticEvaluatorsDoNotProduceNonFiniteOutput) {
    namespace trajectory = xgc2_math::trajectory;
    std::vector<std::unique_ptr<trajectory::TrajectoryEvaluator2>> evaluators;
    evaluators.push_back(std::make_unique<trajectory::HoldCurveEvaluator2>());
    trajectory::CircleCurveParameters2 circle;
    circle.radius = 3.0;
    circle.line_speed = 1.5;
    evaluators.push_back(std::make_unique<trajectory::CircleCurveEvaluator2>(circle));
    trajectory::CircleEntryCurveParameters2 entry;
    entry.entry_duration = 2.0;
    entry.circle = circle;
    evaluators.push_back(std::make_unique<trajectory::CircleEntryCurveEvaluator2>(entry));
    trajectory::FigureEightCurveParameters2 figure_eight;
    figure_eight.radius = 3.0;
    figure_eight.line_speed = 1.5;
    evaluators.push_back(std::make_unique<trajectory::FigureEightCurveEvaluator2>(figure_eight));

    for (const auto& evaluator : evaluators) {
        trajectory::PlanarReference2 output;
        ASSERT_TRUE(evaluator->evaluate(0.5, output));
        EXPECT_TRUE(trajectory::TrajectoryValidator2::finite(output));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
