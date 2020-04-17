#include <Eigen/Dense>
#include <Eigen/Core>
#include <jilog.hpp>
#include <demo/app_options.hpp>
#include <demo/mpl_robot.hpp>

using namespace mpl::demo;

void isApproxTest() {
    Eigen::Matrix<double, 2, 1> a, b;
    a << 10, 10;
    b << 9, 11;
    JI_LOG(INFO) << "a: " << a << ", b: " << b << ", isApprox: " << (a-b).isMuchSmallerThan(1.415, 1) << " " << (a-b).norm();
}

void findFetchGoalsWithConds(AppOptions& app_options) {
    using Scalar = double;
    using Scenario = FetchScenario<Scalar>;
    using State = typename Scenario::State;
    using RNG = std::mt19937_64;
    using Robot = FetchRobot<Scalar>;
    using Frame = typename Robot::Frame;
    RNG rng;
    Scenario scenario = initFetchScenario<Scalar>(app_options);

    auto start = app_options.start<State>();
    Robot r(start);
    auto ee_frame = r.getEndEffectorFrame();
    Eigen::Matrix<Scalar, 3, 1> ee_pos;
    ee_pos << ee_frame(0, 3), ee_frame(1, 3), ee_frame(2,3);
    JI_LOG(INFO) << "Start " << start;
    JI_LOG(INFO) << "Start EE: " << ee_pos;
    int num_goals = 0;
    while (num_goals < 20) {
        auto rand = scenario.randomSample(rng);
        if (!scenario.isValid(rand)) continue;
        if (scenario.isValid(start, rand)) continue; // Don't want straight line paths
        Robot curr(rand);
        auto ee_frame_goal = curr.getEndEffectorFrame();
        Eigen::Matrix<Scalar, 3, 1> ee_pos_goal;
        ee_pos_goal << ee_frame_goal(0, 3), ee_frame_goal(1, 3), ee_frame_goal(2,3);
        //JI_LOG(INFO) << "Goal EE: " << ee_pos_goal;
        //JI_LOG(INFO) << (ee_pos_goal - ee_pos).norm();
        if ((ee_pos_goal - ee_pos).norm() < 0.75) continue;
        JI_LOG(INFO) << rand;
        num_goals++;
    }
}

int main(int argc, char* argv[]) {
   mpl::demo::AppOptions app_options(argc, argv);
   findFetchGoalsWithConds(app_options);
   return 0;
}

