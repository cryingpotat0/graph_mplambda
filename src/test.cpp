
#include <jilog.hpp>
#include <demo/reeds_shepp.hpp>
#include <demo/shape_hierarchy.hpp>
using namespace mpl::demo;
/*
#include <demo/mpl_robot.hpp>
#include <demo/multi_agent_png_2d_scenario.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <demo/app_options.hpp>
//#include <graph.hpp>

#include <interval_tree.hpp>
#include <time.h>
#include <vector>


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
    RNG rng(time(NULL));
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
        if (ee_pos_goal[0] < 0.3) continue; // x coordinate is front of the robot
        //JI_LOG(INFO) << "Goal EE: " << ee_pos_goal;
        //JI_LOG(INFO) << (ee_pos_goal - ee_pos).norm();
        if ((ee_pos_goal - ee_pos).norm() < 1.0) continue;
        JI_LOG(INFO) << rand;
        num_goals++;
    }
}

void graphSaveAndLoadTest() {
    using Scalar = double;
    using Scenario = FetchScenario<Scalar>;
    using State = typename Scenario::State;
    using Distance = typename Scenario::Distance;
    using Vertex = mpl::Vertex<State>;
    using Edge = mpl::Edge<typename Vertex::ID, Distance>;
    using Graph = mpl::UndirectedGraph<Vertex, Edge>;
    Graph graph;

    State q1; q1 << 1, 2, 3, 4, 5, 6, 7, 8; Vertex v1{std::make_pair(0,0), q1}; graph.addVertex(v1);
    State q2; q2 << 1, 2, 3, 1.1, 0, 0, 0, 8; Vertex v2{std::make_pair(1,2), q2}; graph.addVertex(v2);
    State q3; q3 << 1, 2, 3, 4.123231123, 5, 6, 7, 8; Vertex v3{std::make_pair(3,7), q3}; graph.addVertex(v3);
    
    Edge e1{1.0, v1.id(), v2.id()}; graph.addEdge(e1);
    Edge e2{2.0, v2.id(), v3.id()}; graph.addEdge(e2);
    Edge e3{3.0, v1.id(), v3.id()}; graph.addEdge(e3);

    std::ofstream file("graph_test.txt");
    graph.serialize(file);
    
    std::ifstream file_in("graph_test.txt");
    auto graph_copy = Graph::deserialize(file_in);
    std::ofstream file_copy("graph_copy_test.txt");
    graph_copy.serialize(file_copy);
    
}

void multi_agent_png_test(AppOptions& app_options) {
    using Scalar = double;
    using Scenario = MultiAgentPNG2DScenario<Scalar, NUM_AGENTS>;
    using State = typename Scenario::State;
    using Distance = typename Scenario::Distance;

    Scenario scenario = initMultiAgentPNG2DScenario<Scalar, NUM_AGENTS>(app_options);
    using RNG = std::mt19937_64;
    RNG rng;
    int num_goals = 0;
    while (num_goals < 2) {
        auto rand = scenario.randomSample(rng);
        if (!scenario.isValid(rand)) continue;
        JI_LOG(INFO) << rand;
        num_goals++;
    }

    //State q;
    //q << 100, 100, 800, 800;
    //scenario.visualizeAgentStates(q, app_options.env(), outputName);
}


void interval_tree_test() {
    using Scalar = double;
    using Value = int;
    std::vector<Interval<Scalar, Value>> intervals;
    intervals.push_back(Interval<Scalar, Value>(1, 5.5, 0));
    intervals.push_back(Interval<Scalar, Value>(2, 6, 1));
    intervals.push_back(Interval<Scalar, Value>(6, 10, 2));
    
    auto interval_tree = IntervalTree<Scalar, Value>(intervals);
    //std::vector<Interval<Scalar, Value>> results;
    auto results = interval_tree.findOverlapping(1, 2.5);
    for (auto& r: results) {
        JI_LOG(INFO) << r;
    }
}

void multi_agent_time_intersection_test() {
    using Scalar = double;
    using Scenario = MultiAgentPNG2DScenario<Scalar, NUM_AGENTS>;
    using State = typename Scenario::State;
    using SingleAgentState = typename Scenario::SingleAgentState;
    using Distance = typename Scenario::Distance;
    SingleAgentState agent1_xinit, agent1_xfinal, agent2_xinit, agent2_xfinal;
    agent1_xinit << -1, 0;
    agent1_xfinal << 1, 0;
    agent2_xinit << 0, 1;
    agent2_xfinal << 0, -1;
    
    Scalar agent1_tStart=0, agent1_tEnd=1, agent2_tStart=0, agent2_tEnd=1;
    Scalar intersection_radius = 0.1;
    Scalar agent_velocity = 1;
    JI_LOG(INFO) << Scenario::get_intersection_time(agent1_xinit, agent1_xfinal, agent2_xinit, agent2_xfinal, agent1_tStart, agent1_tEnd, agent2_tStart, agent2_tEnd, intersection_radius, agent_velocity);
    //Scenario scenario = initMultiAgentPNG2DScenario<Scalar, NUM_AGENTS>(app_options);
}
*/

void reedsSheppTest() {
    double q0[3] = {50,20,0};
    double q1[3] = {100,100,0};
    auto velocity = 50.0;
    ReedsSheppStateSpace reedsSheppStateSpace(velocity);

    std::vector<std::vector<double>> sampled_path;
    sampled_path.clear();
    reedsSheppStateSpace.sample(q0, q1, 0.5, [&] (double q[3]) {
          sampled_path.push_back({q[0], q[1], q[2]});
          return false;
        });
    // The function inside sample returns true if sampling should be stopped
    
    //for (auto point: sampled_path) {
    //    JI_LOG(INFO) << point[0] << " " << point[1] << " " << point[2];
    //}
    std::ofstream file("test_reed_shepp.svg");
    shape::startSvg(file, 200, 200);
    shape::addPath(file, sampled_path);
    shape::addDubinsCar(file, sampled_path, velocity);
    shape::endSvg(file);

    //auto path = reedsSheppStateSpace.reedsShepp(q0, q1);
    //for (int i=0; i < 5; ++i) {
    //    JI_LOG(INFO) << "Type " << path.type_[i] << " Length: " << path.length_[i];
    //}
}

int main(int argc, char* argv[]) {
    //mpl::demo::AppOptions app_options(argc, argv);
    //isApproxTest();
    //findFetchGoalsWithConds(app_options);
    //graphSaveAndLoadTest();
    //multi_agent_png_test(app_options);
    //interval_tree_test();
    //multi_agent_time_intersection_test();
    reedsSheppTest();
    return 0;
}

