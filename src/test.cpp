
#include <demo/mpl_robot.hpp>
#include <demo/multi_agent_png_2d_scenario.hpp>
#include <prm_planner.hpp>
#include <jilog.hpp>
#include <demo/se3_rigid_body_scenario.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <demo/app_options.hpp>
//#include <graph.hpp>

#include <interval_tree.hpp>
#include <time.h>
#include <vector>
using namespace mpl::demo;


/*
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
 */

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

void findSE3GoalsWithConds(AppOptions& app_options) {
    using Scalar = double;
    using Scenario = SE3RigidBodyScenario<Scalar>;
    using State = typename Scenario::State;
    using RNG = std::mt19937_64;
    Scenario scenario = initSE3Scenario<Scalar>(app_options);

    auto start = app_options.start<State>();
    JI_LOG(INFO) << "Start " << start;
    int num_goals = 0;
    RNG rng(time(NULL));
    while (num_goals < 20) {
        auto rand = scenario.randomSample(rng);
        if (!scenario.isValid(rand)) continue;
        if (scenario.isValid(start, rand)) continue; // Don't want straight line paths
        //Robot curr(rand);
        //auto ee_frame_goal = curr.getEndEffectorFrame();
        //Eigen::Matrix<Scalar, 3, 1> ee_pos_goal;
        //ee_pos_goal << ee_frame_goal(0, 3), ee_frame_goal(1, 3), ee_frame_goal(2,3);
        //if (ee_pos_goal[0] < 0.3) continue; // x coordinate is front of the robot
        ////JI_LOG(INFO) << "Goal EE: " << ee_pos_goal;
        ////JI_LOG(INFO) << (ee_pos_goal - ee_pos).norm();
        //if ((ee_pos_goal - ee_pos).norm() < 1.0) continue;
        JI_LOG(INFO) << rand;
        num_goals++;
    }
}


void rngStateGenerationTest(AppOptions& app_options) {
    using Scalar = double;
    using Scenario = SE3RigidBodyScenario<Scalar>;
    using State = typename Scenario::State;
    using Planner = typename mpl::PRMPlanner<Scenario, Scalar>;
    using RNG = std::mt19937_64;
    Scenario scenario = initSE3Scenario<Scalar>(app_options);
    Planner planner(scenario, 0, false);

    auto start = app_options.start<State>();
    JI_LOG(INFO) << "Start " << start;
    planner.setSeed(app_options.randomSeed());
    
    //int num_goals = 0;
    //RNG rng(time(NULL));
    //while (num_goals < 20) {
    //    auto rand = scenario.randomSample(rng);
    //    if (!scenario.isValid(rand)) continue;
    //    if (scenario.isValid(start, rand)) continue; // Don't want straight line paths
    //    //Robot curr(rand);
    //    //auto ee_frame_goal = curr.getEndEffectorFrame();
    //    //Eigen::Matrix<Scalar, 3, 1> ee_pos_goal;
    //    //ee_pos_goal << ee_frame_goal(0, 3), ee_frame_goal(1, 3), ee_frame_goal(2,3);
    //    //if (ee_pos_goal[0] < 0.3) continue; // x coordinate is front of the robot
    //    ////JI_LOG(INFO) << "Goal EE: " << ee_pos_goal;
    //    ////JI_LOG(INFO) << (ee_pos_goal - ee_pos).norm();
    //    //if ((ee_pos_goal - ee_pos).norm() < 1.0) continue;
    //    JI_LOG(INFO) << rand;
    //    num_goals++;
    //}
}

void generateSequentialGraphTest(AppOptions& app_options) {

    using Scalar = double;
    /* using Scenario = SE3RigidBodyScenario<Scalar>; */
    using Scenario = FetchScenario<Scalar>;
    using Distance = Scenario::Distance;
    using State = typename Scenario::State;
    using Vertex = mpl::Vertex<State>;
    using Vertex_ID = typename Vertex::ID;
    using Edge = mpl::Edge<typename Vertex::ID, Distance>;
    using Graph = mpl::UndirectedGraph<Vertex, Edge>;
    /* Scenario scenario = initSE3Scenario<Scalar>(app_options); */
            JI_LOG(INFO) << "here ";
    Scenario scenario = initFetchScenario<Scalar>(app_options);
            JI_LOG(INFO) << "here ";
    Graph g;
    generateSequentialGraph(scenario, app_options, g, 0, app_options.graphSize());

    JI_LOG(INFO) << "Done generating graph ";
    std::ofstream file("graph_test.txt");
    g.serialize(file);

}

void testCollisionTime() {

    using Scalar = double;
    using Scenario = FetchScenario<Scalar>;
    using State = typename Scenario::State;
    using Bound = typename Scenario::Bound;
    using Frame = typename Scenario::Frame;
    using GoalRadius = Eigen::Matrix<Scalar, 6, 1>;
    using RNG = std::mt19937_64;
    // TODO: hardcoded values in SE3, unused in code for now
    AppOptions app_options;
    app_options.correct_goal_ = "-1.07,0.16,0.88,0,0,0";
    app_options.goalRadius_ = "0.01,0.01,0.01,0.01,0.01,3.141592653589793";
    app_options.envFrame_ = "0.48,1.09,0.00,0,0,-1.570796326794897";
    app_options.env_ = "resources/AUTOLAB.dae";
    app_options.global_min_ = "0,-1.6056,-1.221,-3.141592653589793,-2.251,-3.141592653589793,-2.16,-3.141592653589793";
    app_options.global_max_ = "0.38615,1.6056,1.518,3.141592653589793,2.251,3.141592653589793,2.16,3.141592653589793";

    Frame envFrame = app_options.envFrame<Frame>();
    Frame goal = app_options.correct_goal<Frame>();
    GoalRadius goalRadius = app_options.goalRadius<GoalRadius>();
    auto min = app_options.globalMin<Bound>();
    auto max = app_options.globalMax<Bound>();

    Scenario scenario(envFrame, app_options.env(), goal, goalRadius, min, max, app_options.checkResolution(0.1));


    RNG rng(0);
    auto start = std::chrono::high_resolution_clock::now();
    int valid_duration_count = 0;
    for (int i=0; i < 100; ++i) {
        auto rand1 = scenario.randomSample(rng);
        while (!scenario.isValid(rand1)) {
            rand1 = scenario.randomSample(rng);
        }

        auto rand2 = scenario.randomSample(rng);
        while (!scenario.isValid(rand2)) {
            rand2 = scenario.randomSample(rng);
        }
        auto valid_start = std::chrono::high_resolution_clock::now();
        JI_LOG(INFO) << "rand1 " << rand1 << " rand2 " << rand2;
        scenario.isValid(rand1, rand2) ;
        auto valid_stop = std::chrono::high_resolution_clock::now();
        auto valid_duration = std::chrono::duration_cast<std::chrono::milliseconds>(valid_stop - valid_start);
        valid_duration_count += valid_duration.count();
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    JI_LOG(INFO) << "Duration " << duration.count() << " valid_duration " << valid_duration_count;
}

int main(int argc, char* argv[]) {
    mpl::demo::AppOptions app_options(argc, argv);
    //isApproxTest();
    //findFetchGoalsWithConds(app_options);
    //graphSaveAndLoadTest();
    //multi_agent_png_test(app_options);
    //interval_tree_test();
    //multi_agent_time_intersection_test();
    /* findSE3GoalsWithConds(app_options); */
    generateSequentialGraphTest(app_options);
    /* testCollisionTime(); */
    return 0;
}

