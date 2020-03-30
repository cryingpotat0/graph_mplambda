#include <demo/lambda_fixed_graph.hpp>
#include <demo/png_2d_scenario.hpp>
#include <demo/shape_hierarchy.hpp>
#include <prm_planner.hpp>
#include <pq.hpp>
#include <vector>
#include <png.h>
#include <subspace.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <demo/app_options.hpp>
#include <jilog.hpp>
#include <util.hpp>
#include <comm.hpp>


using namespace mpl::demo;

template <class Scalar>
void runPngScenario(AppOptions &app_options) {
    if (app_options.env(false).empty()) {
        app_options.env_ = "./resources/png_planning_input.png";
    }

    using Scenario = typename mpl::demo::PNG2dScenario<Scalar>;
    using Planner = typename mpl::PRMPlanner<Scenario, Scalar>;
    using Lambda = typename mpl::demo::LocalLambdaFixedGraph<mpl::Comm, Scenario, Planner, Scalar>;
    using State = typename Scenario::State;
    using Bound = typename Scenario::Bound;
    using Subspace_t = typename Lambda::Subspace_t;

    std::vector<FilterColor> filters;

    if (app_options.env() == "./resources/png_planning_input.png") {
        filters.emplace_back(FilterColor(126, 106, 61, 15));
        filters.emplace_back(FilterColor(61, 53, 6, 15));
        filters.emplace_back(FilterColor(255, 255, 255, 5));
    }

    auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
    auto startState = app_options.start<State>(); // 430, 1300;
    auto goalState = app_options.goal<State>(); // 3150, 950

    JI_LOG(INFO) << "Using env " << app_options.env() << " with start " << startState << " and end " << goalState;

    if (app_options.global_min_.empty()) {
        app_options.global_min_ = "0,0";
    }
    if (app_options.global_max_.empty()) {
        app_options.global_max_ = mpl::util::ToString(width) + "," + mpl::util::ToString(height);
    }

    auto min = app_options.globalMin<Bound>();
    auto max = app_options.globalMax<Bound>();
    auto local_min = app_options.min<Bound>();
    auto local_max = app_options.max<Bound>();


    Subspace_t local_subspace(local_min, local_max);
    Subspace_t global_subspace(min, max);

    JI_LOG(INFO) << "Lambda " << app_options.lambdaId() << " local subspace " << local_subspace;

    Scenario scenario(width, height, local_min, local_max, goalState, obstacles);
    Lambda lambda(app_options, scenario, local_subspace, global_subspace);
    for(;;) {
//    for(int i=0; i < 2; ++i) {
//        comm.poll();
        lambda.do_work();
//        comm.flush();
    }

//    const std::string outputName = "png_2d_demo_output.svg";
//    std::ofstream file(outputName);
//    shape::startSvg(file, max[0], max[1]);
//    shape::addBackgroundImg(file, app_options.env());
//
//
//    using Graph = typename Planner::Graph;
//    Graph graph = lambda.getPlanner().getGraph();
//    for (auto& [v_id, u_ids] : graph.getAdjacencyList()) {
//        auto start = graph.getVertex(v_id);
//        for (auto u_id : u_ids) {
//            auto end = graph.getVertex(u_id);
//            shape::addVisitedEdge(file, start.state[0], start.state[1], end.state[0], end.state[1]);
//        }
//    }
//    shape::endSvg(file);

}

int main(int argc, char *argv[]) {

    JI_LOG(INFO) << argc << " " << argv;
    AppOptions app_options(argc, argv);
    using Scalar = double; // TODO: add single precision code

    if (app_options.communicator(false).empty()) {
        app_options.communicator_ = "rabbitmq";
    }

    if (app_options.coordinator(false).empty()) {
        app_options.coordinator_ = "localhost";
    }

    // set defaults outside

    if (app_options.communicator() == "rabbitmq") {
    } else {
        throw std::invalid_argument("Invalid comm");
    }
    JI_LOG(INFO) << "Using coordinator " << app_options.coordinator()
            << " with communicator " << app_options.communicator();

    if (app_options.scenario() == "png") {
        runPngScenario<Scalar>(app_options);
    }
    return 0;
}

//
//int main(int argc, char *argv[]) {
//    int send_flag = 0;
//    auto worker_queue = RabbitMQMessageQueue("localhost", 5672, "work");
//    auto graph_queue = RabbitMQMessageQueue("localhost", 5672, "graph");
//    static struct option longopts[] = {
//            { "send", no_argument, &send_flag, 1 },
//            { NULL, 0, NULL, 0 }
//    };
//
//    for (int ch ; (ch = ::getopt_long(argc, argv, "", longopts, NULL)) != -1 ; ) { }
//    if (send_flag) {
//        std::string test = "teeeeeeessssssstttt";
//        worker_queue.put_raw(test, "work");
//    } else {
//        while(1) JI_LOG(INFO) << "output " << worker_queue.get_raw("work");
//    }
//}