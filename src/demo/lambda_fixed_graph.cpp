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

using namespace mpl::demo;

void runPngScenario(AppOptions &app_options) {

    const std::string inputName = "./resources/png_planning_input.png";
    using Scenario = typename mpl::demo::PNG2dScenario<Scalar>;
}

int main(int argc, char *argv[]) {

    AppOptions app_options(argc, argv);
    using Scalar = double;

    if (app_options.scenario() == "png") {
        runPngScenario(app_options);
    }
    using Comm = RabbitMQMessageQueue;
    using Planner = typename mpl::PRMPlanner<Scenario, Scalar>;
    using Lambda = typename mpl::demo::LocalLambdaFixedGraph<Comm, Scenario, Planner, Scalar>;
    using State = typename Scenario::State;
    using Subspace_t = typename Lambda::Subspace_t;


    std::vector<FilterColor> filters;
    filters.emplace_back(FilterColor(126, 106, 61, 15));
    filters.emplace_back(FilterColor(61, 53, 6, 15));
    filters.emplace_back(FilterColor(255, 255, 255, 5));

    auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, inputName);

    State startState, goalState;
    startState << 430, 1300;
    goalState << 3150, 950;


    Scenario scenario((Scalar)width, (Scalar)height, goalState, obstacles);
    int lambda_id = 0;
    Comm graph_comm("localhost", 5672, "graph");
    Comm vertex_comm("localhost", 5672, "vertex");
    Subspace_t local_subspace(scenario.min(), scenario.max());
    Subspace_t global_subspace(scenario.min(), scenario.max());

    const std::string outputName = "png_2d_demo_output.svg";
    std::ofstream file(outputName);
    shape::startSvg(file, scenario.width(), scenario.height());
    shape::addBackgroundImg(file, inputName);

    Lambda lambda(lambda_id, graph_comm, vertex_comm, scenario, local_subspace, global_subspace);
    lambda.do_work();

    auto graph = lambda.graph();
    using graph_t = Planner::Graph::graph_t;
    boost::graph_traits<graph_t>::edge_iterator e, e_end;
    for(std::tie(e, e_end) = boost::edges(graph); e != e_end; ++e) {
        auto start = graph[boost::source(*e, graph)];
        auto end = graph[boost::target(*e, graph)];
        shape::addVisitedEdge(file, start.state[0], start.state[1], end.state[0], end.state[1]);
    }

    shape::endSvg(file);

    return 0;
}
