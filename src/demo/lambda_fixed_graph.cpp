#include <demo/lambda_fixed_graph.hpp>
#include <demo/png_2d_scenario.hpp>
#include <demo/shape_hierarchy.hpp>
#include <prm_planner.hpp>
#include <pq.hpp>
#include <vector>
#include <png.h>
#include <subspace.hpp>

using namespace mpl::demo;
int main(int argc, char *argv[]) {

    const std::string inputName = "./resources/png_planning_input.png";
    using Scalar = double;
    using Scenario = typename mpl::demo::PNG2dScenario<Scalar>;
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
    Lambda lambda(lambda_id, graph_comm, vertex_comm, scenario, local_subspace, global_subspace);

    //Lambda()
    //Scenario &scenario,
    //int lambda_id,
    //Comm &graph_comm,
    //Comm &vertex_comm,
    //Subspace_t &local_subspace,
    //Subspace_t &global_subspace


    return 0;
}
