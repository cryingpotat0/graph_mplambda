#include <coordinator.hpp>
//#include <pq.hpp>
//#include <jilog.hpp>
//#include <getopt.h>
//#include <string>
//#include <demo/app_options.hpp>
#include <demo/png_2d_scenario.hpp>
#include <demo/shape_hierarchy.hpp>
//#include <demo/fetch_scenario.hpp>
//#include <comm.hpp>
//#include <poll.h>
//#include <packet.hpp>
//#include <buffer.hpp>
//#include <write_queue.hpp>


using namespace mpl::demo;

template <class Coordinator, class State>
void savePngImages(const Coordinator& coord, const AppOptions &app_options) {

    std::vector<FilterColor> filters;
    if (app_options.env() == "./resources/png_planning_input.png") {
        filters.emplace_back(FilterColor(126, 106, 61, 15));
        filters.emplace_back(FilterColor(61, 53, 6, 15));
        filters.emplace_back(FilterColor(255, 255, 255, 5));
    }

    auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
    auto startState = app_options.start<State>(); // 430, 1300;
    auto goalState = app_options.goal<State>(); // 3150, 950

    const std::string outputName = "png_2d_demo_output.svg";
    std::ofstream file(outputName);
    shape::startSvg(file, width, height);
    shape::addBackgroundImg(file, app_options.env());
    for (auto& subspace : coord.getSubspaces()) {
        file << shape::Rect(
                subspace.getLower()[0],
                subspace.getLower()[1],
                subspace.getUpper()[0],
                subspace.getUpper()[1],
                shape::Color(0, 255, 0));
    }

    using Graph = typename Coordinator::Graph;
    auto graph = coord.getGraph();
    for (auto& [v_id, u_ids] : graph.getAdjacencyList()) {
        auto start = graph.getVertex(v_id);
        for (auto u_id : u_ids) {
            auto end = graph.getVertex(u_id);
            auto pos1 = v_id.find("_");
            auto pos2 = u_id.find("_");

            if (v_id.substr(0, pos1) == u_id.substr(0, pos2)) {
                // The 0th element indicates which lambda
                shape::addEdge(file, start.state()[0], start.state()[1], end.state()[0], end.state()[1],
                        6, shape::Color(40, 40, 40));
            } else {
                shape::addEdge(file, start.state()[0], start.state()[1], end.state()[0], end.state()[1],
                               6, shape::Color(250, 50, 50));
            }
        }
    }
    shape::endSvg(file);
}

int main(int argc, char *argv[]) {
    mpl::demo::AppOptions app_options(argc, argv);
    if (app_options.lambdaType(false).empty()) {
        app_options.lambdaType_ = "local";
    }
    if (app_options.communicator(false).empty()) {
        app_options.communicator_ = "rabbitmq";
    }
    if (app_options.coordinator(false).empty()) {
        app_options.coordinator_ = "localhost";
    }
    if (app_options.algorithm(false).empty()) {
        app_options.algorithm_ = "prm_fixed_graph";
    }

    using Scalar = double; // TODO: add single precision code
//    if (app_options.communicator() == "rabbitmq") {
//        using Comm = RabbitMQMessageQueue;
        if (app_options.scenario() == "png") {
            using Scenario = mpl::demo::PNG2dScenario<double>;

            if (app_options.algorithm() == "prm_fixed_graph") {
                using Coordinator = mpl::CoordinatorFixedGraph<Scenario, Scalar>;
                Coordinator coord(app_options);
                coord.start_socket();
                coord.divide_work();
                coord.init_lambdas();
                coord.loop();
                savePngImages<Coordinator, Scenario::State>(coord, app_options);
            }
        } else if (app_options.scenario() == "fetch") {
//            using Scenario = mpl::demo::FetchScenario<double>;
//
//            if (app_options.algorithm() == "prm_fixed_graph") {
//                using Coordinator = mpl::CoordinatorFixedGraph<Scenario, Scalar>;
//                Coordinator coord(app_options);
//                coord.start_socket();
//                coord.divide_work();
//                coord.init_lambdas();
//                coord.loop();
//            }
        }
        else {
            throw std::invalid_argument("Invalid scenario");
        }
//    }
//    else {
//        throw std::invalid_argument("Invalid comm");
//    }


    return 0;
}
