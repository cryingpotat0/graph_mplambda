#include <coordinator.hpp>
//#include <pq.hpp>
//#include <jilog.hpp>
//#include <getopt.h>
//#include <string>
//#include <demo/app_options.hpp>
#include <demo/png_2d_scenario.hpp>
#include <demo/shape_hierarchy.hpp>
#include <demo/mpl_robot.hpp>
#include <list>
#include <demo/fetch_scenario.hpp>
#include <chrono>
//#include <comm.hpp>
//#include <poll.h>
//#include <packet.hpp>
//#include <buffer.hpp>
//#include <write_queue.hpp>


using namespace mpl::demo;

template <class Coordinator>
void runCoordinator (Coordinator& coord, AppOptions& app_options) {
    coord.start_socket();
    coord.divide_work();
    if (app_options.loadGraph() != "") {
        JI_LOG(INFO) << "Loading graph from " << app_options.loadGraph();
        auto start = std::chrono::high_resolution_clock::now();
        coord.loadGraph(app_options.loadGraph());
        auto end = std::chrono::high_resolution_clock::now();
        JI_LOG(INFO) << "Time to load graph is " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    } else {
        coord.init_lambdas();
        coord.loop();
        auto start = std::chrono::high_resolution_clock::now();
        coord.saveGraph("graph-out.txt");
        auto end = std::chrono::high_resolution_clock::now();
        JI_LOG(INFO) << "Time to save graph is " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    }
}

int main(int argc, char *argv[]) {
    mpl::demo::AppOptions app_options(argc, argv);
    if (app_options.lambdaType(false).empty()) {
        app_options.lambdaType_ = "local";
    }
    //if (app_options.communicator(false).empty()) {
    //    app_options.communicator_ = "rabbitmq";
    //}
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
            runCoordinator(coord, app_options);
            pngPostProcessing<Coordinator, Scalar>(coord, app_options);
        } else if (app_options.algorithm() == "prm_common_seed") {
            using Coordinator = mpl::CoordinatorCommonSeed<Scenario, Scalar>;
            Coordinator coord(app_options);
            runCoordinator(coord, app_options);
            pngPostProcessing<Coordinator, Scalar>(coord, app_options);
        }
    } else if (app_options.scenario() == "fetch") {
        using Scenario = mpl::demo::FetchScenario<double>;
        if (app_options.algorithm() == "prm_fixed_graph") {
            using Coordinator = mpl::CoordinatorFixedGraph<Scenario, Scalar>;
            Coordinator coord(app_options);
            runCoordinator(coord, app_options);
            fetchPostProcessing<Coordinator, Scalar>(coord, app_options);
        } else if (app_options.algorithm() == "prm_common_seed") {
            using Coordinator = mpl::CoordinatorCommonSeed<Scenario, Scalar>;
            Coordinator coord(app_options);
            runCoordinator(coord, app_options);
            fetchPostProcessing<Coordinator, Scalar>(coord, app_options);
        }
    } else if (app_options.scenario() == "multi_agent_png") {
        using Scenario = mpl::demo::MultiAgentPNG2DScenario<double, NUM_AGENTS>; // Hardcode 2 agents for now
        if (app_options.algorithm() == "prm_fixed_graph") {
            using Coordinator = mpl::CoordinatorFixedGraph<Scenario, Scalar>;
            Coordinator coord(app_options);
            runCoordinator(coord, app_options);
            multiAgentPngPostProcessing<Coordinator, Scalar>(coord, app_options);
        } else if (app_options.algorithm() == "prm_common_seed") {
            using Coordinator = mpl::CoordinatorCommonSeed<Scenario, Scalar>;
            Coordinator coord(app_options);
            runCoordinator(coord, app_options);
            multiAgentPngPostProcessing<Coordinator, Scalar>(coord, app_options);
        }
    }
    else {
        throw std::invalid_argument("Invalid scenario");
    }

    return 0;
}
