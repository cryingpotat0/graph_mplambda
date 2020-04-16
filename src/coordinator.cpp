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
//#include <comm.hpp>
//#include <poll.h>
//#include <packet.hpp>
//#include <buffer.hpp>
//#include <write_queue.hpp>


using namespace mpl::demo;

template <class Coordinator>
void runCoordinator (Coordinator& coord) {
    coord.start_socket();
    coord.divide_work();
    coord.init_lambdas();
    coord.loop();
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
		runCoordinator(coord);
		pngPostProcessing<Coordinator, Scalar>(coord, app_options);
            }
        } else if (app_options.scenario() == "fetch") {
            using Scenario = mpl::demo::FetchScenario<double>;
            if (app_options.algorithm() == "prm_fixed_graph") {
                using Coordinator = mpl::CoordinatorFixedGraph<Scenario, Scalar>;
		Coordinator coord(app_options);
		runCoordinator(coord);
		//fetchPostProcessing<Coordinator, Scenario>(coord, app_options);
            }
        }
        else {
            throw std::invalid_argument("Invalid scenario");
        }

    return 0;
}
