#include <coordinator.hpp>
//#include <pq.hpp>
//#include <jilog.hpp>
//#include <getopt.h>
//#include <string>
//#include <demo/app_options.hpp>
//#include <demo/png_2d_scenario.hpp>
//#include <comm.hpp>
//#include <poll.h>
//#include <packet.hpp>
//#include <buffer.hpp>
//#include <write_queue.hpp>


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
//                Comm comm(app_options.coordinator(), "graph");
                Coordinator coord(app_options);
                coord.start_socket();
                coord.divide_work();
//                coord.init_lambdas();
//                    comm.poll();
                    coord.loop();
//                    comm.flush();
//                  }
            }
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
