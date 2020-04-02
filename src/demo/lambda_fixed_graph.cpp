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
//#include <demo/fetch_scenario.hpp>


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
    } else if (app_options.env() == "./resources/house_layout.png") {
        filters.emplace_back(FilterColor(0, 0, 0, 5));
        filters.emplace_back(FilterColor(224, 224, 224, 5));
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
//    auto local_min = app_options.min<Bound>();
//    auto local_max = app_options.max<Bound>();

//    Subspace_t local_subspace(local_min, local_max);
    Subspace_t global_subspace(min, max);

    auto eig_num_divisions = app_options.num_divisions<State>();
    std::vector<int> num_divisions =
            std::vector<int>(
                    eig_num_divisions.data(),
                    eig_num_divisions.data() + eig_num_divisions.rows() * eig_num_divisions.cols()
            );

    std::unordered_map<Subspace_t, int> neighborsToLambdaIdGlobal;
    auto divisions = global_subspace.divide(num_divisions);
    for (int i=0; i < divisions.size(); ++i) {
        neighborsToLambdaIdGlobal[divisions[i]] = i;
    }
    auto local_subspace = divisions[app_options.lambdaId()];

    JI_LOG(INFO) << "Lambda " << app_options.lambdaId() << " local subspace " << local_subspace;

    Scenario scenario(width, height, local_subspace.getLower(), local_subspace.getUpper(), goalState, obstacles);
    Lambda lambda(app_options, scenario, local_subspace, global_subspace, neighborsToLambdaIdGlobal);
    for(;;) {
        lambda.do_work();
        if (lambda.isDone()) break;
    }
    JI_LOG(INFO) << "Finished";

}


//template <class Scalar>
//void runFetchScenario(AppOptions &app_options) {
//    if (app_options.env(false).empty()) {
//    }
//
//    using Scenario = mpl::demo::FetchScenario<Scalar>;
//    using Planner = typename mpl::PRMPlanner<Scenario, Scalar>;
//    using Lambda = typename mpl::demo::LocalLambdaFixedGraph<mpl::Comm, Scenario, Planner, Scalar>;
//    using State = typename Scenario::State;
//    using Bound = typename Scenario::Bound;
//    using Subspace_t = typename Lambda::Subspace_t;
//
//    auto min = app_options.globalMin<Bound>();
//    auto max = app_options.globalMax<Bound>();
//    Subspace_t global_subspace(min, max);
//
//    auto eig_num_divisions = app_options.num_divisions<State>();
//    std::vector<int> num_divisions =
//            std::vector<int>(
//                    eig_num_divisions.data(),
//                    eig_num_divisions.data() + eig_num_divisions.rows() * eig_num_divisions.cols()
//            );
//
//    std::unordered_map<Subspace_t, int> neighborsToLambdaIdGlobal;
//    auto divisions = global_subspace.divide(num_divisions);
//    for (int i=0; i < divisions.size(); ++i) {
//        neighborsToLambdaIdGlobal[divisions[i]] = i;
//    }
//    auto local_subspace = divisions[app_options.lambdaId()];
//
//    JI_LOG(INFO) << "Lambda " << app_options.lambdaId() << " local subspace " << local_subspace;
//
//    auto startState = app_options.start<State>();
//    using State = typename Scenario::State;
//    using Frame = typename Scenario::Frame;
//    using GoalRadius = Eigen::Matrix<Scalar, 6, 1>;
//    Frame envFrame = app_options.envFrame<Frame>();
//    Frame goal = app_options.goal<Frame>();
//    GoalRadius goalRadius = app_options.goalRadius<GoalRadius>();
//
//    Scenario scenario(envFrame, app_options.env(), goal, goalRadius, app_options.checkResolution(0.1));
//    Lambda lambda(app_options, scenario, local_subspace, global_subspace, neighborsToLambdaIdGlobal);
//    for(;;) {
////    for(int i=0; i < 2; ++i) {
////        comm.poll();
//        lambda.do_work();
//        if (lambda.isDone()) break;
////        comm.flush();
//    }
//}


void runSelectPlanner(AppOptions& app_options) {
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
	} else if (app_options.scenario() == "fetch") {
		//        runFetchScenario<Scalar>(app_options);
	} else {
		throw std::invalid_argument("Invalid scenario");
	}

}
int main(int argc, char *argv[]) {
	JI_LOG(INFO) << argc << " " << argv;
	AppOptions app_options(argc, argv);
	runSelectPlanner(app_options);
	return 0;
}
