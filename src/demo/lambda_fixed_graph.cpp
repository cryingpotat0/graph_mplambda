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


int main(int argc, char *argv[]) {
	JI_LOG(INFO) << argc << " " << argv;
	mpl::demo::AppOptions app_options(argc, argv);
	mpl::demo::runSelectPlanner(app_options);
	return 0;
}
