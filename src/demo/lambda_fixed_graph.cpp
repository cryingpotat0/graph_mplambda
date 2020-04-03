#include <demo/lambda_fixed_graph.hpp>
#include <demo/app_options.hpp>
#include <jilog.hpp>


int main(int argc, char *argv[]) {
	JI_LOG(INFO) << argc << " " << argv;
	mpl::demo::AppOptions app_options(argc, argv);
	mpl::demo::runSelectPlanner(app_options);
	return 0;
}
