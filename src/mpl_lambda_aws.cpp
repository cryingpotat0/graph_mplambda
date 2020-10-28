// #include <mpl/demo/lambda_common.hpp>
// main.cpp
#include <demo/lambda_fixed_graph.hpp>
#include <aws/lambda-runtime/runtime.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <iostream>
#include <memory>
#include <string>
#include <iostream>

using namespace aws::lambda_runtime;

static void assign_value(std::string& value, const std::string& s) { value = s; }
static void assign_value(double& value, const std::string& s) { value = std::stod(s); }
static void assign_value(std::uint64_t& value, const std::string& s) { value = std::stoull(s); }

template <class T, class J>
static void set(T& value, const J& json, const std::string& name) {
    Aws::String name_aws(name.c_str(), name.size());
    if (json.ValueExists(name_aws)) {
        Aws::String aws_s = json.GetString(name_aws);
        std::string s(aws_s.c_str(), aws_s.size());
        assign_value(value, s);
    }
}

template <class J>
static std::vector<std::string> createVectorOfStrings(const J& json, const std::string& name, std::string delimiter=";") {
    Aws::String name_aws(name.c_str(), name.size());
    std::vector<std::string> ret;
    if (json.ValueExists(name_aws)) {
        Aws::String aws_s = json.GetString(name_aws);
        std::string s(aws_s.c_str(), aws_s.size());
	size_t pos = 0; //s.find(delimiter);
	while ((pos = s.find(delimiter)) != std::string::npos) {
		ret.push_back(s.substr(0, pos));
		s = s.substr(pos + 1);
	}
	if (s.size() > 0) {
		ret.push_back(s);
	}
    }
    return ret;
}

invocation_response my_handler(invocation_request const& request) try {
    using namespace Aws::Utils::Json;
    Aws::String payload(request.payload.c_str(), request.payload.size());
    JsonValue json(payload);
    
    if (!json.WasParseSuccessful()) {
        return invocation_response::failure("Failed to parse input JSON", "InvalidJSON");
    }

    auto v = json.View();

    mpl::demo::AppOptions options;
    set(options.scenario_, v, "scenario");
    std::cerr << options.scenario_<< "here1 \n" ;
    set(options.coordinator_, v, "coordinator");
    std::cerr << options.coordinator_<< "here2 \n" ;
    set(options.global_min_, v, "global_min");
    std::cerr << options.global_min_<< "here3 \n" ;
    set(options.global_max_, v, "global_max");
    std::cerr << options.global_max_<< "here4 \n" ;
    set(options.algorithm_, v, "algorithm");
    std::cerr << options.algorithm_<< "here5 \n" ;
    set(options.env_, v, "env");
    std::cerr << options.env_<< "here6 \n" ;
    set(options.robot_, v, "robot");
    std::cerr << options.robot_<< "here7 \n" ;
    set(options.envFrame_, v, "env-frame");
    std::cerr << options.envFrame_<< "here8 \n" ;
    set(options.goalRadius_, v, "goal-radius");
    std::cerr << options.goalRadius_<< "here9 \n" ;
    set(options.timeLimit_, v, "time-limit");
    std::cerr << options.timeLimit_<< "here10 \n" ;
    set(options.checkResolution_, v, "check-resolution");
    std::cerr << options.checkResolution_<< "here11 \n" ;
    set(options.lambdaId_, v, "lambda_id");
    std::cerr << options.lambdaId_<< "here12 \n" ;
    set(options.num_divisions_, v, "num_divisions");
    std::cerr << options.num_divisions_<< "here13 \n" ;
    set(options.randomSeed_, v, "random_seed");
    std::cerr << options.randomSeed_<< "here14 \n" ;
    set(options.jobs_, v, "jobs");
    std::cerr << options.jobs_<< "here15 \n" ;
    set(options.num_samples_, v, "num_samples");
    std::cerr << options.num_samples_<< "here16 \n" ;
    set(options.graphSize_, v, "graph-size");
    std::cerr << options.graphSize_<< "here17 \n" ;
    set(options.first_packet_, v, "first_packet");
    std::cerr << options.first_packet_<< "here18 \n" ;
    set(options.second_packet_, v, "second_packet");
    std::cerr << options.second_packet_<< "here19 \n" ;
    set(options.delayed_start_time_, v, "delayed_start_time");
    options.starts_ = createVectorOfStrings(v, "start");
    options.goals_ = createVectorOfStrings(v, "goal");
    options.start_ = options.starts_[0];
    options.goal_ = options.goals_[0];
    std::cerr << options.starts_ << "here20 " << options.start_ << " \n" ;
    std::cerr << options.goals_ << "here21 " << options.goal_ << " \n" ;

    mpl::demo::runSelectPlanner(options);
    return invocation_response::success("Solved!", "application/json");
} catch (const std::invalid_argument& ex) {
    std::cerr << "Invalid argument: " << ex.what() << std::endl;
    return invocation_response::failure(ex.what(), "InvalidArgument");
} catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return invocation_response::failure(ex.what(), "Exception");
}

int main()
{
    run_handler(my_handler);
    return 0;
}

