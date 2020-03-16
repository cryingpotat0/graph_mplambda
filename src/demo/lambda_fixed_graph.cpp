#include <demo/lambda_fixed_graph.hpp>
#include <demo/png_2d_scenario.hpp>
#include <demo/shape_hierarchy.hpp>
#include <prm_planner.hpp>
#include <pq.hpp>
#include <vector>
#include <png.h>

using namespace mpl::demo;
int main(int argc, char *argv[]) {

  const std::string inputName = "./resources/png_planning_input.png";
  using Scenario = typename mpl::demo::PNG2dScenario<double>;
  using Comm = RabbitMQMessageQueue;
  using Planner = typename mpl::PRMPlanner<Scenario>;
  using Lambda = typename mpl::demo::LocalLambdaFixedGraph<Comm, Scenario, Planner>;
  using State = Scenario::State;

  std::vector<FilterColor> filters;
  filters.push_back(FilterColor(126, 106, 61, 15));
  filters.push_back(FilterColor(61, 53, 6, 15));
  filters.push_back(FilterColor(255, 255, 255, 5));

  auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, inputName);

  State startState, goalState;
  startState << 430, 1300;
  goalState << 3150, 950;

  std::cout << width << " " << height;

  //Scenario scenario(width, height, goalState, obstacles);
  
  //Lambda()
  //Scenario &scenario, 
  //int lambda_id, 
  //Comm &graph_comm, 
  //Comm &vertex_comm, 
  //Subspace_t &local_subspace, 
  //Subspace_t &global_subspace


  return 1;
}
