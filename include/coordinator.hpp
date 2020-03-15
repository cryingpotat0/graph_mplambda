#include "pq.hpp"
#include <string>
#include <vector>

namespace mpl {
  class Coordinator {
    public:
      void init_lambdas();
  };

  class LocalCoordinator : public Coordinator {
    bool first_division{true};
    int num_lambdas;
    GenericMessageQueue worker_queue;
    GenericMessageQueue graph_queue;
    std::string scenario_name;
    std::string scenario_args;

    public:
      LocalCoordinator(GenericMessageQueue worker_queue, GenericMessageQueue graph_queue, int num_lambdas, std::string scenario_name, std::string scenario_args);
      void init_lambdas();
      void divide_work();
      void merge_graph();

      template<class State>
      std::vector<State> find_path();

      template<class State>
      std::vector<State> solve();

  };
}


