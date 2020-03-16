/*
 * Use this lambda for the strategy where each Lambda keeps only a certain part
 * of the graph and works on that while communicating it's new vertices to it's
 * neighbors.
 */
#pragma once
#ifndef MPL_LAMBDA_FIXED_GRAPH_HPP
#define MPL_LAMBDA_FIXED_GRAPH_HPP

#include <jilog.hpp>
#include <pq.hpp>
#include <unordered_map>
#include <vector>
#include <prm_planner.hpp>
#include <subspace.hpp>

namespace mpl::demo {
  template <class Comm, class Scenario, class Planner>
  class LocalLambdaFixedGraph {
    private:
      using State = typename Scenario::State;
      using Bound = typename Scenario::Bounds;
      using Scalar = typename Scenario::Scalar;
      using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;

      Comm vertex_comm;
      int lambda_id;
      const int samples_per_run = 1000;
      const int max_incoming_vertices = 1000;
      Comm graph_comm;
      Scenario scenario;
      Planner planner;
      Subspace_t local_subspace;
      Subspace_t global_subspace;

      LocalLambdaFixedGraph();

    public:
      LocalLambdaFixedGraph(
          int lambda_id, 
          Comm &graph_comm, 
          Comm &vertex_comm, 
          Scenario &scenario, 
          Subspace_t &local_subspace, 
          Subspace_t &global_subspace
          ) {
        this->lambda_id = lambda_id;
        this->graph_comm = graph_comm;
        this->vertex_comm = vertex_comm;
        this->scenario = scenario;
        this->planner = Planner(scenario, local_subspace, global_subspace);
        this->local_subspace = local_subspace;
        this->global_subspace = global_subspace;
      }

      /*
      do_work() {
        unordered_map<Subspace_t, std::vector<State>> samples_to_send = planner.plan(samples_per_run);
        for (auto& [subspace, states_to_send] : samples_to_send) {
          vertex_comm.set_destination(subspace);
          vertex_comm.put_all(states_to_send);
        }
        Graph graph_diff = planner.get_graph_diff();
        graph_comm.put(graph_diff); // send the graph back to the robot for djikstras and multi query, alternatively can try and do djikstras in a distributed fashion

        std::vector<State> new_states = vertex_comm[local_subspace].get(max_incoming_vertices);
        planner.add_states(new_states);
      }
      */
  };

}

#endif
