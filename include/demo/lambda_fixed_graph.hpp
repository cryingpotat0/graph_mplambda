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
    template <class Comm, class Scenario, class Planner, class Scalar>
    class LocalLambdaFixedGraph {
    public:
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;
        using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;

    private:

        int lambda_id;
        const int samples_per_run = 1000;
        const int max_incoming_vertices = 1000;
        Comm graph_comm;
        Comm vertex_comm;
        Scenario scenario;
        Planner planner;
        Subspace_t local_subspace;
        Subspace_t global_subspace;
        std::vector<State> validSamples;

        LocalLambdaFixedGraph();

        static void trackValidSamples(State validSample, void* lambda) {
            auto localLambdaFixedGraph = static_cast<LocalLambdaFixedGraph *>(lambda);
            localLambdaFixedGraph->validSamples.push_back(validSample);
        }

    public:
        LocalLambdaFixedGraph(
                int lambda_id_,
                Comm &graph_comm_,
                Comm &vertex_comm_,
                Scenario &scenario_,
                Subspace_t &local_subspace_,
                Subspace_t &global_subspace_
        )
                : lambda_id(lambda_id_),
                  graph_comm(graph_comm_),
                  vertex_comm(vertex_comm_),
                  scenario(scenario_),
                  local_subspace(local_subspace_),
                  global_subspace(global_subspace_),
                  planner(Planner(scenario_, this))
        {
            planner.addValidSampleCallback(trackValidSamples);
        }

        void do_work() {
          std::unordered_map<Subspace_t, std::vector<State>> samples_to_send = planner.plan(samples_per_run);
          for (auto& [subspace, states_to_send] : samples_to_send) {
            vertex_comm.set_destination(subspace);
            vertex_comm.put_all(states_to_send);
          }
//          Graph graph_diff = planner.get_graph_diff();
//          graph_comm.put(graph_diff); // send the graph back to the robot for djikstras and multi query, alternatively can try and do djikstras in a distributed fashion

          std::vector<State> new_states = vertex_comm[local_subspace].get(max_incoming_vertices);
          planner.add_states(new_states);
        }
    };

}

#endif
