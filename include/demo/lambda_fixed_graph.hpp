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
#include <util.hpp>
#include <chrono>
#include <demo/app_options.hpp>
#include <message.hpp>
#include <tree.hpp>


namespace mpl::demo {
    template <class Comm, class Scenario, class Planner, class Scalar>
    class LocalLambdaFixedGraph {
    public:
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;
        using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;
        using Vertex_t = Vertex<State>;

    private:

        std::uint64_t lambda_id;
        int samples_per_run;
        float time_limit;
//        const int max_incoming_vertices = 1000;
        Comm comm;
        Scenario scenario;
        Planner planner;
        Subspace_t const local_subspace;
        Subspace_t global_subspace;
        std::unordered_map<Subspace_t, std::vector<Vertex_t>> samples_to_send;

        LocalLambdaFixedGraph();

        static void trackValidSamples(Vertex_t validSample, void* lambda) {
            auto localLambdaFixedGraph = static_cast<LocalLambdaFixedGraph *>(lambda);
            localLambdaFixedGraph->samples_to_send[localLambdaFixedGraph->local_subspace].push_back(validSample);
        }

    public:
        LocalLambdaFixedGraph(
                AppOptions &app_options,
                Scenario &scenario_,
                Subspace_t &local_subspace_,
                Subspace_t &global_subspace_
        )
                : lambda_id(app_options.lambdaId()),
                  scenario(scenario_),
                  local_subspace(local_subspace_),
                  global_subspace(global_subspace_),
                  planner(Planner(scenario_, this, app_options.lambdaId())),
                  samples_per_run(app_options.numSamples()),
                  time_limit(app_options.timeLimit())

        {
            comm.setLambdaId(lambda_id);
            comm.connect(app_options.coordinator());
            comm.template process<Vertex_t, State>();
            // First record neighbors of this lambda from the local and global subspace

            // Then add the callback to track these neighbors
            planner.addValidSampleCallback(trackValidSamples);

            // Add start and goal samples
            auto start_state = app_options.start<State>();
            planner.addSample(start_state);
            auto goal_state = app_options.goal<State>();
            planner.addSample(goal_state);
        }

        void do_work() {
            comm.template process<Vertex_t, State>();
//          std::unordered_map<Subspace_t, std::vector<State>> samples_to_send = planner.plan(samples_per_run);
            auto start = std::chrono::high_resolution_clock::now();
            planner.plan(samples_per_run);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

            JI_LOG(INFO) << "Lambda id " << lambda_id << ": time to sample " << samples_per_run << " points is " << duration;

//            for (auto& [subspace, states_to_send] : samples_to_send) {
//                comm.set_destination(mpl::util::ToCString(subspace));
//                comm.put_all(states_to_send);
//            }
            auto new_vertices = planner.getNewVertices();
            comm.template sendVertices<Vertex_t, State>(std::move(new_vertices));
            planner.clearVertices();
//            comm.put(new_vertices, "graph_vertices");
//            planner.clearVertices();
//
//            auto new_edges = planner.getNewEdges();
//            comm.put(new_vertices, "graph_edges");
//            planner.clearEdges();


//            vertex_comm.set_destination(mpl::util::ToCString(local_subspace));
//            std::vector<State> new_states = vertex_comm.get(max_incoming_vertices);
//            planner.add_states(new_states);
        }

        const Planner& getPlanner() const {
            return planner;
        }

    };

}

#endif
