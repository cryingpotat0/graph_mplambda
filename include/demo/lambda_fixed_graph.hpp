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
#include <unordered_map>


namespace mpl::demo {
    template <class Comm, class Scenario, class Planner, class Scalar>
    class LocalLambdaFixedGraph {
    public:
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;
        using Distance = typename Scenario::Distance;
        using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;
        using Vertex_t = typename Planner::Vertex_t;
        using Edge_t = typename Planner::Edge_t;

    private:

        std::uint64_t lambda_id;
        int samples_per_run;
        float time_limit;
//        const int max_incoming_vertices = 1000;
        Comm comm;
        Scenario scenario;
        Planner planner;
        Subspace_t local_subspace;
        Subspace_t global_subspace;
        std::unordered_map<int, std::vector<Vertex_t>> samples_to_send;
        std::unordered_map<Subspace_t, int> neighborsToLambdaId;

        LocalLambdaFixedGraph();

        static void trackValidSamples(Vertex_t& validSample, void* lambda) {
            auto localLambdaFixedGraph = static_cast<LocalLambdaFixedGraph *>(lambda);
            auto prmRadius = localLambdaFixedGraph->planner.getrPRM();
            for (auto [neighbor, lambda_id] : localLambdaFixedGraph->neighborsToLambdaId) {
                if (neighbor.is_within(prmRadius, validSample.state())) {
                    localLambdaFixedGraph->samples_to_send[lambda_id].push_back(validSample);
                }
            }
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
            comm.template process<Edge_t, Distance, Vertex_t, State>();

            // First record neighbors of this lambda from the local and global subspace
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
            local_subspace = divisions[app_options.lambdaId()];
            auto neighbors = local_subspace.get_neighbors(global_subspace);
            JI_LOG(INFO) << "Local subspace " << local_subspace;
            for (auto& n : neighbors) {
                if (n != local_subspace) neighborsToLambdaId[n] = neighborsToLambdaIdGlobal[n];
            }

//            auto [tree_root, local_subspace_pointer] = global_subspace.layered_divide(num_divisions, &local_subspace);
//            Tree<Subspace_t> *curr_node_pointer = local_subspace_pointer;
//            for (int i=0; i < global_subspace.dimension() - 1; ++i) {
//                curr_node_pointer = curr_node_pointer->getParent();
//            }
//            // Now get the leaves of the curr_node_pointer
//            std::vector<Tree<Subspace_t>*> stack{curr_node_pointer};
//            while (!stack.empty()) {
//                auto next_node_pointer = stack.back();
//                if (next_node_pointer->isLeaf() && (next_node_pointer->getData() != local_subspace)) {
//                    neighborsToLambdaId[next_node_pointer->getData()] = neighborsToLambdaIdGlobal[next_node_pointer->getData()];
//                } else {
//                    const std::vector<Tree<Subspace_t>*>& children = next_node_pointer->getChildren();
//                    stack.insert(stack.end(), children.begin(), children.end());
//                }
//                stack.pop_back();
//            }
//
            for (auto& [sub, id] : neighborsToLambdaId) {
                JI_LOG(INFO) << "Neighboring subspace " << sub << " with id: " << id;
            }




            // Then add the callback to track these neighbors
            planner.addValidSampleCallback(trackValidSamples);

            // Add start and goal samples
            auto start_state = app_options.start<State>();
            planner.addSample(start_state);
            auto goal_state = app_options.goal<State>();
            planner.addSample(goal_state);
        }

        void do_work() {
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
            JI_LOG(INFO) << "Sending " << new_vertices.size() << " new vertices";
            comm.template sendVertices<Vertex_t, State>(std::move(new_vertices), 0, 0); // destination=0 means send to coordinator
            planner.clearVertices();
//            comm.put(new_vertices, "graph_vertices");
//            planner.clearVertices();
//
            auto new_edges = planner.getNewEdges();
            JI_LOG(INFO) << "Sending " << new_edges.size() << " new edges";
            comm.template sendEdges<Edge_t, Distance>(std::move(new_edges));
            planner.clearEdges();


            for (auto &[lambdaId, vertices] : samples_to_send) {
                comm.template sendVertices<Vertex_t, State>(std::move(vertices), 1, lambdaId); // destination=1 means send to other lambda
                JI_LOG(INFO) << "Sending " << vertices.size() << " to lambda " << lambdaId;
                vertices.clear();
            }
//            vertex_comm.set_destination(mpl::util::ToCString(local_subspace));
//            std::vector<State> new_states = vertex_comm.get(max_incoming_vertices);
//            planner.add_states(new_states);
            comm.template process<Edge_t, Distance, Vertex_t, State>();
        }

        const Planner& getPlanner() const {
            return planner;
        }

    };

}

#endif
