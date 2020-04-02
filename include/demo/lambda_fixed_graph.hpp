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
#include <packet.hpp>


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
                Subspace_t &global_subspace_,
                std::unordered_map<Subspace_t, int>& neighborsToLambdaIdGlobal
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
            for (auto& [start, goal] : app_options.getStartsAndGoals<State>()) {
                JI_LOG(INFO) << "Testing start " << start;
                if (local_subspace.contains(start)) {
                    if (!scenario.isValid(start)) {
                        JI_LOG(ERROR) << "Start " << start << " is not valid";
                        continue;
                    }
                    planner.addSample(start, true);
                }

                JI_LOG(INFO) << "Testing start " << start;
                if (local_subspace.contains(goal)) {
                    if (!scenario.isValid(goal)) {
                        JI_LOG(ERROR) << "Goal " << goal << " is not valid";
                        continue;
                    }
                    JI_LOG(INFO) << "Printing goal id";
                    planner.addSample(goal, true);
                }
            }

            auto min_subspace_size = std::numeric_limits<Scalar>::max();
            for (int i=0; i < local_subspace.dimension(); ++i) {
                auto subspace_size = local_subspace.getUpper()[i] - local_subspace.getLower()[i];
                if (subspace_size < min_subspace_size) {
                    min_subspace_size = subspace_size;
                }
            }
            planner.setrPRM(min_subspace_size / 2.0); // TODO: arbitrary here, cannot be greater than min_subspace_size, but no other constraints
        }

        inline bool isDone() {
            return comm.isDone();
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
            if (new_vertices.size() > 0) {
                JI_LOG(INFO) << "Sending " << new_vertices.size() << " new vertices";
                comm.template sendVertices<Vertex_t, State>(std::move(new_vertices), 0, 0); // destination=0 means send to coordinator
                planner.clearVertices();
            }
//            comm.put(new_vertices, "graph_vertices");
//            planner.clearVertices();
//
            auto new_edges = planner.getNewEdges();
            if (new_edges.size() > 0) {
                JI_LOG(INFO) << "Sending " << new_edges.size() << " new edges";
                comm.template sendEdges<Edge_t, Distance>(std::move(new_edges));
                planner.clearEdges();
            }


            for (auto &[lambdaId, vertices] : samples_to_send) {
                if (vertices.size() > 0) {
                    JI_LOG(INFO) << "Sending " << vertices.size() << " to lambda " << lambdaId;
                    comm.template sendVertices<Vertex_t, State>(std::move(vertices), 1, lambdaId); // destination=1 means send to other lambda
                }
                vertices.clear();
            }
//            vertex_comm.set_destination(mpl::util::ToCString(local_subspace));
//            std::vector<State> new_states = vertex_comm.get(max_incoming_vertices);
//            planner.add_states(new_states);
            comm.template process<Edge_t, Distance, Vertex_t, State>(
                    [&] (auto &&pkt) {
                        using T = std::decay_t<decltype(pkt)>;
                        if constexpr (packet::is_vertices<T>::value) {
                            handleIncomingVertices(std::move(pkt.vertices()));
                        } else if constexpr (packet::is_num_samples<T>::value) {
                            handleGlobalNumSamplesUpdate(pkt.num_samples());
                        }
                    });

            // Repeat this step to send interconnections immediately
            new_edges = planner.getNewEdges();
            if (new_edges.size() > 0) {
                JI_LOG(INFO) << "Sending " << new_edges.size() << " new edges";
                comm.template sendEdges<Edge_t, Distance>(std::move(new_edges));
                planner.clearEdges();
            }
        }

        void handleGlobalNumSamplesUpdate(std::uint64_t num_samples) {
            JI_LOG(INFO) << "Updating num samples " << num_samples;
            planner.updatePrmRadius(num_samples, local_subspace.dimension());
        }

        void handleIncomingVertices(const std::vector<Vertex_t>&& incoming_vertices) {
            JI_LOG(INFO) << "Processing incoming vertices";
            for (auto v: incoming_vertices) {
                planner.addExistingVertex(v);
            }
        }

        const Planner& getPlanner() const {
            return planner;
        }

    };

}

#endif
