/*
 * Use this lambda for the strategy where each Lambda keeps only a certain part
 * of the graph and works on that while communicating it's new vertices to it's
 * neighbors.
 */
#pragma once
#ifndef MPL_LAMBDA_FIXED_GRAPH_HPP
#define MPL_LAMBDA_FIXED_GRAPH_HPP

#include <jilog.hpp>
#include <unordered_map>
#include <vector>
#include <prm_planner.hpp>
#include <subspace.hpp>
#include <util.hpp>
#include <chrono>
#include <demo/app_options.hpp>
#include <unordered_map>
#include <packet.hpp>
#include <demo/png_2d_scenario.hpp>
#include <comm.hpp>
#include <demo/fetch_scenario.hpp>
#include <demo/mpl_robot.hpp>


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
                //        const int max_incoming_vertices = 1000;
                Comm comm;
                Scenario scenario;
                Planner planner;
                Subspace_t local_subspace;
                Subspace_t global_subspace;
                std::unordered_map<int, std::vector<Vertex_t>> samples_to_send;
                std::vector<std::pair<Subspace_t, int>> neighborsToLambdaId;
                std::chrono::high_resolution_clock::time_point start_time;
                bool done_ = false;
                double time_limit = 100.0; // Set safety maximum limit so we don't get charged on AWS

                LocalLambdaFixedGraph();

                //static void trackValidSamples(Vertex_t& validSample, void* lambda) {
                //    auto localLambdaFixedGraph = static_cast<LocalLambdaFixedGraph *>(lambda);
                //    auto prmRadius = localLambdaFixedGraph->planner.getrPRM();
                //    for (auto [neighbor, lambda_id] : localLambdaFixedGraph->neighborsToLambdaId) {
                //        if (neighbor.is_within(prmRadius, validSample.state())) {
                //            localLambdaFixedGraph->samples_to_send[lambda_id].push_back(validSample);
                //        }
                //    }
                //}

            public:
                LocalLambdaFixedGraph(
                        AppOptions &app_options,
                        Scenario &scenario_,
                        Subspace_t &local_subspace_,
                        Subspace_t &global_subspace_,
                        std::vector<std::pair<Subspace_t, int>>& neighborsToLambdaIdGlobal
                        )
                    : lambda_id(app_options.lambdaId()),
                    scenario(scenario_),
                    local_subspace(local_subspace_),
                    global_subspace(global_subspace_),
                    planner(Planner(scenario_, app_options.lambdaId())),
                    samples_per_run(app_options.numSamples()) {
                        comm.setLambdaId(lambda_id);
                        comm.connect(app_options.coordinator());
                        comm.template process<Edge_t, Distance, Vertex_t, State>();

                        //// First record neighbors of this lambda from the local and global subspace
                        ////auto neighbors = local_subspace.get_neighbors(global_subspace);
                        JI_LOG(INFO) << "Local subspace " << local_subspace;
                        for (auto [neighbor, neighborLambdaId] : neighborsToLambdaIdGlobal) {
                            if (neighborLambdaId != lambda_id) {
                                neighborsToLambdaId.push_back(std::make_pair(neighbor, neighborLambdaId));
                            }
                        }
                        //neighborsToLambdaId = neighborsToLambdaIdGlobal;

                        //// Then add the callback to track these neighbors
                        //planner.addValidSampleCallback(trackValidSamples);
                        planner.addValidSampleCallback([&] (Vertex_t& validSample) -> void {
                                auto prmRadius = planner.getrPRM();
                                for (auto& [neighbor, lambda_id] : neighborsToLambdaId) {
                                if (neighbor.is_within(prmRadius, validSample.state())) {
                                samples_to_send[lambda_id].push_back(validSample);
                                }
                                }
                                });

                        //// Add start and goal samples. For now assume a single start position that you want to add, and 
                        //// the goals are sampled later by the coordinator.
                        //auto start = app_options.start<State>();
                        //JI_LOG(INFO) << "Testing start " << start;
                        //if (local_subspace.contains(start) && scenario.isValid(start)) {
                        //        planner.addSample(start, true);
                        //}

                        //auto s = std::chrono::high_resolution_clock::now();
                        //for (auto& [start, goal] : app_options.getStartsAndGoals<State>()) {
                        //    if (local_subspace.contains(start)) {
                        //        if (!scenario.isValid(start)) {
                        //            JI_LOG(ERROR) << "Start " << start << " is not valid";
                        //            continue;
                        //        }
                        //        planner.addSample(start, true);
                        //    }

                        //    JI_LOG(INFO) << "Testing goal " << goal;
                        //    if (local_subspace.contains(goal)) {
                        //        if (!scenario.isValid(goal)) {
                        //            JI_LOG(ERROR) << "Goal " << goal << " is not valid";
                        //            continue;
                        //        }
                        //        JI_LOG(INFO) << "Printing goal id";
                        //        planner.addSample(goal, true);
                        //    }
                        //}

                        //for (auto& goal: app_options.goals<State>()) {
                        //    if (local_subspace.contains(goal)) {
                        //        if (!scenario.isValid(goal)) {
                        //            JI_LOG(ERROR) << "Goal " << goal << " is not valid";
                        //            continue;
                        //        }
                        //        JI_LOG(INFO) << "Printing goal id";
                        //        planner.addSample(goal, true);
                        //    }
                        //}
                        //auto e = std::chrono::high_resolution_clock::now();
                        //auto start_goal_time = std::chrono::duration_cast<std::chrono::milliseconds>(e - s);
                        //JI_LOG(INFO) << "Time to check starts and goals " << start_goal_time;

                        //auto min_subspace_size = std::numeric_limits<Scalar>::max();
                        //for (int i=0; i < local_subspace.dimension(); ++i) {
                        //    auto subspace_size = local_subspace.getUpper()[i] - local_subspace.getLower()[i];
                        //    if (subspace_size < min_subspace_size) {
                        //        min_subspace_size = subspace_size;
                        //    }
                        //}
                        //handleGlobalNumSamplesUpdate(neighborsToLambdaIdGlobal.size() * samples_per_run); // If we imagine it as a large batching process, this assumption can hold
                        start_time = std::chrono::high_resolution_clock::now();
                }

                inline bool isDone() {
                    return comm.isDone() || done_;
                }


                void shutdown() {
                    comm.sendDone();
                    //comm.
                    //comm.template process<Edge_t, Distance, Vertex_t, State>();
                    JI_LOG(INFO) << "Sent done" ;
                }

                void do_work() {
                    auto start = std::chrono::high_resolution_clock::now();
                    auto lambda_running_for = std::chrono::duration_cast<std::chrono::seconds>(start - start_time);
                    if (lambda_running_for.count() > time_limit || comm.isDone()) {
                        done_ = true;
                        return;
                    }
                    planner.plan(samples_per_run);
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

                    // Even if we have no vertices to send, tell the coordinator we are done sampling
                    auto new_vertices = planner.getNewVertices();
                    JI_LOG(INFO) << "Sending " << new_vertices.size() << " new vertices";
                    comm.template sendVertices<Vertex_t, State>(std::move(new_vertices), 0, 0); // destination=0 means send to coordinator
                    planner.clearVertices();

                    JI_LOG(INFO) << "Lambda id " << lambda_id << ": time to sample " << samples_per_run << " points is " << duration;
                    auto new_edges = planner.getNewEdges();
                    auto edgeSize = packet::Edges<Edge_t, Distance>::edgeSize_;
                    auto edgeHeaderSize = packet::Edges<Edge_t, Distance>::edgeHeaderSize_;
                    auto maxPacketSize = mpl::packet::MAX_PACKET_SIZE;
                    if (new_edges.size() > 0) {
                        JI_LOG(INFO) << "Sending " << new_edges.size() << " new edges";
                        if (edgeSize * new_edges.size() + edgeHeaderSize < maxPacketSize) {
                            comm.template sendEdges<Edge_t, Distance>(std::move(new_edges));
                        } else {
                            auto freePacketSpace = maxPacketSize - edgeHeaderSize;
                            int numEdgesPerPacket = freePacketSpace / edgeSize;
                            for (int i=0; i < new_edges.size(); i+= numEdgesPerPacket) {
                                auto end_val = std::min( (int) new_edges.size(), i + numEdgesPerPacket);
                                std::vector<Edge_t> newEdgesPartial(new_edges.begin() + i, new_edges.begin() + end_val);
                                JI_LOG(INFO) << "Sending " << newEdgesPartial.size() << " partial new edges";
                                comm.template sendEdges<Edge_t, Distance>(std::move(newEdgesPartial));
                            }
                        }
                        planner.clearEdges();
                    }



                    for (auto &[lambdaId, vertices] : samples_to_send) {
                        if (vertices.size() > 0) {
                            JI_LOG(INFO) << "Sending " << vertices.size() << " to lambda " << lambdaId;
                            comm.template sendVertices<Vertex_t, State>(std::move(vertices), 1, lambdaId); // destination=1 means send to other lambda
                            vertices.clear();
                        }
                    }

                    comm.template process<Edge_t, Distance, Vertex_t, State>(
                            [&] (auto &&pkt) {
                            using T = std::decay_t<decltype(pkt)>;
                            if constexpr (packet::is_vertices<T>::value) {
                            handleIncomingVertices(std::move(pkt.vertices()));
                            } else if constexpr (packet::is_num_samples<T>::value) {
                            handleGlobalNumSamplesUpdate(pkt.num_samples());
                            }
                            });

                }

                void handleGlobalNumSamplesUpdate(std::uint64_t num_samples) {
                    JI_LOG(INFO) << "Updating num samples " << num_samples;
                    planner.updatePrmRadius(num_samples);
                }

                void handleIncomingVertices(const std::vector<Vertex_t>&& incoming_vertices) {
                    JI_LOG(INFO) << "Processing incoming vertices";
                    for (auto v: incoming_vertices) {
                        planner.connectVertex(v);
                    }
                }

                const Planner& getPlanner() const {
                    return planner;
                }

                const std::uint64_t& lambdaId() const {
                    return lambda_id;
                }

        };

    template <class Comm, class Scenario, class Planner, class Scalar>
        class LocalLambdaCommonSeed {
            public:
                using State = typename Scenario::State;
                using Bound = typename Scenario::Bound;
                using Distance = typename Scenario::Distance;
                using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;
                using Vertex_t = typename Planner::Vertex_t;
                using Edge_t = typename Planner::Edge_t;

            private:

                std::uint64_t total_samples_{0};
                std::uint64_t total_valid_samples_{0};
                std::uint64_t num_edges_connected_{0};
                std::uint64_t lambda_id;
                std::uint64_t num_lambdas;
                std::vector<State> randomSamples_;
                std::vector<Vertex_t> validSamples_;
                int samples_per_run;
                Comm comm;
                Scenario scenario;
                Planner planner;
                std::chrono::high_resolution_clock::time_point start_time;
                bool done_ = false;
                double time_limit = 100.0; // Set safety maximum limit so we don't get charged on AWS

                LocalLambdaCommonSeed();

            public:
                LocalLambdaCommonSeed(
                        AppOptions &app_options,
                        Scenario &scenario_
                        )
                    : lambda_id(app_options.lambdaId()),
                    scenario(scenario_),
                    planner(Planner(scenario_, 0)),
                    samples_per_run(app_options.numSamples()),
                    num_lambdas(app_options.jobs()) // TODO: make sure jobs is passed through to lambda
                    {
                        comm.setLambdaId(lambda_id);
                        comm.connect(app_options.coordinator());
                        comm.template process<Edge_t, Distance, Vertex_t, State>();
                        start_time = std::chrono::high_resolution_clock::now();

                        JI_LOG(INFO) << "Using seed: " << app_options.randomSeed();
                        JI_LOG(INFO) << "Num jobs: " << app_options.jobs();
                        planner.setSeed(app_options.randomSeed());
                        //for (int i=0; i < 10000000; ++i) {
                        //    JI_LOG(INFO) << "i " << i << " " << planner.generateRandomSample();
                        //}
                        //exit(0);
                    }

                inline bool isDone() {
                    return comm.isDone() || done_;
                }


                void shutdown() {
                    comm.sendDone();
                    JI_LOG(INFO) << "Sent done" ;
                }


                void generateRandomSamples() {
                    auto start = std::chrono::high_resolution_clock::now();
                    for (int i=0; i < samples_per_run; ++i) {
                        auto s = planner.generateRandomSample();
                        randomSamples_.push_back(s);
                        //JI_LOG(INFO) << "SAMPLE " << s << " NUMSAMPLES " << total_samples_;
                    }
                    total_samples_ += samples_per_run;
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                    JI_LOG(INFO) << "Lambda id " << lambda_id << ": time to generate " << samples_per_run << " random samples is " << duration;
                }
                
                void checkValidSamples() {
                    auto start = std::chrono::high_resolution_clock::now();
                    for (auto& s: randomSamples_) {
                        //JI_LOG(INFO) << "State for rng test " << s;
                        //if (total_valid_samples_ >= 24580) {
                        //    scenario.isValidPrint(s);
                        //    JI_LOG(INFO) << "Obstacle again " << scenario.isValid(s);
                        //    //JI_LOG(INFO) << "Obstacle planner again " << planner.validateSample(s);
                        //}
                        if (scenario.isValid(s)) {
                            auto v = Vertex_t{planner.generateVertexID(), s};
                            //JI_LOG(INFO) << "VERTEX " << s << " VALIDNUMSAMPLES " << total_valid_samples_;
                            validSamples_.push_back(v);
                            planner.addExistingVertex(v); // Keeping track of vertices outside the lambda, only use it for nn checks
                            ++total_valid_samples_;
                        }
                    }
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                    JI_LOG(INFO) << "Lambda id " << lambda_id << ": time to validate " << samples_per_run << " random samples is " << duration;
                }

                void connectSamples() {
                    auto start = std::chrono::high_resolution_clock::now();
                    for (auto& v : validSamples_) {
                        if (v.id().second % num_lambdas == lambda_id) {
                            //JI_LOG(INFO) << "Lambda " << lambda_id << " processing vertex " << v.id();
                            planner.connectVertex(v, [] (Edge_t& edge) {
                                    //if (edge.u().second > edge.v().second) JI_LOG(INFO) << "Processing edge " << edge.u() << "-" << edge.v();
                                    return edge.u().second > edge.v().second; // Connect 0-1, 0-2, 0-3, 1-2, 1-3, 2-3... lambda-0 is likely to start befor other lambdas so give it more work, have to validate this logic.
                                    });
                        }
                    }
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                    JI_LOG(INFO) << "Lambda id " << lambda_id << ": time to connect is " << duration;
                }

                void do_work() {
                    auto start = std::chrono::high_resolution_clock::now();
                    auto lambda_running_for = std::chrono::duration_cast<std::chrono::seconds>(start - start_time);
                    if (lambda_running_for.count() > time_limit || comm.isDone()) {
                        done_ = true;
                        return;
                    }
                    //if (total_valid_samples_ >= 20) {
                    //    done_ = true;
                    //    return;
                    //}
                    generateRandomSamples();
                    checkValidSamples();
                    planner.updatePrmRadius(total_samples_);
                    connectSamples();

                    JI_LOG(INFO) << "Sending " << validSamples_.size() << " new vertices";
                    comm.template sendVertices<Vertex_t, State>(std::move(validSamples_), 0, 0); // destination=0 means send to coordinator. TODO: everyone sends vertices to coordinator for now, this is to deal with inconsistent sampling. This can be made more efficient.
                    validSamples_.clear(); randomSamples_.clear();

                    auto new_edges = planner.getNewEdges();
                    num_edges_connected_ += new_edges.size();
                    JI_LOG(INFO) << "Total num edges connected " << num_edges_connected_;
                    auto edgeSize = packet::Edges<Edge_t, Distance>::edgeSize_;
                    auto edgeHeaderSize = packet::Edges<Edge_t, Distance>::edgeHeaderSize_;
                    auto maxPacketSize = mpl::packet::MAX_PACKET_SIZE;
                    if (new_edges.size() > 0) {
                        JI_LOG(INFO) << "Sending " << new_edges.size() << " new edges";
                        // Logic below is to partition in case of size exceeding the maximum packet size
                        if (edgeSize * new_edges.size() + edgeHeaderSize < maxPacketSize) {
                            comm.template sendEdges<Edge_t, Distance>(std::move(new_edges));
                        } else {
                            auto freePacketSpace = maxPacketSize - edgeHeaderSize;
                            int numEdgesPerPacket = freePacketSpace / edgeSize;
                            for (int i=0; i < new_edges.size(); i+= numEdgesPerPacket) {
                                auto end_val = std::min( (int) new_edges.size(), i + numEdgesPerPacket);
                                std::vector<Edge_t> newEdgesPartial(new_edges.begin() + i, new_edges.begin() + end_val);
                                JI_LOG(INFO) << "Sending " << newEdgesPartial.size() << " partial new edges";
                                comm.template sendEdges<Edge_t, Distance>(std::move(newEdgesPartial));
                            }
                        }
                        planner.clearEdges();
                    }
                    comm.template process<Edge_t, Distance, Vertex_t, State>(); // Only sends and no receives for this strategy
                }

                const Planner& getPlanner() const {
                    return planner;
                }

                const std::uint64_t& lambdaId() const {
                    return lambda_id;
                }
	};



    template <class Scenario, class Scalar>
        void runScenario(Scenario& scenario, AppOptions& app_options) {
            JI_LOG(INFO) << "Lambda ID" << app_options.lambdaId();

            using Planner = typename mpl::PRMPlanner<Scenario, Scalar>;
            using State = typename Scenario::State;
            using Bound = typename Scenario::Bound;
            if (app_options.algorithm() == "prm_fixed_graph") {
                using Lambda = typename mpl::demo::LocalLambdaFixedGraph<mpl::Comm, Scenario, Planner, Scalar>;
                using Subspace_t = typename Lambda::Subspace_t;


                // Construct local and global subspaces
                auto min = app_options.globalMin<Bound>();
                auto max = app_options.globalMax<Bound>();
                Subspace_t global_subspace(min, max);

                auto eig_num_divisions = app_options.num_divisions<State>();
                std::vector<int> num_divisions =
                    std::vector<int>(
                            eig_num_divisions.data(),
                            eig_num_divisions.data() + eig_num_divisions.rows() * eig_num_divisions.cols()
                            );

                std::vector<std::pair<Subspace_t, int>> subspaceToLambdaId;
                auto divisions = global_subspace.divide(num_divisions);
                for (int i=0; i < divisions.size(); ++i) {
                    subspaceToLambdaId.push_back(std::make_pair(divisions[i], i));
                }
                JI_LOG(INFO) << "Total number of lambdas " << subspaceToLambdaId.size();
                auto local_subspace = divisions[app_options.lambdaId()];

                JI_LOG(INFO) << "Lambda " << app_options.lambdaId() << " local subspace " << local_subspace;
                // End subspace construction

                scenario.setMin(local_subspace.getLower());
                scenario.setMax(local_subspace.getUpper());
                Lambda lambda(app_options, scenario, local_subspace, global_subspace, subspaceToLambdaId);
                runLambda(lambda);
            } else if (app_options.algorithm() == "prm_common_seed") {
                using Lambda = typename mpl::demo::LocalLambdaCommonSeed<mpl::Comm, Scenario, Planner, Scalar>;
                Lambda lambda(app_options, scenario);
                runLambda(lambda);
            }
        }

    template <class Lambda>
        void runLambda(Lambda& lambda) {
            for(;;) {
                lambda.do_work();
                if (lambda.isDone()) break;
            }
            lambda.shutdown();
            JI_LOG(INFO) << "Finished";

        }


    void runSelectPlanner(AppOptions& app_options) {
        using Scalar = double; // TODO: add single precision code

        if (app_options.coordinator(false).empty()) {
            app_options.coordinator_ = "localhost";
        }

        // set defaults outside
        if (app_options.scenario() == "png" || app_options.scenario() == "sequential_multi_agent_png") {
            using Scenario = PNG2dScenario<Scalar>;
            Scenario scenario = initPngScenario<Scalar>(app_options);
            runScenario<Scenario, Scalar>(scenario, app_options);
        } if (app_options.scenario() == "dubins_png") {
            using Scenario = DubinsPNG2dScenario<Scalar>;
            Scenario scenario = initDubinsPngScenario<Scalar>(app_options);
            runScenario<Scenario, Scalar>(scenario, app_options);
        } else if (app_options.scenario() == "fetch") {
            //using Scenario = FetchScenario<Scalar>;
            //Scenario scenario = initFetchScenario<Scalar>(app_options);
            //runScenario<Scenario, Scalar>(scenario, app_options);
        } else if (app_options.scenario() == "multi_agent_png") {
            using Scenario = MultiAgentPNG2DScenario<Scalar, NUM_AGENTS>;
            Scenario scenario = initMultiAgentPNG2DScenario<Scalar, NUM_AGENTS>(app_options);
            runScenario<Scenario, Scalar>(scenario, app_options);
        } else {
            throw std::invalid_argument("Invalid scenario");
        }

    }

}

#endif
