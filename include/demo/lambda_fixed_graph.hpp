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
#include <demo/se3_rigid_body_scenario.hpp>


namespace mpl::demo {
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
                std::uint64_t lambda_id_;
                std::uint64_t num_lambdas_;
                std::vector<State> randomSamples_;
                std::vector<Vertex_t> validSamples_;
                std::queue<std::pair<std::uint64_t, std::uint64_t>> work_queue_;
                int samples_per_run_;
                Comm comm_;
                Scenario scenario_;
                Planner planner_;
                std::chrono::high_resolution_clock::time_point start_time_;
                bool done_{false};
                double time_limit_{100.0}; // Set safety maximum limit so we don't get charged on AWS
                int graph_size_{std::numeric_limits<int>::infinity()};

                LocalLambdaCommonSeed();

            public:
                LocalLambdaCommonSeed(
                        AppOptions &app_options,
                        Scenario &scenario
                        )
                    : lambda_id_(app_options.lambdaId()),
                    scenario_(scenario),
                    planner_(Planner(scenario_, 0, false)),
                    samples_per_run_(app_options.numSamples()),
                    num_lambdas_(app_options.jobs()), // TODO: make sure jobs is passed through to lambda
                    graph_size_(app_options.graphSize()) {


                        comm_.setLambdaId(lambda_id_);

                        start_time_ = std::chrono::high_resolution_clock::now();
                        std::uint64_t time = std::chrono::time_point_cast<std::chrono::milliseconds>(start_time_).time_since_epoch().count();

                        JI_LOG(INFO) << "Delayed start time: " << app_options.delayedStartTime();
                        if (app_options.delayedStartTime() != 0) {
                            JI_LOG(INFO) << "Delaying" << app_options.delayedStartTime();
                            while (time < app_options.delayedStartTime()) {
                                start_time_ = std::chrono::high_resolution_clock::now();
                                time = std::chrono::time_point_cast<std::chrono::milliseconds>(start_time_).time_since_epoch().count();
                            }
                        }

                        JI_LOG(INFO) << "Sending start time: " << time;
                        comm_.setStartTime(time);

                        comm_.connect(app_options.coordinator());
                        comm_.template process<Edge_t, Distance, Vertex_t, State>();


                        JI_LOG(INFO) << "Using seed: " << app_options.randomSeed();
                        JI_LOG(INFO) << "Num jobs: " << app_options.jobs();
                        planner_.setSeed(app_options.randomSeed());
                        initWorkQueue(app_options);
                    }

                void initWorkQueue(AppOptions& app_options) {
                    auto val = app_options.FirstWorkPacket<std::pair<std::uint64_t, std::uint64_t>>();
                    JI_LOG(INFO) << val << " put into work_queue_";
                    work_queue_.push(val);

                    if (app_options.second_packet_ != "") {
                        auto val = app_options.SecondWorkPacket<std::pair<std::uint64_t, std::uint64_t>>();
                        JI_LOG(INFO) << val << " put into work_queue_";
                        work_queue_.push(val);
                    }
                    /* work_queue_.push(app_options.SecondWorkPacket()); */
                    /* auto tmp_queue = mpl::util::generateWorkQueueEqualWorkAmt(app_options.graphSize(), app_options.jobs(), app_options.numSamples()); */
                    /* for (int i=0; i < app_options.jobs(); ++i) { */
                    /*     if (tmp_queue.size() < 0) { */
                    /*         JI_LOG(ERROR) << "Queue too small"; */
                    /*         break; */
                    /*     } */
                    /*     auto val = tmp_queue.front(); */
                    /*     tmp_queue.pop(); */
                    /*     if (i == lambda_id_ || i == (lambda_id_ + num_lambdas_)) { */
                    /*         work_queue_.push(val); */
                    /*         JI_LOG(INFO) << val << " put into work_queue_"; */
                    /*     } */
                    /* } */
                }

                inline bool isDone() {
                    return comm_.isDone() || done_;
                }


                void shutdown() {
                    comm_.sendDone();
                    JI_LOG(INFO) << "Sent done" ;
                }


                /* void generateRandomSamples() { */
                /*     auto start = std::chrono::high_resolution_clock::now(); */
                    /* for (int i=0; i < samples_per_run_; ++i) { */
                /*         auto s = planner.generateRandomSample(); */
                /*         randomSamples_.push_back(s); */
                /*         //JI_LOG(INFO) << "SAMPLE " << s << " NUMSAMPLES " << total_samples_; */
                /*     } */
                /*     total_samples_ += samples_per_run_; */
                /*     auto stop = std::chrono::high_resolution_clock::now(); */
                /*     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); */
                /*     //JI_LOG(INFO) << "Lambda id " << lambda_id_ << ": time to generate " << samples_per_run_ << " random samples is " << duration; */
                /* } */

                /* void checkValidSamples() { */
                /*     auto start = std::chrono::high_resolution_clock::now(); */
                /*     for (auto& s: randomSamples_) { */
                /*         //JI_LOG(INFO) << "State for rng test " << s; */
                /*         //if (total_valid_samples_ >= 24580) { */
                /*         //    scenario_.isValidPrint(s); */
                /*         //    JI_LOG(INFO) << "Obstacle again " << scenario_.isValid(s); */
                /*         //    //JI_LOG(INFO) << "Obstacle planner_ again " << planner_.validateSample(s); */
                /*         //} */

                /*         auto v = Vertex_t{planner_.generateVertexID(), s}; */
                /*         if (scenario_.isValid(s)) { */
                /*             //JI_LOG(INFO) << "VERTEX " << s << " VALIDNUMSAMPLES " << total_valid_samples_; */
                /*             validSamples_.push_back(v); */
                /*             planner_.addExistingVertex(v); // Keeping track of vertices outside the lambda, only use it for nn checks */
                /*             ++total_valid_samples_; */
                /*         } */
                /*     } */
                /*     auto stop = std::chrono::high_resolution_clock::now(); */
                /*     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); */
                /*     //JI_LOG(INFO) << "Lambda id " << lambda_id_ << ": time to validate " << samples_per_run_ << " random samples is " << duration; */
                /* } */

                /* void connectSamples() { */
                /*     auto start = std::chrono::high_resolution_clock::now(); */
                /*     for (auto& v : validSamples_) { */
                /*         /1* if (v.id().second % num_lambdas_ == lambda_id_) { *1/ */
                /*         //JI_LOG(INFO) << "Lambda " << lambda_id_ << " processing vertex " << v.id(); */
                /*         planner_.connectVertex(v, [] (Edge_t& edge) { */
                /*                 //if (edge.u().second > edge.v().second) JI_LOG(INFO) << "Processing edge " << edge.u() << "-" << edge.v(); */
                /*                 return edge.u().second > edge.v().second; // Connect 0-1, 0-2, 0-3, 1-2, 1-3, 2-3... lambda-0 is likely to start befor other lambdas so give it more work, have to validate this logic. */
                /*                 }); */
                /*         /1* } *1/ */
                /*     } */
                /*     auto stop = std::chrono::high_resolution_clock::now(); */
                /*     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); */
                /*     //JI_LOG(INFO) << "Lambda id " << lambda_id_ << ": time to connect is " << duration; */
 
                void processWorkPacketModulo(std::pair<std::uint64_t, std::uint64_t> start_and_end_id) {
                    auto& [start_id, end_id] = start_and_end_id;

                    /* JI_LOG(INFO) << "Doing work " << start_id << " " << end_id */
                    /*     << " current location " << total_valid_samples_; */
                    // start_id indicates the start_num_of_valid_vertices
                    // end_id indicated the end_num_of_valid_vertices
                    while (total_valid_samples_ < start_id) {
                        /* JI_LOG(INFO) << "total_valid_samples_ " << */
                        /*     total_valid_samples_ << " start_id " << start_id; */
                        /* JI_LOG(INFO) << "total_valid_samples_ " << */
                        /*     total_valid_samples_ << " start_id " << start_id << " total_samples_ " << total_samples_; */
                        auto s = planner_.generateRandomSample();
                        auto v = Vertex_t{planner_.generateVertexID(), s};
                        /* JI_LOG(INFO) << "VERTEX " << v.id_; */
                        if (scenario_.isValid(s)) {
                            //JI_LOG(INFO) << "VERTEX " << s << " VALIDNUMSAMPLES " << total_valid_samples_;
                            validSamples_.push_back(v);
                            planner_.addExistingVertex(v); // Keeping track of vertices outside the lambda, only use it for nn checks
                            ++total_valid_samples_;
                        }
                        /* planner_.updatePrmRadius(total_samples_++); */
                        total_samples_++;
                        planner_.updateKPrm(total_valid_samples_);
                    }

                    while (total_valid_samples_ < end_id) {
                        /* JI_LOG(INFO) << "total_valid_samples_ " << */
                        /*     total_valid_samples_ << " end_id " << end_id << " total_samples_ " << total_samples_; */
                        auto s = planner_.generateRandomSample();
                        auto v = Vertex_t{planner_.generateVertexID(), s};
                        if (scenario_.isValid(s)) {
                            /* JI_LOG(INFO) << "VERTEX " << s << " VALIDNUMSAMPLES " << total_valid_samples_; */
                            ++total_valid_samples_;
                            validSamples_.push_back(v);
                            if (total_valid_samples_ % num_lambdas_ == lambda_id_) {
                                JI_LOG(INFO) << "VERTEX " << total_valid_samples_;
                                planner_.connectVertex(v);
                            }
                            planner_.addExistingVertex(v); // Keeping track of vertices outside the lambda, only use it for nn checks
                        }
                        /* planner_.updatePrmRadius(total_samples_++); */
                        total_samples_++;
                        planner_.updateKPrm(total_valid_samples_);
                    }
                    /* JI_LOG(INFO) << "Done work " << start_id << " " << end_id; */
                }               /* } */

                void processWorkPacket(std::pair<std::uint64_t, std::uint64_t> start_and_end_id) {
                    auto& [start_id, end_id] = start_and_end_id;

                    /* JI_LOG(INFO) << "Doing work " << start_id << " " << end_id */
                    /*     << " current location " << total_valid_samples_; */
                    // start_id indicates the start_num_of_valid_vertices
                    // end_id indicated the end_num_of_valid_vertices
                    while (total_valid_samples_ < start_id) {
                        /* JI_LOG(INFO) << "total_valid_samples_ " << */
                        /*     total_valid_samples_ << " start_id " << start_id; */
                        /* JI_LOG(INFO) << "total_valid_samples_ " << */
                        /*     total_valid_samples_ << " start_id " << start_id << " total_samples_ " << total_samples_; */
                        auto s = planner_.generateRandomSample();
                        auto v = Vertex_t{planner_.generateVertexID(), s};
                        /* JI_LOG(INFO) << "VERTEX " << v.id_; */
                        if (scenario_.isValid(s)) {
                            //JI_LOG(INFO) << "VERTEX " << s << " VALIDNUMSAMPLES " << total_valid_samples_;
                            validSamples_.push_back(v);
                            planner_.addExistingVertex(v); // Keeping track of vertices outside the lambda, only use it for nn checks
                            ++total_valid_samples_;
                        }
                        /* planner_.updatePrmRadius(total_samples_++); */
                        total_samples_++;
                        planner_.updateKPrm(total_valid_samples_);
                    }

                    while (total_valid_samples_ < end_id) {
                        /* JI_LOG(INFO) << "total_valid_samples_ " << */
                        /*     total_valid_samples_ << " end_id " << end_id << " total_samples_ " << total_samples_; */
                        auto s = planner_.generateRandomSample();
                        auto v = Vertex_t{planner_.generateVertexID(), s};
                        /* JI_LOG(INFO) << "VERTEX " << v.id_; */
                        if (scenario_.isValid(s)) {
                            //JI_LOG(INFO) << "VERTEX " << s << " VALIDNUMSAMPLES " << total_valid_samples_;
                            validSamples_.push_back(v);
                            planner_.connectVertex(v);
                            planner_.addExistingVertex(v); // Keeping track of vertices outside the lambda, only use it for nn checks
                            ++total_valid_samples_;
                        }
                        /* planner_.updatePrmRadius(total_samples_++); */
                        total_samples_++;
                        planner_.updateKPrm(total_valid_samples_);
                    }
                    /* JI_LOG(INFO) << "Done work " << start_id << " " << end_id; */
                }

                void do_work() {
                    auto start = std::chrono::high_resolution_clock::now();
                    auto lambda_running_for = std::chrono::duration_cast<std::chrono::seconds>(start - start_time_);
                    if (lambda_running_for.count() > time_limit_ ||
                            comm_.isDone() || 
                                total_valid_samples_ >= graph_size_) {
                        done_ = true;
                        return;
                    }
                    //if (total_valid_samples_ >= 20) {
                    //    done_ = true;
                    //    return;
                    //}
                    if (work_queue_.size() > 0) {
                        processWorkPacket(work_queue_.front());
                        //processWorkPacketModulo(work_queue_.front());
                        work_queue_.pop();
                        /* planner_.updatePrmRadius(total_samples_); */
                        /* connectSamples(); */
                        validSamples_.clear(); randomSamples_.clear();
                        comm_.template sendVertices<Vertex_t, State>(std::move(validSamples_), 0, 0); // destination=0 means send to coordinator. TODO: everyone sends vertices to coordinator for now, this is to deal with inconsistent sampling. This can be made more efficient.
                        /* JI_LOG(INFO) << "total_valid_samples_ " << total_valid_samples_; */
                    }
                    /* JI_LOG(INFO) << "Completed loop waiting for data"; */


                    auto new_edges = planner_.getNewEdges();
                    num_edges_connected_ += new_edges.size();
                    //JI_LOG(INFO) << "Total num edges connected " << num_edges_connected_;
                    //JI_LOG(INFO) << "Total samples generated " << total_samples_;
                    //JI_LOG(INFO) << "Total valid samples generated " << total_valid_samples_;
                    auto edgeSize = packet::Edges<Edge_t, Distance>::edgeSize_;
                    auto edgeHeaderSize = packet::Edges<Edge_t, Distance>::edgeHeaderSize_;
                    auto maxPacketSize = mpl::packet::MAX_PACKET_SIZE;
                    if (new_edges.size() > 0) {
                        JI_LOG(INFO) << "Sending " << new_edges.size() << " new edges";
                        // Logic below is to partition in case of size exceeding the maximum packet size
                        if (edgeSize * new_edges.size() + edgeHeaderSize < maxPacketSize) {
                            comm_.template sendEdges<Edge_t, Distance>(std::move(new_edges));
                        } else {
                            auto freePacketSpace = maxPacketSize - edgeHeaderSize;
                            int numEdgesPerPacket = freePacketSpace / edgeSize;
                            for (int i=0; i < new_edges.size(); i+= numEdgesPerPacket) {
                                auto end_val = std::min( (int) new_edges.size(), i + numEdgesPerPacket);
                                std::vector<Edge_t> newEdgesPartial(new_edges.begin() + i, new_edges.begin() + end_val);
                                //JI_LOG(INFO) << "Sending " << newEdgesPartial.size() << " partial new edges";
                                comm_.template sendEdges<Edge_t, Distance>(std::move(newEdgesPartial));
                            }
                        }
                        planner_.clearEdges();
                    }
                    comm_.template process<Edge_t, Distance, Vertex_t, State>(
                            [&] (auto &&pkt) {
                            using T = std::decay_t<decltype(pkt)>;
                            if constexpr (packet::is_random_seed_work<T>::value) {
                            work_queue_.emplace(std::pair(pkt.start_id(),
                                        pkt.end_id()));

                            } 
                            }
                            ); 
                }

                const Planner& getPlanner() const {
                    return planner_;
                }

                const std::uint64_t& lambdaId() const {
                    return lambda_id_;
                }
        };



    template <class Scenario, class Scalar>
        void runScenario(Scenario& scenario, AppOptions& app_options) {
            JI_LOG(INFO) << "Lambda ID" << app_options.lambdaId();

            using Planner = typename mpl::PRMPlanner<Scenario, Scalar>;
            using State = typename Scenario::State;
            using Bound = typename Scenario::Bound;
            if (app_options.algorithm() == "prm_fixed_graph") {
                //using Lambda = typename mpl::demo::LocalLambdaFixedGraph<mpl::Comm, Scenario, Planner, Scalar>;
                //using Subspace_t = typename Lambda::Subspace_t;


                //// Construct local and global subspaces
                //auto min = app_options.globalMin<Bound>();
                //auto max = app_options.globalMax<Bound>();
                //Subspace_t global_subspace(min, max);

                //auto eig_num_divisions = app_options.num_divisions<State>();
                //std::vector<int> num_divisions =
                //    std::vector<int>(
                //            eig_num_divisions.data(),
                //            eig_num_divisions.data() + eig_num_divisions.rows() * eig_num_divisions.cols()
                //            );

                //std::vector<std::pair<Subspace_t, int>> subspaceToLambdaId;
                //auto divisions = global_subspace.divide(num_divisions);
                //for (int i=0; i < divisions.size(); ++i) {
                //    subspaceToLambdaId.push_back(std::make_pair(divisions[i], i));
                //}
                //JI_LOG(INFO) << "Total number of lambdas " << subspaceToLambdaId.size();
                //auto local_subspace = divisions[app_options.lambdaId()];

                //JI_LOG(INFO) << "Lambda " << app_options.lambdaId() << " local subspace " << local_subspace;
                //// End subspace construction

                //scenario.setMin(local_subspace.getLower());
                //scenario.setMax(local_subspace.getUpper());
                //Lambda lambda(app_options, scenario, local_subspace, global_subspace, subspaceToLambdaId);
                //runLambda(lambda);
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
        } else if (app_options.scenario() == "fetch") {
            using Scenario = FetchScenario<Scalar>;
            Scenario scenario = initFetchScenario<Scalar>(app_options);
            runScenario<Scenario, Scalar>(scenario, app_options);
        } else if (app_options.scenario() == "multi_agent_png") {
            using Scenario = MultiAgentPNG2DScenario<Scalar, NUM_AGENTS>;
            Scenario scenario = initMultiAgentPNG2DScenario<Scalar, NUM_AGENTS>(app_options);
            runScenario<Scenario, Scalar>(scenario, app_options);
        } else if (app_options.scenario() == "se3") {
            using Scenario = SE3RigidBodyScenario<Scalar>;
            Scenario scenario = initSE3Scenario<Scalar>(app_options);
            runScenario<Scenario, Scalar>(scenario, app_options);
        }
        else {
            throw std::invalid_argument("Invalid scenario");
        }

    }

}

#endif
