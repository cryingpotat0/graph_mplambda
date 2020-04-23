
#ifndef MPL_COORDINATOR_HPP
#define MPL_COORDINATOR_HPP

#include <string>
#include <vector>
#include <demo/app_options.hpp>
#include <jilog.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <system_error>
#include <sys/errno.h>
#include <subspace.hpp>
#include <graph.hpp>
#include <prm_planner.hpp>
#include <connection.hpp>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <syserr.hpp>
#include <signal.h>
#include <list>
#include <util.hpp>

#if HAS_AWS_SDK
#include <aws/lambda-runtime/runtime.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/Outcome.h>
#include <aws/lambda/LambdaClient.h>
#include <aws/lambda/model/InvokeRequest.h>
#include <aws/core/utils/json/JsonSerializer.h>
#endif


namespace mpl {
    struct LambdaType {
        constexpr static auto LAMBDA_PSEUDO = "local";
        constexpr static auto LAMBDA_AWS = "aws";
    };

    template <class Scenario, class Scalar>
        class CoordinatorFixedGraph {
            public:
                using State = typename Scenario::State;
                using Bound = typename Scenario::Bound;
                using Distance = typename Scenario::Distance;
                using Subspace_t = Subspace<Bound, State, Scalar>;
                using Planner = mpl::PRMPlanner<Scenario, Scalar>;
                using Vertex = mpl::Vertex<State>;
                using Vertex_ID = typename Vertex::ID;
                using Edge = mpl::Edge<typename Vertex::ID, Distance>;
                //using Graph = UndirectedGraph<Vertex, Edge>;

                using TimedVertex = mpl::TimedVertex<State>;
                using TimedEdge = mpl::TimedEdge<typename Vertex::ID, Distance>;
                using TimedGraph = UndirectedGraph<TimedVertex, TimedEdge>;
                using Connection_t = Connection<CoordinatorFixedGraph>;

                std::vector<std::vector<Buffer>> buffered_data_; // Any data to be written if lambda is not up yet
                //std::unordered_map<std::pair<Vertex_ID, Vertex_ID>,
                //        std::pair<bool, std::vector<Vertex_ID>>> pathFromGoalToStarts;
                demo::AppOptions app_options;
                std::chrono::high_resolution_clock::time_point start_time;

            private:
                std::vector<std::uint64_t> num_samples_per_lambda_;
                std::uint64_t global_min_samples{0};
                std::uint64_t global_num_uniform_samples_{0};
                std::vector<std::pair<std::uint64_t, std::uint64_t>> timed_global_num_uniform_samples_;
                Subspace_t global_subspace;
                std::vector<Subspace_t> lambda_subspaces;
                int listen_{-1};
                int port_{0x415E};
                std::vector<struct pollfd> pfds;
                std::list<Connection_t> connections_;
                std::list<std::pair<int, int>> childProcesses_;
                std::vector<Connection_t*> lambdaId_to_connection_;
                TimedGraph graph;
#if HAS_AWS_SDK
                static constexpr const char* ALLOCATION_TAG = "mplLambdaAWS";
                std::shared_ptr<Aws::Lambda::LambdaClient> lambdaClient_;
#endif

                void init_aws_lambdas() {
#if !HAS_AWS_SDK
                    throw std::invalid_argument("AWS SDK is not available");
#else
                    for (int i=0; i < app_options.jobs() ; ++i) {
                        Aws::Lambda::Model::InvokeRequest invokeRequest;
                        invokeRequest.SetFunctionName("mpl_fixed_graph_aws");
                        invokeRequest.SetInvocationType(Aws::Lambda::Model::InvocationType::Event);
                        std::shared_ptr<Aws::IOStream> payload = Aws::MakeShared<Aws::StringStream>("PayloadData");
                        Aws::Utils::Json::JsonValue jsonPayload;

                        //jsonPayload.WithString("algorithm", prob.algorithm() == 'r' ? "rrt" : "cforest");

                        auto lambdaId = std::to_string(i);
                        auto lambda_min = lambda_subspaces[i].getLower();
                        auto lambda_max = lambda_subspaces[i].getUpper();
                        std::vector<std::string> args = {
                            "scenario", app_options.scenario(),
                            "global_min", app_options.global_min_,
                            "global_max", app_options.global_max_,
                            "lambda_id", lambdaId,
                            "algorithm", app_options.algorithm(),
                            "coordinator", app_options.coordinator(),
                            //"communicator", app_options.communicator(),
                            "num_samples", std::to_string(app_options.numSamples()),
                            "time-limit", std::to_string(app_options.timeLimit()),
                            "env", app_options.env(false),
                            "env-frame", app_options.envFrame_,
                            "num_divisions", app_options.num_divisions_
                        };
                        args.push_back("start");
                        std::string starts;
                        for (int i=0; i < app_options.starts_.size(); ++i) {
                            starts += app_options.starts_[i];
                            if (i != app_options.starts_.size() - 1) starts += ";";
                        }
                        args.push_back(starts);

                        args.push_back("goal");
                        std::string goals;
                        for (int i=0; i < app_options.goals_.size(); ++i) {
                            goals += app_options.goals_[i];
                            if (i != app_options.goals_.size() - 1) goals += ";";
                        }
                        args.push_back(goals);

                        for (std::size_t i=0 ; i+1<args.size() ; i+=2) {
                            const std::string& key = args[i];
                            const std::string& val = args[i+1];
                            jsonPayload.WithString(
                                    Aws::String(key.c_str(), key.size()),
                                    Aws::String(val.c_str(), val.size()));
                        }

                        *payload << jsonPayload.View().WriteReadable();
                        invokeRequest.SetBody(payload);
                        invokeRequest.SetContentType("application/json");

                        auto outcome = lambdaClient_->Invoke(invokeRequest);
                        if (outcome.IsSuccess()) {
                            auto &result = outcome.GetResult();
                            Aws::IOStream& payload = result.GetPayload();
                            Aws::String functionResult;
                            std::getline(payload, functionResult);
                            JI_LOG(INFO) << "Lambda result: " << functionResult;
                        } else {
                            auto &error = outcome.GetError();
                            std::ostringstream msg;
                            msg << "name: '" << error.GetExceptionName() << "', message: '" << error.GetMessage() << "'";
                            throw std::runtime_error(msg.str());

                        }
                    }
#endif

                }

                void init_local_lambdas() {
                    for (int i=0; i < app_options.jobs() ; ++i) {
                        int p[2];
                        if (::pipe(p) == -1)
                            throw std::system_error(errno, std::system_category(), "Pipe");
                        if (int pid = ::fork()) {
                            // parent process
                            ::close(p[1]);
                            continue;
                        }
                        ::close(p[0]);

                        std::string program = "./mpl_lambda_fixed_graph";
                        auto lambdaId = std::to_string(i);
                        auto lambda_min = lambda_subspaces[i].getLower();
                        auto lambda_max = lambda_subspaces[i].getUpper();
                        std::vector<std::string> args = {
                            "--scenario", app_options.scenario(),
                            "--global_min", app_options.global_min_,
                            "--global_max", app_options.global_max_,
                            "--lambda_id", lambdaId,
                            "--algorithm", app_options.algorithm(),
                            "--coordinator", app_options.coordinator(),
                            //"--communicator", app_options.communicator(),
                            "--num_samples", std::to_string(app_options.numSamples()),
                            "--time-limit", std::to_string(app_options.timeLimit()),
                            "--env", app_options.env(false),
                            "--env-frame", app_options.envFrame_,
                            "--num_divisions", app_options.num_divisions_,
                        };
                        for (int i=0; i < app_options.starts_.size(); ++i) {
                            args.push_back("--start");
                            args.push_back(app_options.starts_[i]);
                        }
                        for (int i=0; i < app_options.goals_.size(); ++i) {
                            args.push_back("--goal");
                            args.push_back(app_options.goals_[i]);
                        }


                        std::vector <const char *> argv;

                        char file[20];
                        snprintf(file, sizeof(file), "lambda-%d.out", i);

                        argv.push_back(program.c_str());
                        std::string command = program;

                        for (auto ind=0; ind < args.size(); ++ind) {
                            argv.push_back(args[ind].c_str());
                            command += " " + args[ind];
                        }
                        argv.push_back(nullptr); // <-- required terminator
                        JI_LOG(TRACE) << "Running lambda" << lambdaId << " file " << file << ": " << command;

                        int fd = ::open(file, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
                        dup2(fd, 1); // make stdout write to file
                        dup2(fd, 2); // make stderr write to file
                        close(fd); // close fd, dups remain open
                        execv(argv[0], const_cast<char*const*>(argv.data()));
                        throw std::system_error(errno, std::system_category(), "Lambda died");
                    }

                }
            public:
                CoordinatorFixedGraph (demo::AppOptions& app_options_) :
                    app_options(app_options_),
                    global_subspace(Subspace_t(app_options_.globalMin<Bound>(), app_options_.globalMax<Bound>()))
            {
#if HAS_AWS_SDK
                if (app_options.lambdaType() == LambdaType::LAMBDA_AWS) {
                    JI_LOG(INFO) << "initializing lambda client";
                    Aws::SDKOptions options;
                    Aws::InitAPI(options);
                    Aws::Client::ClientConfiguration clientConfig;
                    clientConfig.region = "us-west-2";
                    lambdaClient_ = Aws::MakeShared<Aws::Lambda::LambdaClient>(ALLOCATION_TAG, clientConfig);
                }
#endif
            }

                ~CoordinatorFixedGraph() {
                    if (listen_ != -1 && !::close(listen_))
                        JI_LOG(WARN) << "failed to close listening socket";

#if HAS_AWS_SDK
                    if (app_options.lambdaType() == LambdaType::LAMBDA_AWS) {
                        Aws::SDKOptions options;
                        Aws::ShutdownAPI(options);
                    }
#endif
                }

                inline std::string algorithm() {
                    return "prm_fixed_graph";
                }

                void addVertices(const std::vector<Vertex>& vertices) {
                    auto stop = std::chrono::high_resolution_clock::now();
                    std::uint64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time).count();
                    for (auto& v : vertices) {
                        //JI_LOG(INFO) << "Adding " << v.id() << " at " << curr_time;
                        graph.addVertex(TimedVertex{v.id(), v.state(), curr_time});
                    }
                }

                void addEdges(const std::vector<Edge>& edges) {
                    auto stop = std::chrono::high_resolution_clock::now();
                    std::uint64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time).count();
                    for (auto& e : edges) {
                        //JI_LOG(INFO) << "Adding " << e.u() << "-" << e.v() << " at " << curr_time;
                        graph.addEdge(TimedEdge{e.distance(), e.u(), e.v(), curr_time});
                    }
                }

                void update_num_samples(std::uint64_t lambdaId, size_t successful_samples) {
                    num_samples_per_lambda_[lambdaId] += app_options.numSamples(); // Regardless of how many successful samples, each lambda tries to sample these many points
                    auto curr_min_samples = std::min_element(
                            num_samples_per_lambda_.begin(),
                            num_samples_per_lambda_.end()
                            );
                    if ((*curr_min_samples) > global_min_samples) {
                        global_min_samples = *curr_min_samples;
                        global_num_uniform_samples_ = global_min_samples * app_options.jobs();
                        JI_LOG(INFO) << "Updating global_num_samples to " << global_num_uniform_samples_;
                        auto stop = std::chrono::high_resolution_clock::now();
                        std::uint64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time).count();
                        timed_global_num_uniform_samples_.push_back(std::make_pair(curr_time, global_num_uniform_samples_));
                        for (auto& c: connections_) {
                            c.write(packet::NumSamples(global_num_uniform_samples_));
                        }
                    } else {
                        JI_LOG(INFO) << "Not updating global_num_samples " << global_min_samples << " from curr_min " << *curr_min_samples;

                    }
                    JI_LOG(INFO) << num_samples_per_lambda_;
                }


                const TimedGraph& getGraph() const {
                    return graph;
                };

                TimedGraph& getGraph() {
                    return graph;
                };
                
                //Graph getGraph(std::uint64_t time_limit) {
                //    // Filter all the edges and vertices < time_limit in milliseconds
                //    Graph return_graph;
                //    auto adjacency_list = graph.getAdjacencyList();
                //    for (auto &[curr_id, others] : adjacency_list) {
                //        auto v = graph.getVertex(curr_id);
                //        if (v.timestamp_millis() > time_limit) continue;
                //        return_graph.addVertex(Vertex{v.id(), v.state()});
                //        for (auto& u : others) {
                //            auto& curr_edge = graph.getEdge(curr_id, u);
                //            if (curr_edge.timestamp_millis() < v.timestamp_millis()) {
                //                JI_LOG(ERROR) << "BIG ERROR, VERTICES SHOULD BE ADDED BEFORE EDGES";
                //            }
                //            if (curr_edge.timestamp_millis() > time_limit) continue;
                //            return_graph.addEdge(Edge{curr_edge.distance(), curr_edge.u(), curr_edge.v()});
                //        }
                //    }
                //    return return_graph;
                //}

                const std::vector<Subspace_t> getSubspaces() const {
                    return lambda_subspaces;
                }


                void start_socket() {
                    if ((listen_ = ::socket(PF_INET, SOCK_STREAM, 0)) == -1)
                        throw syserr("socket()");

                    int on = 1;
                    if (::setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1)
                        throw syserr("set reuse addr");
                    // TODO: replace below code with MSG_NOSIGNAL in writev later
                    signal(SIGPIPE, SIG_IGN);


                    struct sockaddr_in addr;
                    addr.sin_family = AF_INET;
                    addr.sin_addr.s_addr = htonl(INADDR_ANY);
                    addr.sin_port = htons(port_);

                    if (::bind(listen_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1)
                        throw syserr("bind()");

                    socklen_t addrLen = sizeof(addr);
                    if (::getsockname(listen_, reinterpret_cast<struct sockaddr*>(&addr), &addrLen) == -1)
                        throw syserr("getsockname()");

                    JI_LOG(INFO) << "listening on port: " << ntohs(addr.sin_port);

                    if (::listen(listen_, 10000) == -1)
                        throw syserr("listen()");
                }

                void divide_work() {
                    auto eig_num_divisions = app_options.num_divisions<State>();
                    std::vector<int> num_divisions = std::vector<int>(
                            eig_num_divisions.data(),
                            eig_num_divisions.data() + eig_num_divisions.rows() * eig_num_divisions.cols()
                            );
                    assert(num_divisions.size() == global_subspace.dimension());
                    lambda_subspaces = global_subspace.divide(num_divisions);
                    app_options.jobs_ = lambda_subspaces.size();

                    JI_LOG(INFO) << "Num lambdas: " << app_options.jobs();
                    JI_LOG(INFO) << "Global Space: " << global_subspace;
                    JI_LOG(INFO) << "Subspaces: " << lambda_subspaces;
                }



                void loop() {
                    // Do communication stuff
                    int done_ = 0;
                    JI_LOG(INFO) << "loop started";

                    start_time = std::chrono::high_resolution_clock::now(); // class variable
                    for(;;) {


                        pfds.clear();
                        // first comes the fds of the child processes (note that
                        // connection processing may change the child process list, so
                        // this must be processed first)
                        for (auto[pid, fd] : childProcesses_) {
                            pfds.emplace_back();
                            pfds.back().fd = fd;
                            pfds.back().events = POLLIN;
                        }

                        // then et of pollfds is 1:1 with connections
                        for (Connection_t &conn : connections_)
                            pfds.emplace_back(conn);

                        // then comes the accepting socket
                        pfds.emplace_back();
                        pfds.back().fd = listen_;
                        pfds.back().events = POLLIN;

                        int nReady = ::poll(pfds.data(), pfds.size(), 1);
                        auto stop = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                        if (duration.count() > app_options.timeLimit() * 1000) { // specified time limit in s
                            if (!done_) done_ = 1;
                        }

                        if (nReady == -1) {
                            if (errno == EAGAIN || errno == EINTR) {
                                continue;
                            } else {
                                throw syserr("poll");
                            }

                        }

                        auto pit = pfds.begin();
                        for (auto cit = childProcesses_.begin(); cit != childProcesses_.end(); ++pit) {
                            assert(pit != pfds.end());

                            if ((pit->revents & POLLHUP) == 0) {
                                ++cit;
                            } else {
                                int stat = 0;
                                if (::close(cit->second) == -1)
                                    JI_LOG(WARN) << "close failed with error: " << errno;
                                JI_LOG(INFO) << "child process " << cit->first << " exited with status " << stat;
                                cit = childProcesses_.erase(cit);
                            }
                        }

                        for (auto cit = connections_.begin(); cit != connections_.end(); ++pit) {
                            assert(pit != pfds.end());

                            if (cit->process(*pit)) {
                                //JI_LOG(INFO) << "lambda " << cit->lambdaId() << " recv hello " << cit->recvHello();
                                if (cit->recvDone()) {
                                    lambdaId_to_connection_[cit->lambdaId()] = nullptr;
                                    cit = connections_.erase(cit);
                                    continue;
                                }
                                if (cit->recvHello() &&
                                        (lambdaId_to_connection_[cit->lambdaId()] == nullptr)) {
                                    lambdaId_to_connection_[cit->lambdaId()] = &(*cit);
                                    stop = std::chrono::high_resolution_clock::now();
                                    auto duration_to_lambda = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                                    JI_LOG(INFO) << "New lambda " << cit->lambdaId() << " new size " << lambdaId_to_connection_.size() << " duration " << duration_to_lambda.count() << " milliseconds";
                                    for (auto& buf: buffered_data_[cit->lambdaId()]) {
                                        JI_LOG(INFO) << "Writing buffered_data of num buffers " << buffered_data_[cit->lambdaId()].size();
                                        cit->write_buf(std::move(buf));
                                    }
                                    buffered_data_[cit->lambdaId()].clear();
                                } else {
                                    //JI_LOG(INFO) << "Existing lambda " << cit->lambdaId() << " new size " << lambdaId_to_connection_.size() << " " << (lambdaId_to_connection_[cit->lambdaId()] == nullptr);
                                }
                                ++cit;
                            }
                            else {
                                if (cit->recvHello()) {
                                    // it should be in the map
                                    lambdaId_to_connection_[cit->lambdaId()] = nullptr;
                                }
                                cit = connections_.erase(cit);
                            }
                        }

                        assert(pit + 1 == pfds.end());

                        if (pit->revents & (POLLERR | POLLHUP))
                            break;
                        if (pit->revents & POLLIN) {
                            JI_LOG(INFO) << "new connection";
                            connections_.emplace_back(*this);
                            if (!connections_.back()) {
                                JI_LOG(WARN) << "accept failed with error: " << errno;
                                connections_.pop_back();
                            }
                            // struct sockaddr_in addr;
                            // socklen_t addLen = sizeof(addr);

                            // int fd = ::accept(listen, &addr, &addrLen);
                            // if (fd != -1)
                            //     connections.emplace_back(socket_);
                            // && !connections.emplace_back(listen_))
                            // connections.pop_back();
                        }

                        if (done_) {
                            if (connections_.size() == 0) break;
                            if (done_ == 1) {
                                for (auto& c : connections_) {
                                    c.sendDone();
                                }
                                done_ += 1;
                                JI_LOG(INFO) << "Sending done to all connections";
                            }
                            break; // Don't wait for lambda queues to finish
                        }
                    }
                    JI_LOG(INFO) << "Loop done";

                    // Handle cleaning up lambdas
                    connections_.clear();

                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                    JI_LOG(INFO) << "Loop finished in " << duration;
                    JI_LOG(INFO) << "Num vertices in graph " << graph.vertexCount();
                    JI_LOG(INFO) << "Num edges in graph " << graph.edgeCount();
                    JI_LOG(INFO) << "Final samples_per_lambda " << num_samples_per_lambda_;
                    JI_LOG(INFO) << "Final global_num_samples " << getGlobalNumUniformSamples();
                }


                const std::uint64_t getGlobalNumUniformSamples() const {
                    return global_num_uniform_samples_;
                }

                const std::uint64_t getGlobalNumUniformSamples(std::uint64_t time_limit) const {
                    std::uint64_t prev = 0;
                    for (auto& [time, global_num_uniform_samples_curr] : timed_global_num_uniform_samples_) {
                        if (time > time_limit) break;
                        prev = global_num_uniform_samples_curr;
                    }
                    return prev;
                }

                void saveGraph(std::string filename) {
                    std::ofstream file(filename);
                    for (auto& [time, global_num_uniform_samples_curr] : timed_global_num_uniform_samples_) {
                        file << "num_global_uniform_samples;" 
                            << time << ";" 
                            << global_num_uniform_samples_curr 
                            << "\n";
}
                    file << "end_num_global_uniform_samples" << "\n";
                    graph.serialize(file);
                    file.close();
                }

                void loadGraph(std::string filename) {
                    std::ifstream file(filename);
                    // First get the num_global_uniform_samples
                    timed_global_num_uniform_samples_.clear();
                    for(std::string line; std::getline(file, line); ) {
                        if (line == "end_num_global_uniform_samples") break;
                        std::vector<std::string> results;
                        mpl::util::string_split(results, line, ";");
                        timed_global_num_uniform_samples_.push_back(std::make_pair(std::stoull(results[1]), std::stoull(results[2])));
                        global_num_uniform_samples_ = std::stoull(results[2]);
                    }

                    // Then read the graph
                    graph = TimedGraph::deserialize(file);
                    file.close();
                    JI_LOG(INFO) << "Num vertices in graph " << graph.vertexCount();
                    JI_LOG(INFO) << "Num edges in graph " << graph.edgeCount();
                    JI_LOG(INFO) << "Final global_num_samples " << global_num_uniform_samples_;
                }

                void init_lambdas() {
                    if (app_options.lambdaType() == LambdaType::LAMBDA_PSEUDO) {
                        init_local_lambdas();
                    } else if (app_options.lambdaType() == LambdaType::LAMBDA_AWS) {
                        init_aws_lambdas();
                    }

                    int num_lambdas = app_options.jobs();
                    num_samples_per_lambda_.resize(num_lambdas, 0);
                    lambdaId_to_connection_.resize(num_lambdas, nullptr);
                    buffered_data_.resize(num_lambdas);
                }

                Connection_t* getConnection(std::uint64_t lambdaId) {
                    return lambdaId_to_connection_[lambdaId];
                }

                int accept(struct sockaddr *addr, socklen_t * addrLen) {
                    return ::accept(listen_, addr, addrLen);
                }
                
                template <class Packet>
                void writePacketToLambda(int sourceLambdaId, int destinationLambdaId, Packet&& pkt) {
                    auto other_connection = getConnection(destinationLambdaId);
                    if (other_connection != nullptr) {
			JI_LOG(INFO) << "Writing " << pkt.vertices().size() << " vertices to lambda " << destinationLambdaId << " from " << sourceLambdaId;
                        other_connection->write(std::move(pkt));
                    } else {
		    	JI_LOG(INFO) << "Buffering " << pkt.vertices().size() << " vertices to lambda " << destinationLambdaId << " from " << sourceLambdaId;
                        buffered_data_[destinationLambdaId].push_back(std::move(pkt));
                    }
                }


        };

    template <class Scenario, class Scalar>
        class CoordinatorCommonSeed {
            public:
                using State = typename Scenario::State;
                using Bound = typename Scenario::Bound;
                using Distance = typename Scenario::Distance;
                using Subspace_t = Subspace<Bound, State, Scalar>;
                using Planner = mpl::PRMPlanner<Scenario, Scalar>;
                using Vertex = mpl::Vertex<State>;
                using Vertex_ID = typename Vertex::ID;
                using Edge = mpl::Edge<typename Vertex::ID, Distance>;
                //using Graph = UndirectedGraph<Vertex, Edge>;

                using TimedVertex = mpl::TimedVertex<State>;
                using TimedEdge = mpl::TimedEdge<typename Vertex::ID, Distance>;
                using TimedGraph = UndirectedGraph<TimedVertex, TimedEdge>;
                using Connection_t = Connection<CoordinatorCommonSeed>;

                std::vector<std::vector<Buffer>> buffered_data_; // Any data to be written if lambda is not up yet
                demo::AppOptions app_options;
                std::chrono::high_resolution_clock::time_point start_time;

            private:
                std::vector<std::uint64_t> num_samples_per_lambda_;
                std::uint64_t global_min_samples{0};
                std::uint64_t global_num_uniform_samples_{0};
                std::vector<std::pair<std::uint64_t, std::uint64_t>> timed_global_num_uniform_samples_;
                std::vector<Subspace_t> lambda_subspaces;
                int listen_{-1};
                int port_{0x415E};
                std::vector<struct pollfd> pfds;
                std::list<Connection_t> connections_;
                std::list<std::pair<int, int>> childProcesses_;
                std::vector<Connection_t*> lambdaId_to_connection_;
                TimedGraph graph;
#if HAS_AWS_SDK
                static constexpr const char* ALLOCATION_TAG = "mplLambdaAWS";
                std::shared_ptr<Aws::Lambda::LambdaClient> lambdaClient_;
#endif

                void init_aws_lambdas() {
#if !HAS_AWS_SDK
                    throw std::invalid_argument("AWS SDK is not available");
#else
                    //for (int i=0; i < app_options.jobs() ; ++i) {
                    //    Aws::Lambda::Model::InvokeRequest invokeRequest;
                    //    invokeRequest.SetFunctionName("mpl_fixed_graph_aws");
                    //    invokeRequest.SetInvocationType(Aws::Lambda::Model::InvocationType::Event);
                    //    std::shared_ptr<Aws::IOStream> payload = Aws::MakeShared<Aws::StringStream>("PayloadData");
                    //    Aws::Utils::Json::JsonValue jsonPayload;

                    //    //jsonPayload.WithString("algorithm", prob.algorithm() == 'r' ? "rrt" : "cforest");

                    //    auto lambdaId = std::to_string(i);
                    //    auto lambda_min = lambda_subspaces[i].getLower();
                    //    auto lambda_max = lambda_subspaces[i].getUpper();
                    //    std::vector<std::string> args = {
                    //        "scenario", app_options.scenario(),
                    //        "global_min", app_options.global_min_,
                    //        "global_max", app_options.global_max_,
                    //        "lambda_id", lambdaId,
                    //        "algorithm", app_options.algorithm(),
                    //        "coordinator", app_options.coordinator(),
                    //        //"communicator", app_options.communicator(),
                    //        "num_samples", std::to_string(app_options.numSamples()),
                    //        "time-limit", std::to_string(app_options.timeLimit()),
                    //        "env", app_options.env(false),
                    //        "env-frame", app_options.envFrame_,
                    //        "num_divisions", app_options.num_divisions_
                    //    };
                    //    args.push_back("start");
                    //    std::string starts;
                    //    for (int i=0; i < app_options.starts_.size(); ++i) {
                    //        starts += app_options.starts_[i];
                    //        if (i != app_options.starts_.size() - 1) starts += ";";
                    //    }
                    //    args.push_back(starts);

                    //    args.push_back("goal");
                    //    std::string goals;
                    //    for (int i=0; i < app_options.goals_.size(); ++i) {
                    //        goals += app_options.goals_[i];
                    //        if (i != app_options.goals_.size() - 1) goals += ";";
                    //    }
                    //    args.push_back(goals);

                    //    for (std::size_t i=0 ; i+1<args.size() ; i+=2) {
                    //        const std::string& key = args[i];
                    //        const std::string& val = args[i+1];
                    //        jsonPayload.WithString(
                    //                Aws::String(key.c_str(), key.size()),
                    //                Aws::String(val.c_str(), val.size()));
                    //    }

                    //    *payload << jsonPayload.View().WriteReadable();
                    //    invokeRequest.SetBody(payload);
                    //    invokeRequest.SetContentType("application/json");

                    //    auto outcome = lambdaClient_->Invoke(invokeRequest);
                    //    if (outcome.IsSuccess()) {
                    //        auto &result = outcome.GetResult();
                    //        Aws::IOStream& payload = result.GetPayload();
                    //        Aws::String functionResult;
                    //        std::getline(payload, functionResult);
                    //        JI_LOG(INFO) << "Lambda result: " << functionResult;
                    //    } else {
                    //        auto &error = outcome.GetError();
                    //        std::ostringstream msg;
                    //        msg << "name: '" << error.GetExceptionName() << "', message: '" << error.GetMessage() << "'";
                    //        throw std::runtime_error(msg.str());

                    //    }
                    //}
#endif

                }

                void init_local_lambdas() {
                    for (int i=0; i < app_options.jobs() ; ++i) {
                        int p[2];
                        if (::pipe(p) == -1)
                            throw std::system_error(errno, std::system_category(), "Pipe");
                        if (int pid = ::fork()) {
                            // parent process
                            ::close(p[1]);
                            continue;
                        }
                        ::close(p[0]);

                        std::string program = "./mpl_lambda_fixed_graph";
                        auto lambdaId = std::to_string(i);
                        std::vector<std::string> args = {
                            "--scenario", app_options.scenario(),
                            "--global_min", app_options.global_min_,
                            "--global_max", app_options.global_max_,
                            "--lambda_id", lambdaId,
                            "--algorithm", app_options.algorithm(),
                            "--coordinator", app_options.coordinator(),
                            //"--communicator", app_options.communicator(),
                            "--num_samples", std::to_string(app_options.numSamples()),
                            "--time-limit", std::to_string(app_options.timeLimit()),
                            "--env", app_options.env(false),
                            "--env-frame", app_options.envFrame_,
                            "--jobs", std::to_string(app_options.jobs_),
                        };
                        for (int i=0; i < app_options.starts_.size(); ++i) {
                            args.push_back("--start");
                            args.push_back(app_options.starts_[i]);
                        }
                        for (int i=0; i < app_options.goals_.size(); ++i) {
                            args.push_back("--goal");
                            args.push_back(app_options.goals_[i]);
                        }


                        std::vector <const char *> argv;

                        char file[20];
                        snprintf(file, sizeof(file), "lambda-%d.out", i);

                        argv.push_back(program.c_str());
                        std::string command = program;

                        for (auto ind=0; ind < args.size(); ++ind) {
                            argv.push_back(args[ind].c_str());
                            command += " " + args[ind];
                        }
                        argv.push_back(nullptr); // <-- required terminator
                        JI_LOG(TRACE) << "Running lambda" << lambdaId << " file " << file << ": " << command;

                        int fd = ::open(file, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
                        dup2(fd, 1); // make stdout write to file
                        dup2(fd, 2); // make stderr write to file
                        close(fd); // close fd, dups remain open
                        execv(argv[0], const_cast<char*const*>(argv.data()));
                        throw std::system_error(errno, std::system_category(), "Lambda died");
                    }

                }
            public:
                CoordinatorCommonSeed (demo::AppOptions& app_options_) :
                    app_options(app_options_)
            {
#if HAS_AWS_SDK
                if (app_options.lambdaType() == LambdaType::LAMBDA_AWS) {
                    JI_LOG(INFO) << "initializing lambda client";
                    Aws::SDKOptions options;
                    Aws::InitAPI(options);
                    Aws::Client::ClientConfiguration clientConfig;
                    clientConfig.region = "us-west-2";
                    lambdaClient_ = Aws::MakeShared<Aws::Lambda::LambdaClient>(ALLOCATION_TAG, clientConfig);
                }
#endif
            }

                ~CoordinatorCommonSeed() {
                    if (listen_ != -1 && !::close(listen_))
                        JI_LOG(WARN) << "failed to close listening socket";

#if HAS_AWS_SDK
                    if (app_options.lambdaType() == LambdaType::LAMBDA_AWS) {
                        Aws::SDKOptions options;
                        Aws::ShutdownAPI(options);
                    }
#endif
                }

                inline std::string algorithm() {
                    return "prm_common_seed";
                }

                void addVertices(const std::vector<Vertex>& vertices) {
                    auto stop = std::chrono::high_resolution_clock::now();
                    std::uint64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time).count();
                    for (auto& v : vertices) {
                        //JI_LOG(INFO) << "Adding " << v.id() << " at " << curr_time;
                        graph.addVertex(TimedVertex{v.id(), v.state(), curr_time});
                    }
                }

                void addEdges(const std::vector<Edge>& edges) {
                    auto stop = std::chrono::high_resolution_clock::now();
                    std::uint64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time).count();
                    for (auto& e : edges) {
                        //JI_LOG(INFO) << "Adding " << e.u() << "-" << e.v() << " at " << curr_time;
                        graph.addEdge(TimedEdge{e.distance(), e.u(), e.v(), curr_time});
                    }
                }

                void update_num_samples(std::uint64_t lambdaId, size_t successful_samples) {
                    num_samples_per_lambda_[lambdaId] += app_options.numSamples(); // Regardless of how many successful samples, each lambda tries to sample these many points
                    auto curr_min_samples = std::min_element(
                            num_samples_per_lambda_.begin(),
                            num_samples_per_lambda_.end()
                            );
                    if ((*curr_min_samples) > global_min_samples) {
                        global_min_samples = *curr_min_samples;
                        global_num_uniform_samples_ = global_min_samples;
                        JI_LOG(INFO) << "Updating global_num_samples to " << global_num_uniform_samples_;
                        auto stop = std::chrono::high_resolution_clock::now();
                        std::uint64_t curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time).count();
                        timed_global_num_uniform_samples_.push_back(std::make_pair(curr_time, global_num_uniform_samples_));
                    } else {
                        JI_LOG(INFO) << "Not updating global_num_samples " << global_min_samples << " from curr_min " << *curr_min_samples;

                    }
                    JI_LOG(INFO) << num_samples_per_lambda_;
                }


                const TimedGraph& getGraph() const {
                    return graph;
                };

                TimedGraph& getGraph() {
                    return graph;
                };
                
                void start_socket() {
                    if ((listen_ = ::socket(PF_INET, SOCK_STREAM, 0)) == -1)
                        throw syserr("socket()");

                    int on = 1;
                    if (::setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1)
                        throw syserr("set reuse addr");
                    // TODO: replace below code with MSG_NOSIGNAL in writev later
                    signal(SIGPIPE, SIG_IGN);


                    struct sockaddr_in addr;
                    addr.sin_family = AF_INET;
                    addr.sin_addr.s_addr = htonl(INADDR_ANY);
                    addr.sin_port = htons(port_);

                    if (::bind(listen_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1)
                        throw syserr("bind()");

                    socklen_t addrLen = sizeof(addr);
                    if (::getsockname(listen_, reinterpret_cast<struct sockaddr*>(&addr), &addrLen) == -1)
                        throw syserr("getsockname()");

                    JI_LOG(INFO) << "listening on port: " << ntohs(addr.sin_port);

                    if (::listen(listen_, 10000) == -1)
                        throw syserr("listen()");
                }

                void divide_work() {
                    JI_LOG(INFO) << "Num lambdas: " << app_options.jobs();
                }


                void loop() {
                    // Do communication stuff
                    int done_ = 0;
                    JI_LOG(INFO) << "loop started";

                    start_time = std::chrono::high_resolution_clock::now(); // class variable
                    for(;;) {


                        pfds.clear();
                        // first comes the fds of the child processes (note that
                        // connection processing may change the child process list, so
                        // this must be processed first)
                        for (auto[pid, fd] : childProcesses_) {
                            pfds.emplace_back();
                            pfds.back().fd = fd;
                            pfds.back().events = POLLIN;
                        }

                        // then et of pollfds is 1:1 with connections
                        for (Connection_t &conn : connections_)
                            pfds.emplace_back(conn);

                        // then comes the accepting socket
                        pfds.emplace_back();
                        pfds.back().fd = listen_;
                        pfds.back().events = POLLIN;

                        int nReady = ::poll(pfds.data(), pfds.size(), 1);
                        auto stop = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                        if (duration.count() > app_options.timeLimit() * 1000) { // specified time limit in s
                            if (!done_) done_ = 1;
                        }

                        if (nReady == -1) {
                            if (errno == EAGAIN || errno == EINTR) {
                                continue;
                            } else {
                                throw syserr("poll");
                            }

                        }

                        auto pit = pfds.begin();
                        for (auto cit = childProcesses_.begin(); cit != childProcesses_.end(); ++pit) {
                            assert(pit != pfds.end());

                            if ((pit->revents & POLLHUP) == 0) {
                                ++cit;
                            } else {
                                int stat = 0;
                                if (::close(cit->second) == -1)
                                    JI_LOG(WARN) << "close failed with error: " << errno;
                                JI_LOG(INFO) << "child process " << cit->first << " exited with status " << stat;
                                cit = childProcesses_.erase(cit);
                            }
                        }

                        for (auto cit = connections_.begin(); cit != connections_.end(); ++pit) {
                            assert(pit != pfds.end());

                            if (cit->process(*pit)) {
                                //JI_LOG(INFO) << "lambda " << cit->lambdaId() << " recv hello " << cit->recvHello();
                                if (cit->recvDone()) {
                                    lambdaId_to_connection_[cit->lambdaId()] = nullptr;
                                    cit = connections_.erase(cit);
                                    continue;
                                }
                                if (cit->recvHello() &&
                                        (lambdaId_to_connection_[cit->lambdaId()] == nullptr)) {
                                    lambdaId_to_connection_[cit->lambdaId()] = &(*cit);
                                    stop = std::chrono::high_resolution_clock::now();
                                    auto duration_to_lambda = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                                    JI_LOG(INFO) << "New lambda " << cit->lambdaId() << " new size " << lambdaId_to_connection_.size() << " duration " << duration_to_lambda.count() << " milliseconds";
                                    for (auto& buf: buffered_data_[cit->lambdaId()]) {
                                        JI_LOG(INFO) << "Writing buffered_data of num buffers " << buffered_data_[cit->lambdaId()].size();
                                        cit->write_buf(std::move(buf));
                                    }
                                    buffered_data_[cit->lambdaId()].clear();
                                } else {
                                    //JI_LOG(INFO) << "Existing lambda " << cit->lambdaId() << " new size " << lambdaId_to_connection_.size() << " " << (lambdaId_to_connection_[cit->lambdaId()] == nullptr);
                                }
                                ++cit;
                            }
                            else {
                                if (cit->recvHello()) {
                                    // it should be in the map
                                    lambdaId_to_connection_[cit->lambdaId()] = nullptr;
                                }
                                cit = connections_.erase(cit);
                            }
                        }

                        assert(pit + 1 == pfds.end());

                        if (pit->revents & (POLLERR | POLLHUP))
                            break;
                        if (pit->revents & POLLIN) {
                            JI_LOG(INFO) << "new connection";
                            connections_.emplace_back(*this);
                            if (!connections_.back()) {
                                JI_LOG(WARN) << "accept failed with error: " << errno;
                                connections_.pop_back();
                            }
                            // struct sockaddr_in addr;
                            // socklen_t addLen = sizeof(addr);

                            // int fd = ::accept(listen, &addr, &addrLen);
                            // if (fd != -1)
                            //     connections.emplace_back(socket_);
                            // && !connections.emplace_back(listen_))
                            // connections.pop_back();
                        }

                        if (done_) {
                            if (connections_.size() == 0) break;
                            if (done_ == 1) {
                                for (auto& c : connections_) {
                                    c.sendDone();
                                }
                                done_ += 1;
                                JI_LOG(INFO) << "Sending done to all connections";
                            }
                            break; // Don't wait for lambda queues to finish
                        }
                    }
                    JI_LOG(INFO) << "Loop done";

                    // Handle cleaning up lambdas
                    connections_.clear();

                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                    JI_LOG(INFO) << "Loop finished in " << duration;
                    JI_LOG(INFO) << "Num vertices in graph " << graph.vertexCount();
                    JI_LOG(INFO) << "Num edges in graph " << graph.edgeCount();
                    JI_LOG(INFO) << "Final samples_per_lambda " << num_samples_per_lambda_;
                    JI_LOG(INFO) << "Final global_num_samples " << getGlobalNumUniformSamples();
                }


                const std::uint64_t getGlobalNumUniformSamples() const {
                    return global_num_uniform_samples_;
                }

                const std::uint64_t getGlobalNumUniformSamples(std::uint64_t time_limit) const {
                    std::uint64_t prev = 0;
                    for (auto& [time, global_num_uniform_samples_curr] : timed_global_num_uniform_samples_) {
                        if (time > time_limit) break;
                        prev = global_num_uniform_samples_curr;
                    }
                    return prev;
                }

                void saveGraph(std::string filename) {
                    std::ofstream file(filename);
                    for (auto& [time, global_num_uniform_samples_curr] : timed_global_num_uniform_samples_) {
                        file << "num_global_uniform_samples;" 
                            << time << ";" 
                            << global_num_uniform_samples_curr 
                            << "\n";
}
                    file << "end_num_global_uniform_samples" << "\n";
                    graph.serialize(file);
                    file.close();
                }

                void loadGraph(std::string filename) {
                    std::ifstream file(filename);
                    // First get the num_global_uniform_samples
                    timed_global_num_uniform_samples_.clear();
                    for(std::string line; std::getline(file, line); ) {
                        if (line == "end_num_global_uniform_samples") break;
                        std::vector<std::string> results;
                        mpl::util::string_split(results, line, ";");
                        timed_global_num_uniform_samples_.push_back(std::make_pair(std::stoull(results[1]), std::stoull(results[2])));
                        global_num_uniform_samples_ = std::stoull(results[2]);
                    }

                    // Then read the graph
                    graph = TimedGraph::deserialize(file);
                    file.close();
                    JI_LOG(INFO) << "Num vertices in graph " << graph.vertexCount();
                    JI_LOG(INFO) << "Num edges in graph " << graph.edgeCount();
                    JI_LOG(INFO) << "Final global_num_samples " << global_num_uniform_samples_;
                }

                void init_lambdas() {
                    if (app_options.lambdaType() == LambdaType::LAMBDA_PSEUDO) {
                        init_local_lambdas();
                    } else if (app_options.lambdaType() == LambdaType::LAMBDA_AWS) {
                        init_aws_lambdas();
                    }

                    int num_lambdas = app_options.jobs();
                    num_samples_per_lambda_.resize(num_lambdas, 0);
                    lambdaId_to_connection_.resize(num_lambdas, nullptr);
                    buffered_data_.resize(num_lambdas);
                }

                Connection_t* getConnection(std::uint64_t lambdaId) {
                    return lambdaId_to_connection_[lambdaId];
                }

                int accept(struct sockaddr *addr, socklen_t * addrLen) {
                    return ::accept(listen_, addr, addrLen);
                }
                
                template <class Packet>
                void writePacketToLambda(int sourceLambdaId, int destinationLambdaId, Packet&& pkt) {
                    JI_LOG(ERROR) << "Should not send vertices for this algorithm";
                }

                const std::vector<Subspace_t> getSubspaces() const {
                    std::vector<Subspace_t> subspaces;
                    subspaces.push_back(Subspace_t(app_options.globalMin<Bound>(), app_options.globalMax<Bound>()));
                    return subspaces;
                }
        };

}


#endif //MPL_COORDINATOR_HPP
