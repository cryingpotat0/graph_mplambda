#pragma once
#ifndef MPL_ROBOT_HPP
#define MPL_ROBOT_HPP

#include <demo/png_2d_scenario.hpp>
#include <demo/shape_hierarchy.hpp>
#include <demo/fetch_scenario.hpp>
#include <list>
#include <chrono>
#include <time.h>
#include <prm_planner.hpp>
#include <demo/multi_agent_png_2d_scenario.hpp>
#include <demo/se3_rigid_body_scenario.hpp>
#include <util.hpp>

#ifndef NUM_AGENTS
#define NUM_AGENTS 20
#endif

namespace mpl::demo {
    template <class Coordinator, class State, class Paths, class Graph>
    void saveSolutionPaths(const Coordinator& coord, const AppOptions &app_options, const Paths& paths, const Graph& graph) {

        std::vector<FilterColor> filters;

        auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
        int i=0;
        for (auto& [locs, info] : paths) {
            auto& [start, goal] = locs;
            auto& [found, path] = info;
            const std::string outputName = "png_2d_demo_output-" + std::to_string(i++) + ".svg";
            //std::to_string(start.first) + "_" + std::to_string(start.second) + "-" +
            //std::to_string(goal.first) + "_" + std::to_string(goal.second) + ".svg";
            std::ofstream file(outputName);
            shape::startSvg(file, width, height);
            shape::addBackgroundImg(file, app_options.env());

            shape::addStartState(file, start[0], start[1], 10);
            shape::addGoalState(file, goal[0], goal[1], 10);

            if (found) {
                for (auto it=path.begin(); it < path.end() - 1; it++) {
                    auto u = graph.getVertex((*it));
                    auto v = graph.getVertex(*(it+1));
                    shape::addSolutionEdge(file, u.state()[0], u.state()[1], v.state()[0], v.state()[1]);
                }
            }
            shape::endSvg(file);
        }
    }


    template <class State, class Paths, class Graph>
    void visualizeMultiAgentPaths(const AppOptions &app_options, const Paths& paths, const Graph& graph) {
        std::vector<FilterColor> filters;
        auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
        int j=0;
        for (auto& [locs, info] : paths) {
            JI_LOG(INFO) << "Visualizing path";
            auto& [start, goal] = locs;
            auto& [found, path] = info;
            const std::string outputName = "png_2d_demo_output-" + std::to_string(j++) + ".svg";
            //std::to_string(start.first) + "_" + std::to_string(start.second) + "-" +
            //std::to_string(goal.first) + "_" + std::to_string(goal.second) + ".svg";
            std::ofstream file(outputName);
            shape::startSvg(file, width, height);
            shape::addBackgroundImg(file, app_options.env());

            for (int i=0; i < NUM_AGENTS; ++i) {
                shape::addStartState(file, start[2*i], start[2*i+1], 10);
                shape::addGoalState(file, goal[2*i], goal[2*i+1], 10);
            }

            if (found) {
                std::vector<std::vector<std::vector<double>>> path_by_agent;
                path_by_agent.resize(NUM_AGENTS);
                for (auto it=path.begin(); it < path.end() - 1; it++) {
                    auto u = graph.getVertex((*it));
                    auto v = graph.getVertex(*(it+1));
                    for (int i=0; i < NUM_AGENTS; ++i) {
                        shape::addSolutionEdge(file, u.state()[2*i], u.state()[2*i+1], v.state()[2*i], v.state()[2*i + 1]);
                        std::vector<double> curr = {u.state()[2*i], u.state()[2*i + 1]};
                        path_by_agent[i].push_back(curr);
                    }
                }
                for (int i=0; i < NUM_AGENTS; ++i) {
                    std::vector<double> curr = {goal[2*i], goal[2*i + 1]};
                    path_by_agent[i].push_back(curr);
                    shape::addAnimatedState(file, start[2*i], start[2*i + 1], 10, path_by_agent[i]);
                }
            }
            shape::endSvg(file);
        }
    }

    template <class State, class Paths, class Graph>
    void visualizeSequentialMultiAgentPaths(const AppOptions &app_options, const Paths& paths, const Graph& graph) {
        std::vector<FilterColor> filters;
        auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
        int j=0;
        int agentRadius = 10;
        for (auto& [locs, info] : paths) {
            JI_LOG(INFO) << "Visualizing path";
            auto& [start, goal] = locs;
            //auto& [found, path, velocities] = info;
            const std::string outputName = "png_2d_demo_output-" + std::to_string(j++) + ".svg";
            //std::to_string(start.first) + "_" + std::to_string(start.second) + "-" +
            //std::to_string(goal.first) + "_" + std::to_string(goal.second) + ".svg";
            std::ofstream file(outputName);
            shape::startSvg(file, width, height);
            shape::addBackgroundImg(file, app_options.env());

            for (int i=0; i < NUM_AGENTS; ++i) {
                shape::addState(file, start[2*i], start[2*i+1], agentRadius, "S" + std::to_string(i));
                shape::addState(file, goal[2*i], goal[2*i+1], agentRadius, "G" + std::to_string(i));
            }
            int i=0;
            for (auto& [found, path, velocities] : info) {
                // for each agent do
                if (found) {
                    std::vector<std::pair<std::pair<double, double>, double>> path_with_velocity; // (u, velocity)
                    for (int i=0; i < path.size(); ++i) {
                        auto u = graph.getVertex(path[i]).state();
                        auto velocity = velocities[i];
                        path_with_velocity.emplace_back(std::make_pair(std::make_pair(u[0], u[1]), velocity));
                        if (i < path.size() - 1) {
                            auto v = graph.getVertex(path[i+1]).state();
                            //shape::addSolutionEdge(file, u[0], u[1], v[0], v[1]);
                        }
                    }
                    shape::addAnimatedStateWithVelocity(file, agentRadius, path_with_velocity);
                } else {
                    for (auto& id: path) { // path contains everything in the fringe
                        auto& u = graph.getVertex(id).state();
                        auto adjacency_list = graph.getAdjacencyList();
                        auto outgoing_edges = adjacency_list.find(id);
                        if (outgoing_edges != adjacency_list.end()) {
                            for (auto& other_id: outgoing_edges->second) {
                                auto& v = graph.getVertex(other_id).state();
                                //shape::addVisitedEdge(file, u[0], u[1], v[0], v[1]);
                            }

                        }
                        shape::addState(file, u[0], u[1], 2, std::to_string(i));
                    }
                }

                i++;
            }
            shape::endSvg(file);
        }
    }

    template <class Scalar>
    FetchScenario<Scalar> initFetchScenario(AppOptions& app_options) {
        using Scenario = FetchScenario<Scalar>;
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;
        using Frame = typename Scenario::Frame;
        using GoalRadius = Eigen::Matrix<Scalar, 6, 1>;

        // TODO: hardcoded values in SE3, unused in code for now
        app_options.correct_goal_ = "-1.07,0.16,0.88,0,0,0";
        app_options.goalRadius_ = "0.01,0.01,0.01,0.01,0.01,3.14";

        Frame envFrame = app_options.envFrame<Frame>();
        Frame goal = app_options.correct_goal<Frame>();
        GoalRadius goalRadius = app_options.goalRadius<GoalRadius>();
        auto min = app_options.globalMin<Bound>();
        auto max = app_options.globalMax<Bound>();

        Scenario scenario(envFrame, app_options.env(), goal, goalRadius, min, max, app_options.checkResolution(0.1));
        return scenario;
    }

    template <class Scalar>
    SE3RigidBodyScenario<Scalar> initSE3Scenario(AppOptions& app_options) {
        using Scenario = SE3RigidBodyScenario<Scalar>;
        using Bound = typename Scenario::Bound;
        using State = typename Scenario::State;
        State goal = app_options.goal<State>();
        Bound min = app_options.globalMin<Bound>();
        Bound max = app_options.globalMax<Bound>();
        Scenario scenario(app_options.env(), app_options.robot(), goal, min, max, app_options.checkResolution(0.1));
        return scenario;
    }

    template <class Scalar>
    PNG2dScenario<Scalar> initPngScenario(AppOptions &app_options) {
        using Scenario = PNG2dScenario<Scalar>;
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;

        std::vector<FilterColor> filters;
        if (app_options.env(false).empty()) {
            app_options.env_ = "resources/png_planning_input.png";
        }

        if (app_options.env() == "resources/png_planning_input.png") {
            JI_LOG(INFO) << "Using png planning input";
            filters.emplace_back(FilterColor(126, 106, 61, 15));
            filters.emplace_back(FilterColor(61, 53, 6, 15));
            filters.emplace_back(FilterColor(255, 255, 255, 5));
        } else if (app_options.env() == "resources/house_layout.png") {
            JI_LOG(INFO) << "Using house layout input";
            filters.emplace_back(FilterColor(0, 0, 0, 5));
            filters.emplace_back(FilterColor(224, 224, 224, 5));
            filters.emplace_back(FilterColor(255, 255, 255, 5));
        }
        auto min = app_options.globalMin<Bound>();
        auto max = app_options.globalMax<Bound>();
        //Subspace_t global_subspace(min, max);

        auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
        JI_LOG(INFO) << "width " << width << " height " << height;
        return Scenario(width, height, min, max, obstacles);
    }

    template <class Scalar, int num_agents>
    MultiAgentPNG2DScenario<Scalar, num_agents> initMultiAgentPNG2DScenario(AppOptions &app_options) {
        using Scenario = MultiAgentPNG2DScenario<Scalar, num_agents>;
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;
        using SingleAgentState = typename Scenario::SingleAgentState;

        std::vector<FilterColor> filters;
        if (app_options.env(false).empty()) {
            app_options.env_ = "resources/png_planning_input.png";
        }

        if (app_options.env() == "resources/png_planning_input.png") {
            JI_LOG(INFO) << "Using png planning input";
            filters.emplace_back(FilterColor(126, 106, 61, 15));
            filters.emplace_back(FilterColor(61, 53, 6, 15));
            filters.emplace_back(FilterColor(255, 255, 255, 5));
        } else if (app_options.env() == "resources/house_layout.png") {
            JI_LOG(INFO) << "Using house layout input";
            filters.emplace_back(FilterColor(0, 0, 0, 5));
            filters.emplace_back(FilterColor(224, 224, 224, 5));
            filters.emplace_back(FilterColor(255, 255, 255, 5));
        }
        auto min_b = app_options.globalMin<SingleAgentState>();
        auto max_b = app_options.globalMax<SingleAgentState>();
        Bound min;
        Bound max;
        for (int i=0; i < num_agents; ++i) {
                min[2*i] = min_b[0];
                min[2*i + 1] = min_b[1];
                max[2*i] = max_b[0];
                max[2*i + 1] = max_b[1];
        }
        //Subspace_t global_subspace(min, max);

        auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
        return Scenario(width, height, min, max, obstacles);

    }

    template <class Coordinator, class State, class Graph>
    void savePngImages(const Coordinator& coord, AppOptions &app_options, Graph& graph) {

        std::vector<FilterColor> filters;

        auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
        //auto startState = app_options.start<State>(); // 430, 1300;
        //auto goalState = app_options.goal<State>(); // 3150, 950

        const std::string outputName = "png_2d_demo_output.svg";
        std::ofstream file(outputName);
        shape::startSvg(file, width, height);
        shape::addBackgroundImg(file, app_options.env());
        for (auto& subspace : coord.getSubspaces()) {
            //file << shape::Rect(
            //        subspace.getLower()[0],
            //        subspace.getLower()[1],
            //        subspace.getUpper()[0],
            //        subspace.getUpper()[1],
            //        shape::Color(0, 255, 0));
        }

        for (auto& [v_id, u_ids] : graph.getAdjacencyList()) {
            auto start = graph.getVertex(v_id);
            for (auto u_id : u_ids) {
                auto end = graph.getVertex(u_id);
                auto& [lambdaId1, vertexId1] = v_id;
                auto& [lambdaId2, vertexId2] = u_id;

                if (lambdaId1 == lambdaId2) {
                    // The 0th element indicates which lambda
                    shape::addEdge(file, start.state()[0], start.state()[1], end.state()[0], end.state()[1],
                                   6, shape::Color(40, 100, 100));
                } else {
                    shape::addEdge(file, start.state()[0], start.state()[1], end.state()[0], end.state()[1],
                                   6, shape::Color(250, 50, 50));
                }
            }
        }
        for (auto& [v_id, v]: graph.getVertices()) {
            auto state = v.state();
            shape::addState(file, state[0], state[1], 5, ' ');
        }
        for (int i=0; i < coord.getSubspaces().size(); ++i) {
            auto lower = coord.getSubspaces()[i].getLower();
            auto upper = coord.getSubspaces()[i].getUpper();
            auto mid = (lower + upper) / 2;
            //shape::addText(file, std::to_string(i), mid[0], mid[1], shape::Color(50, 50, 250), 48);
        }
        shape::endSvg(file);
    }

    template <class Graph, class Scenario, class Vertex>
    ////std::vector<std::pair<bool, std::vector<State>>> findPathsFromStartToGoals(const Graph& graph, const Scenario& scenario)
    decltype(auto) findPathsFromStartToGoals(const Graph& graph, Scenario& scenario, const std::vector<std::pair<Vertex, Vertex>>& startsAndGoals) {
        using State = typename Scenario::State;
        using VertexID = typename Vertex::ID;
        std::vector<std::pair<
                std::pair<State, State>,
                std::pair<bool, std::vector<VertexID>>
        >> pathsFromStartToGoals; // ((start, goal) -> (found, path))
        for (auto& [start, goal]: startsAndGoals) {
            //scenario.setGoal(goal.state());
            auto start_djikstras = std::chrono::high_resolution_clock::now();
            auto path = graph.djikstras(start.id(), goal.id());
            //auto path = graph.djikstras(start.id(), [&scenario] (auto& vertex) {
            //    	    return scenario.isGoal(vertex.state());
            //    	    });
            auto end_djikstras = std::chrono::high_resolution_clock::now();
            JI_LOG(INFO) << "Time to do djikstras for start " << start.state() << " goal " << goal.state()
                         << " is " << std::chrono::duration_cast<std::chrono::milliseconds>(end_djikstras - start_djikstras);
            auto identifier = std::make_pair(start.state(), goal.state());
            if (path.first) {
                double d = 0;
                for (auto it=path.second.begin(); it < path.second.end() - 1; it++) {
                    auto u = (*it);
                    auto v = (*(it+1));
                    //JI_LOG(INFO) << "Edge " << graph.getVertex(u).state() << "-" << graph.getVertex(v).state() << " edgeid " << u << "-" << v << " distance " << graph.getEdge(u, v).distance();
                    d += graph.getEdge(u, v).distance();
                }
                JI_LOG(INFO) << "Path found for start " << start.state() << " goal " << goal.state() << " with distance " << d;
            } else {
                JI_LOG(INFO) << "No Path found for start " << start.state() << " goal " << goal.state() << " with distance inf";
            }
            pathsFromStartToGoals.push_back(std::make_pair(identifier, path));
        }
        return pathsFromStartToGoals;
    }

    template <class Graph, class TimedGraph, class Vertex, class Edge>
    void getGraphAtTime(const TimedGraph& graph, Graph& return_graph, std::uint64_t time_limit) {
        auto adjacency_list = graph.getAdjacencyList();
        auto vertices = graph.getVertices();
        //for (auto &[curr_id, others] : adjacency_list) {
        for (auto &[curr_id, v] : vertices) {
            //auto v = graph.getVertex(curr_id);
            if (v.timestamp_millis() > time_limit) continue;
            return_graph.addVertex(Vertex{v.id(), v.state()});
            auto others = adjacency_list.find(curr_id);
            if (others == adjacency_list.end()) continue;
            for (auto& u : others->second) {
                auto& curr_edge = graph.getEdge(curr_id, u);
                //if (curr_edge.timestamp_millis() < v.timestamp_millis()) {
                //    JI_LOG(ERROR) << "BIG ERROR, VERTICES SHOULD BE ADDED BEFORE EDGES " << curr_edge.timestamp_millis() << " " << v.timestamp_millis() << " " << graph.getVertex(u).timestamp_millis();
                //} // Test to make sure nothing was broken
                if (curr_edge.timestamp_millis() > time_limit) continue;
                return_graph.addEdge(Edge{curr_edge.distance(), curr_edge.u(), curr_edge.v()});
            }
        }
        JI_LOG(INFO) << "Num vertices in graph " << return_graph.vertexCount();
        JI_LOG(INFO) << "Num edges in graph " << return_graph.edgeCount();
    }

    template <class Scenario, class Graph, class Vertex>
    decltype(auto) connectStartsAndGoals(Scenario& scenario, AppOptions& app_options, Graph& graph, std::uint64_t global_num_uniform_samples) {
        using State = typename Scenario::State;
        using Scalar = double; // TODO: don't hardcode this

        JI_LOG(INFO) << "Final global_num_samples " << global_num_uniform_samples;
        std::vector<std::pair<Vertex, Vertex>> start_goal_vertices;
        auto planner = mpl::PRMPlanner<Scenario, Scalar>(scenario, -1); // Use -1 as the standard prefix
        planner.clearVertices(); planner.clearEdges();
        planner.updatePrmRadius(global_num_uniform_samples);
        for (auto& [v_id, connections]: graph.getAdjacencyList()) {
            auto vertex = graph.getVertex(v_id);
            planner.addExistingVertex(vertex);
        }

        Vertex curr_start, curr_goal;
        if (app_options.goals_.size() == 0) {
            randomizeMultiAgentGoals(scenario, app_options);
        }
        for (auto& [start, goal] : app_options.getStartsAndGoals<State>()) {
            auto start_time = std::chrono::high_resolution_clock::now();
            planner.addSample(start);
            curr_start = planner.getNewVertices()[0]; // should always exist
            graph.addVertex(curr_start);
            for (auto& e: planner.getNewEdges()) graph.addEdge(e);
            planner.clearVertices(); planner.clearEdges();

            planner.addSample(goal);
            curr_goal = planner.getNewVertices()[0]; // should always exist
            graph.addVertex(curr_goal);
            for (auto& e: planner.getNewEdges()) graph.addEdge(e);
            planner.clearVertices(); planner.clearEdges();

            auto end_time = std::chrono::high_resolution_clock::now();
            JI_LOG(INFO) << "Time to add start " << curr_start.id() << ": " << start
                         << "; goal " << curr_goal.id() << ": " << goal
                         << " is " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            start_goal_vertices.emplace_back(std::make_pair(curr_start, curr_goal));
        }
        return start_goal_vertices;
    }

    template <class Coordinator, class Scalar>
    void pngPostProcessing(Coordinator& coord, AppOptions& app_options) {
        using Scenario = PNG2dScenario<Scalar>;
        using State = typename Scenario::State;
        using Vertex = typename Coordinator::Vertex;
        using Edge = typename Coordinator::Edge;
        using Graph = typename mpl::UndirectedGraph<Vertex, Edge>;
        using TimedGraph = typename Coordinator::TimedGraph;

        Scenario scenario = initPngScenario<Scalar>(app_options);

        int evaluate_every_millis = 10000;
        //long int current_time_limit = app_options.timeLimit() * 1000;
        long int start_time = app_options.timeLimit() * 1000;
        long int current_time_limit = start_time;
        while (current_time_limit > 0) {
            JI_LOG(INFO) << "Current time limit: " << current_time_limit;
            Graph graph;
            getGraphAtTime<Graph, TimedGraph, Vertex, Edge>(coord.getGraph(), graph, current_time_limit);
            if (current_time_limit == start_time) {
                savePngImages<Coordinator, State>(coord, app_options, graph);
            }
            auto startsAndGoals = connectStartsAndGoals<Scenario, Graph, Vertex>(scenario, app_options, graph, coord.getGlobalNumUniformSamples(current_time_limit));
            auto paths = findPathsFromStartToGoals(graph, scenario, startsAndGoals);
            if (current_time_limit == start_time) {
                saveSolutionPaths<Coordinator, State>(coord, app_options, paths, graph);
            }
            current_time_limit -= evaluate_every_millis;
        }
    }

    template <class Coordinator, class Scalar>
    void fetchPostProcessing(Coordinator& coord, AppOptions& app_options) {
        using Scenario = FetchScenario<Scalar>;
        using State = typename Scenario::State;
        using Vertex = typename Coordinator::Vertex;
        using Edge = typename Coordinator::Edge;
        using Graph = typename mpl::UndirectedGraph<Vertex, Edge>;
        using TimedGraph = typename Coordinator::TimedGraph;
        Scenario scenario = initFetchScenario<Scalar>(app_options);
        int evaluate_every_millis = 3000;
        long int current_time_limit = app_options.timeLimit() * 1000;
        while (current_time_limit > 0) {
            JI_LOG(INFO) << "Current time limit: " << current_time_limit;
            Graph graph;
            getGraphAtTime<Graph, TimedGraph, Vertex, Edge>(coord.getGraph(), graph, current_time_limit); // Time limit specified in seconds
            auto startsAndGoals = connectStartsAndGoals<Scenario, Graph, Vertex>(scenario, app_options, graph, coord.getGlobalNumUniformSamples(current_time_limit));
            auto paths = findPathsFromStartToGoals(graph, scenario, startsAndGoals);
            current_time_limit -= evaluate_every_millis;
        }
        //auto startsAndGoals = connectStartsAndGoals(scenario, app_options, coord, graph);
        //auto paths = findPathsFromStartToGoals(graph, scenario, startsAndGoals, app_options);
    }

    template <class Scenario>
    void randomizeMultiAgentGoals(Scenario& scenario, AppOptions& app_options, int num_goals=5, int seed=-1) {
        using RNG = std::mt19937_64;
        if (seed == -1) seed = time(NULL);
        RNG rng(seed);
        auto state = scenario.randomSample(rng);
        for(int i=0; i < num_goals; ++i) {
            while (!scenario.isValid(state)) state = scenario.randomSample(rng);
            app_options.goals_.push_back(mpl::util::ToString(state.format(mpl::util::FullPrecisionCommaInitFormat)));
            state = scenario.randomSample(rng);
        }
        JI_LOG(INFO) << app_options.goals_;
    }

    template <class Coordinator, class Scalar>
    void multiAgentPngPostProcessing(Coordinator& coord, AppOptions& app_options) {
        using Scenario = MultiAgentPNG2DScenario<Scalar, NUM_AGENTS>;
        using State = typename Scenario::State;

        using Vertex = typename Coordinator::Vertex;
        using Edge = typename Coordinator::Edge;
        using Graph = typename mpl::UndirectedGraph<Vertex, Edge>;
        using TimedGraph = typename Coordinator::TimedGraph;
        Scenario scenario = initMultiAgentPNG2DScenario<Scalar, NUM_AGENTS>(app_options);
        long int current_time_limit = app_options.timeLimit() * 1000;
        JI_LOG(INFO) << "Current time limit: " << current_time_limit;
        Graph graph;
        getGraphAtTime<Graph, TimedGraph, Vertex, Edge>(coord.getGraph(), graph, current_time_limit); // Time limit specified in seconds
        auto startsAndGoals = connectStartsAndGoals<Scenario, Graph, Vertex>(scenario, app_options, graph, coord.getGlobalNumUniformSamples(current_time_limit));
        auto paths = findPathsFromStartToGoals(graph, scenario, startsAndGoals);
        visualizeMultiAgentPaths<State>(app_options, paths, graph);
    }

    template <class Coordinator, class Scalar>
    void sequentialMultiAgentPngPostProcessing(Coordinator& coord, AppOptions& app_options) {
        using Scenario = PNG2dScenario<Scalar>;
        using State = typename Scenario::State;
        using MultiAgentScenario = MultiAgentPNG2DScenario<Scalar, NUM_AGENTS>;
        using MultiAgentState = typename MultiAgentScenario::State;
        using Vertex = typename Coordinator::Vertex;
        using Edge = typename Coordinator::Edge;
        using Graph = typename mpl::UndirectedGraph<Vertex, Edge>;
        using TimedGraph = typename Coordinator::TimedGraph;

        Scenario scenario = initPngScenario<Scalar>(app_options);

        int evaluate_every_millis = 10000;
        //long int current_time_limit = app_options.timeLimit() * 1000;
        long int start_time = app_options.timeLimit() * 1000;
        long int current_time_limit = start_time;
        JI_LOG(INFO) << "Current time limit: " << current_time_limit;
        Graph graph;
        getGraphAtTime<Graph, TimedGraph, Vertex, Edge>(coord.getGraph(), graph, current_time_limit);
        //savePngImages<Coordinator, State>(coord, app_options, graph);
        //auto startsAndGoals = connectStartsAndGoals<Scenario, Graph, Vertex>(scenario, app_options, graph, coord.getGlobalNumUniformSamples(current_time_limit));
        if (app_options.goals_.size() == 0) {
            auto goal_scenario = initMultiAgentPNG2DScenario<Scalar, NUM_AGENTS>(app_options);
            randomizeMultiAgentGoals(goal_scenario, app_options, 10, 1);
        }

        auto paths = sequentialMultiAgentPlanning<Graph, Scenario, Vertex, Edge, MultiAgentScenario>(graph, scenario, app_options, coord.getGlobalNumUniformSamples(current_time_limit));
        visualizeSequentialMultiAgentPaths<State>(app_options, paths, graph);
        //saveSolutionPaths<Coordinator, State>(coord, app_options, paths, graph);

    }
}

#endif 
