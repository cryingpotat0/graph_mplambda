#pragma once
#ifndef MPL_ROBOT_HPP
#define MPL_ROBOT_HPP

#include <demo/png_2d_scenario.hpp>
#include <demo/shape_hierarchy.hpp>
#include <demo/fetch_scenario.hpp>
#include <list>
#include <chrono>
#include <prm_planner.hpp>

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

                shape::addStartState(file, start[0], start[1], 20);
                shape::addGoalState(file, goal[0], goal[1], 20);

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
        PNG2dScenario<Scalar> initPngScenario(AppOptions &app_options) {
            using Scenario = PNG2dScenario<Scalar>;
            using State = typename Scenario::State;
            using Bound = typename Scenario::Bound;

            std::vector<FilterColor> filters;
            if (app_options.env(false).empty()) {
                app_options.env_ = "resources/png_planning_input.png";
            }

            if (app_options.env() == "resources/png_planning_input.png") {
                filters.emplace_back(FilterColor(126, 106, 61, 15));
                filters.emplace_back(FilterColor(61, 53, 6, 15));
                filters.emplace_back(FilterColor(255, 255, 255, 5));
            } else if (app_options.env() == "resources/house_layout.png") {
                filters.emplace_back(FilterColor(0, 0, 0, 5));
                filters.emplace_back(FilterColor(224, 224, 224, 5));
                filters.emplace_back(FilterColor(255, 255, 255, 5));
            }
            auto min = app_options.globalMin<Bound>();
            auto max = app_options.globalMax<Bound>();
            //Subspace_t global_subspace(min, max);

            auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
            return Scenario(width, height, min, max, obstacles);
        }


    template <class Coordinator, class State, class Graph>
        void savePngImages(const Coordinator& coord, AppOptions &app_options, Graph& graph) {

            std::vector<FilterColor> filters;

            auto [obstacles, width, height] = mpl::demo::readAndFilterPng(filters, app_options.env());
            auto startState = app_options.start<State>(); // 430, 1300;
            auto goalState = app_options.goal<State>(); // 3150, 950

            const std::string outputName = "png_2d_demo_output.svg";
            std::ofstream file(outputName);
            shape::startSvg(file, width, height);
            shape::addBackgroundImg(file, app_options.env());
            for (auto& subspace : coord.getSubspaces()) {
                file << shape::Rect(
                        subspace.getLower()[0],
                        subspace.getLower()[1],
                        subspace.getUpper()[0],
                        subspace.getUpper()[1],
                        shape::Color(0, 255, 0));
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
                                6, shape::Color(40, 40, 40));
                    } else {
                        shape::addEdge(file, start.state()[0], start.state()[1], end.state()[0], end.state()[1],
                                6, shape::Color(250, 50, 50));
                    }
                }
            }
            for (auto& [v_id, u_id]: graph.getAdjacencyList()) {
                auto state = graph.getVertex(v_id).state();
                shape::addState(file, state[0], state[1], 2, 'v');
            }
            for (int i=0; i < coord.getSubspaces().size(); ++i) {
                auto lower = coord.getSubspaces()[i].getLower();
                auto upper = coord.getSubspaces()[i].getUpper();
                auto mid = (lower + upper) / 2;
                shape::addText(file, std::to_string(i), mid[0], mid[1], shape::Color(50, 50, 250), 48);
            }
            shape::endSvg(file);
        }

    template <class Graph, class Scenario, class Vertex>
        ////std::vector<std::pair<bool, std::vector<State>>> findPathsFromStartToGoals(const Graph& graph, const Scenario& scenario) 
        decltype(auto) findPathsFromStartToGoals(const Graph& graph, Scenario& scenario, const std::vector<std::pair<Vertex, Vertex>>& startsAndGoals, AppOptions& app_options) {
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
            for (auto &[curr_id, others] : adjacency_list) {
                auto v = graph.getVertex(curr_id);
                if (v.timestamp_millis() > time_limit) continue;
                return_graph.addVertex(Vertex{v.id(), v.state()});
                for (auto& u : others) {
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
            
            int evaluate_every_millis = 250;
            std::uint64_t current_time_limit = app_options.timeLimit() * 1000;
            while (current_time_limit > 0) {
                JI_LOG(INFO) << "Current time limit: " << current_time_limit;
                Graph graph;
                getGraphAtTime<Graph, TimedGraph, Vertex, Edge>(coord.getGraph(), graph, current_time_limit); 
                auto startsAndGoals = connectStartsAndGoals<Scenario, Graph, Vertex>(scenario, app_options, graph, coord.getGlobalNumUniformSamples(current_time_limit));
                auto paths = findPathsFromStartToGoals(graph, scenario, startsAndGoals, app_options);
                if (current_time_limit == app_options.timeLimit() * 1000) {
                    savePngImages<Coordinator, State>(coord, app_options, graph);
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
            Graph graph;
            getGraphAtTime<Graph, TimedGraph, Vertex, Edge>(coord.getGraph(), graph, app_options.timeLimit() * 1000); // Time limit specified in seconds
            int evaluate_every_millis = 1000;
            std::uint64_t current_time_limit = app_options.timeLimit() * 1000;
            while (current_time_limit > 0) {
                JI_LOG(INFO) << "Current time limit: " << current_time_limit;
                Graph graph;
                getGraphAtTime<Graph, TimedGraph, Vertex, Edge>(coord.getGraph(), graph, current_time_limit); // Time limit specified in seconds
                auto startsAndGoals = connectStartsAndGoals<Scenario, Graph, Vertex>(scenario, app_options, graph, coord.getGlobalNumUniformSamples(current_time_limit));
                auto paths = findPathsFromStartToGoals(graph, scenario, startsAndGoals, app_options);
                current_time_limit -= evaluate_every_millis;
            }
            //auto startsAndGoals = connectStartsAndGoals(scenario, app_options, coord, graph);
            //auto paths = findPathsFromStartToGoals(graph, scenario, startsAndGoals, app_options);
        }



}

#endif 
