#ifndef MPL_PRM_PLANNER_HPP
#define MPL_PRM_PLANNER_HPP
#include <subspace.hpp>
#include <graph.hpp>
#include <vector>
#include <random>
#include <nigh/auto_strategy.hpp>
#include <string>
#include <iostream>

namespace mpl {
    template <class Scenario, class Scalar>
    class PRMPlanner {
    private:
        using State = typename Scenario::State;
        using Bound = typename Scenario::Bound;
        using Space = typename Scenario::Space;
        using Distance = typename Scenario::Distance;
        using RNG = std::mt19937_64;


        using Concurrency = unc::robotics::nigh::Concurrent;
        using NNStrategy = unc::robotics::nigh::auto_strategy_t<Space, Concurrency>;

    public:
        using Vertex_t = Vertex<State>;
        using Edge_t = Edge<typename Vertex_t::ID, Distance>;
        using Graph = UndirectedGraph<Vertex_t, Edge_t>;
        using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;

    private:

        struct KeyFn {
            const State& operator() (const Vertex_t& v) const {
                return v.state(); // TODO: Add scenario scale here.
            }
        };

        Scenario scenario;
        void* lambda; // TODO: extremely hacky, have to resolve the circular dependence here. Implement template fix later.
        Graph graph_;
        std::vector<void (*) (Vertex_t&, void*)> validSampleCallbacks;
        unc::robotics::nigh::Nigh<Vertex_t, Space, KeyFn, Concurrency, NNStrategy> nn;
        Distance maxDistance;
        uint64_t num_samples_;
        std::vector<Vertex_t> new_vertices;
        std::vector<Edge_t> new_edges;
        std::uint64_t lambda_id_; // To create vertex IDs that work across computers
        RNG rng;
        Scalar rPRM;

        void addRandomSample() {
            State s = scenario.randomSample(rng);
            addSample(s);
        }

    public:

        explicit PRMPlanner(Scenario scenario_, void* lambda_, std::uint64_t lambda_id)
                : scenario(scenario_),
                  lambda(lambda_),
                  maxDistance(scenario_.maxSteering()),
                  rPRM(scenario_.prmRadius()),
                  lambda_id_(lambda_id),
                  num_samples_(0)
        {}

//        explicit PRMPlanner(Scenario scenario_, void* lambda_, Graph existing_graph)
//                : scenario(scenario_),
//                  lambda(lambda_),
//                  graph_(existing_graph),
//                  maxDistance(scenario_.maxSteering()),
//                  rPRM(scenario_.prmRadius())
//        {}

        void addValidSampleCallback(void (*f)(Vertex_t&, void*)) {
            validSampleCallbacks.push_back(f);
        }

        void setrPRM(Scalar rPRM_) {
            rPRM = rPRM_;
        }

        Scalar getrPRM() {
            return rPRM;
        }

        void plan(int num_samples) {
            for(int i=0; i < num_samples; ++i) {
                //if (num_samples_ > 22) {
                //    return;
                //}
//                JI_LOG(INFO) << num_samples_;
                addRandomSample();
            }
        }

        void addGraph(Graph other) {
            graph_.merge(other);
        }

        void findPath() {
            // TODO
        }

        void clearVertices() {
            new_vertices.clear();
        }

        void clearEdges() {
            new_edges.clear();
        }

        const std::vector<Vertex_t>& getNewVertices() {
            return new_vertices;
        }

        const std::vector<Edge_t>& getNewEdges() {
            return new_edges;
        }

        const Graph& getGraph() const {
            return graph_;
        }

        void addSample(State& s, bool print_id=false) {
            if (!scenario.isValid(s)) return;
            std::string id = std::to_string(lambda_id_) + "_" + std::to_string(num_samples_);
            if (print_id) {
                JI_LOG(INFO) << "Vertex id " << id;
            }
            //else {
            //    JI_LOG(INFO) << "state " << s;
            //}
            Vertex_t v{id, s};
            std::vector<std::pair<Vertex_t, Scalar>> nbh;
            auto k = std::numeric_limits<std::size_t>::max();

            // add to graph
//            graph_.addVertex(v);
            // add to nearest neighbor structure
            new_vertices.push_back(v);
            nn.insert(v);
            // add valid edges
//            nn.nearest(nbh, Scenario::scale(v.state()), k, rPRM);
            nn.nearest(nbh, v.state(), k, rPRM);
            for(auto &[other, dist] : nbh) {
                // Other ones must be valid and in the graph by definition
                if (dist > 0 && scenario.isValid(v.state(), other.state())) {
                    Edge_t e{dist, v.id_, other.id_};
//                    graph_.addEdge(e);
//                    JI_LOG(INFO) << "u: " << v.id_ << " v: " << other.id_;
                    new_edges.push_back(std::move(e));
                }
            }
            for (auto fn : validSampleCallbacks) {
                fn(v, lambda);
            }
            ++num_samples_;
        }

        void updatePrmRadius(std::uint64_t num_samples, int dimension) {
            auto new_radius = scenario.prmRadius() * pow(log( num_samples) / (1.0 * num_samples), 1.0 / dimension);
            if (new_radius < rPRM) {
                JI_LOG(INFO) << "New rPRM is " << new_radius;
                rPRM = new_radius;
            }
        }

        void addExistingVertex(Vertex_t& v) {
            if (!scenario.isValid(v.state())) return;
            std::vector<std::pair<Vertex_t, Scalar>> nbh;
            auto k = std::numeric_limits<std::size_t>::max();

            // add to graph
//            graph_.addVertex(v);
            // add to nearest neighbor structure
            nn.insert(v);
            // add valid edges
//            nn.nearest(nbh, Scenario::scale(v.state()), k, rPRM);
            nn.nearest(nbh, v.state(), k, rPRM);
            for(auto &[other, dist] : nbh) {
                // Other ones must be valid and in the graph by definition
                if (scenario.isValid(v.state(), other.state())) {
                    Edge_t e{dist, v.id_, other.id_};
//                    graph_.addEdge(e);
                    new_edges.push_back(std::move(e));
                }
            }
            //for (auto fn : validSampleCallbacks) {
            //    fn(v, lambda);
            //}
        }
    };
}

#endif
