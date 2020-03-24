#ifndef MPL_PRM_PLANNER_HPP
#define MPL_PRM_PLANNER_HPP
#include <subspace.hpp>
#include <graph.hpp>
#include <vector>
#include <random>
#include <nigh/auto_strategy.hpp>

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

        struct Vertex { State state; };
        struct Edge { Distance distance; };

    public:
        using Graph = UndirectedGraph<Vertex, Edge>;
        using VertexWithRef = typename Graph::VertexWithRef;
        using Subspace_t = typename mpl::Subspace<Bound, State, Scalar>;

    private:

        struct KeyFn {
            const State& operator() (const VertexWithRef& v) const {
                return v.vertex.state; // TODO: is this copying unnecessarily. Add scenario scale here.
            }
        };

        Scenario scenario;
        void* lambda; // TODO: extremely hacky, have to resolve the circular dependence here
        Graph graph_;
        std::vector<void (*) (State, void*)> validSampleCallbacks;
        unc::robotics::nigh::Nigh<VertexWithRef, Space, KeyFn, Concurrency, NNStrategy> nn;
        Distance maxDistance;
        int num_samples_;
        RNG rng;
        Scalar rPRM;

        void addRandomSample() {
            State s = scenario.randomSample(rng);
            if (!scenario.isValid(s)) return;
            Vertex v{s};
            std::vector<std::pair<VertexWithRef, Scalar>> nbh;
            auto k = std::numeric_limits<std::size_t>::max();

            // add to graph
            auto v_with_ref = graph_.addVertex(v);
            // add to nearest neighbor structure
            nn.insert(v_with_ref);
            // add valid edges
            nn.nearest(nbh, Scenario::scale(v_with_ref.vertex.state), k, rPRM);
            for(auto &[other, dist] : nbh) {
                // Other ones must be valid and in the graph by definition
                if (scenario.isValid(v_with_ref.vertex.state, other.vertex.state)) {
                    Edge e{dist};
                    graph_.addEdge(v_with_ref.vertex_ref, other.vertex_ref, e);
                }
            }
            for (auto fn : validSampleCallbacks) {
                fn(s, lambda);
            }
        }

    public:

        explicit PRMPlanner(Scenario scenario_, void* lambda_)
                : scenario(scenario_),
                  lambda(lambda_),
                  maxDistance(scenario_.maxSteering()),
                  rPRM(scenario_.prmRadius())
        {}

        explicit PRMPlanner(Scenario scenario_, void* lambda_, Graph existing_graph)
                : scenario(scenario_),
                  lambda(lambda_),
                  graph_(existing_graph),
                  maxDistance(scenario_.maxSteering()),
                  rPRM(scenario_.prmRadius())
        {}

        void addValidSampleCallback(void (*f)(State, void*)) {
            validSampleCallbacks.push_back(f);
        }

        void plan(int num_samples) {
            for(int i=0; i < num_samples; ++i) {
                ++num_samples_;
                addRandomSample();

                // TODO: rPRM update in loop
            }
        }

        void addGraph(Graph other) {
            graph_.merge(other);
        }

        void findPath() {
            // TODO
        }

        decltype(auto) graph() {
            return graph_.graph();
        }

    };
}

#endif
