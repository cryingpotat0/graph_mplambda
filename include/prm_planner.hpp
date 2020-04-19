#ifndef MPL_PRM_PLANNER_HPP
#define MPL_PRM_PLANNER_HPP
#include <subspace.hpp>
#include <graph.hpp>
#include <vector>
#include <random>
#include <nigh/auto_strategy.hpp>
#include <string>
#include <iostream>
#include <time.h>


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
        std::vector<std::function<void(Vertex_t&)>> validSampleCallbacks;
        unc::robotics::nigh::Nigh<Vertex_t, Space, KeyFn, Concurrency, NNStrategy> nn;
        uint64_t num_samples_;
        std::vector<Vertex_t> new_vertices;
        std::vector<Edge_t> new_edges;
        std::uint16_t id_prefix_; // To create vertex IDs that work across computers
        RNG rng;
        Scalar rPRM;


    public:

        explicit PRMPlanner(Scenario scenario_, std::uint16_t id_prefix)
                : scenario(scenario_),
                  rPRM(scenario_.prmRadius()),
                  id_prefix_(id_prefix),
                  num_samples_(0),
                  rng(time(NULL))
        {}

        void addValidSampleCallback(std::function<void(Vertex_t&)> f) {
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

        void updatePrmRadius(std::uint64_t num_samples, int dimension) {
            auto new_radius = scenario.prmRadius() * pow(log( num_samples) / (1.0 * num_samples), 1.0 / dimension);
            if (new_radius > 0 && new_radius < rPRM) {
                JI_LOG(INFO) << "New rPRM is " << new_radius;
                rPRM = new_radius;
            }
        }

        void addRandomSample() {
            State s = scenario.randomSample(rng);
            addSample(s);
        }

        void addSample(State& s, bool print_id=false) {
            if (!scenario.isValid(s)) return;
            auto id = std::make_pair(id_prefix_, num_samples_);
            if (print_id) {
                JI_LOG(INFO) << "Vertex id " << id;
            }
            Vertex_t v{id, s};
            new_vertices.push_back(v);


            // add valid edges
            connectVertex(v);
            //std::vector<std::pair<Vertex_t, Scalar>> nbh;
            //auto k = std::numeric_limits<std::size_t>::max();
            //nn.nearest(nbh, v.state(), k, rPRM);
            //for(auto &[other, dist] : nbh) {
            //    // Other ones must be valid and in the graph by definition
            //    if (scenario.isValid(v.state(), other.state())) {
            //        Edge_t e{dist, v.id_, other.id_};
            //        new_edges.push_back(std::move(e));
            //    }
            //}
            // add to nearest neighbor structure
            nn.insert(v);
            for (auto fn : validSampleCallbacks) {
                fn(v);
            }
            ++num_samples_;
        }

        //void addExistingVertex(Vertex_t& v) {
        //    if (!scenario.isValid(v.state())) return;
        //    connectVertex(v);

        //    //std::vector<std::pair<Vertex_t, Scalar>> nbh;
        //    //auto k = std::numeric_limits<std::size_t>::max();
        //    //nn.nearest(nbh, v.state(), k, rPRM);
        //    //for(auto &[other, dist] : nbh) {
        //    //    // Other ones must be valid and in the graph by definition
        //    //    if (scenario.isValid(v.state(), other.state())) {
        //    //        Edge_t e{dist, v.id_, other.id_};
        //    //        new_edges.push_back(std::move(e));
        //    //    }
        //    //}
        //}
        void addExistingVertex(Vertex_t& v) {
            nn.insert(v);
        }
        
        void connectVertex(Vertex_t& v) {
            std::vector<std::pair<Vertex_t, Scalar>> nbh;
            auto k = std::numeric_limits<std::size_t>::max();
            nn.nearest(nbh, v.state(), k, rPRM);
            for(auto &[other, dist] : nbh) {
                // Other ones must be valid and in the graph by definition
                if (scenario.isValid(v.state(), other.state())) {
                    Edge_t e{dist, v.id_, other.id_};
                    new_edges.push_back(std::move(e));
                }
            }
        }
    };
}

#endif
