//
// Created by Raghav Anand on 2020-03-19.
//



#ifndef MPLAMBDA_GRAPH_HPP
#define MPLAMBDA_GRAPH_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <iterator>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace mpl {
    template<class T1, class T2>
    struct pair_hash {
        std::size_t operator() (const std::pair<T1, T2> &pair) const
        {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };


    template <class State>
    struct Vertex {
        using ID = std::string;
        ID id_;
        State state_;

        const ID& id() const { return id_; }
        ID id() { return id_; }

        const State& state() const { return state_; }
        State state() { return state_; }

//            std::string serialize() {
//
//                std::ostringstream oStream;
//                oStream << state.format(mpl::util::CommaInitFormat); // ends with a semicolon
//                oStream << "id=" << id_;
//                return oStream.str();
//            }
//
//            static Vertex deserialize(std::string v) {
//                std::vector<std::string> vec_split;
//                auto pos = v.find(";id=");
//                ID curr_id = v.substr(pos + 4);
//                auto vec_portion = v.substr(0, pos);
//                boost::split(vec_split, vec_portion, [](char c){return c==',';});
//                State q;
//                assert(q.size() == vec_split.size());
//                for (int i=0; i < vec_split.size(); ++i) {
//                    q[i] = std::stod(vec_split[i]);
//                }
//                return Vertex{curr_id, q};
//            }
    };

    template<class VertexID, class Distance>
    struct Edge {
        Distance distance_;
        VertexID u_;
        VertexID v_;

        const VertexID& u() const { return u_; }
        const VertexID& v() const { return v_; }

        const Distance& distance() const { return distance_; }
        Distance distance() { return distance_; }

//        std::string serialize() {
//            std::ostringstream oStream;
//            oStream << "u=" << u_.id() << ";v=" << v_.id() << ";distance=" << distance;
//            return oStream.str();
//        }
//
//        static Edge deserialize(std::string v) {
//            std::vector<std::string> results;
//            boost::split(results, v, [](char c){return c==';';});
//            auto u_val = results[0].substr(2);
//            auto v_val = results[1].substr(2);
//            auto dist_val = std::stod(results[2].substr(9));
//            return Edge{dist_val, u_val, v_val};
//        }
    };


    template <class Vertex, class Edge>
    class UndirectedGraph {
        // Custom class for unique vertex IDs that can be strings
        // External user must manage Vertex IDs
        // Vertex must have the Vertex::ID type and the vertex.id() method
        // Edge must implement edge.u() and edge.v() which return the endpoints IDs of vertices
    private:
        using VertexID = typename Vertex::ID;
        std::unordered_map<VertexID, std::unordered_set<VertexID>> adjacency_list;
        std::unordered_map<VertexID, Vertex> vertex_properties;
        typedef std::pair<VertexID, VertexID> EdgeID;
        std::unordered_map<EdgeID, Edge, pair_hash<VertexID, VertexID>> edge_properties;

    public:
        UndirectedGraph() = default;
//        explicit UndirectedGraph(UndirectedGraph &other) { merge(other); }

        const std::unordered_map<VertexID, std::unordered_set<VertexID>>& getAdjacencyList() const {
            return adjacency_list;
        }

        void merge(UndirectedGraph &other) {
            for (auto& [v_id, u_ids] : other.getAdjacencyList()) {
                adjacency_list[v_id].merge(u_ids);
            }
        }

        void addVertex(Vertex v) {
            vertex_properties[v.id()] = v;
        }

        const Vertex& getVertex(const VertexID& id) const {
            return vertex_properties.at(id);
        }

        void addEdge(Edge e) {
            // TODO: checks for existence of vertices
            auto u = e.u();
            auto v = e.v();
            EdgeID forward(u, v);
            edge_properties[forward] = e;
            EdgeID backward(v, u);
            edge_properties[backward] = e;
            adjacency_list[v].insert(u);
            adjacency_list[u].insert(v);
        }


//        struct Vertex {
//            State state;
//            std::string serialize() {
//
//            }
//            static Vertex deserialize(std::string v) {
//
//            }
//        };
//
//        struct Edge {
//            Distance distance;
//            std::string serialize() {
//
//            }
//            static Edge deserialize(std::string v) {
//
//            }
//        };

//        using graph_t = typename boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, Vertex, Edge>;
//        using vertex_t = typename boost::graph_traits<graph_t>::vertex_descriptor;
//        using edge_t = typename boost::graph_traits<graph_t>::edge_descriptor;
//
//        struct VertexWithRef {
//            vertex_t vertex_ref;
//            Vertex vertex;
//        };
//    private:
//        graph_t graph_;
//    public:
//        UndirectedGraph() = default;
//        UndirectedGraph(UndirectedGraph<Vertex, Edge> &other) { merge(other); }
//
//        void merge(UndirectedGraph<Vertex, Edge> &other) {
//            // Merge another graph into the current one
//        };
//
//        VertexWithRef addVertex(Vertex &v) {
//            auto vertex_ref = boost::add_vertex(v, graph_);
//            return VertexWithRef{vertex_ref, v};
//        }
//
//        void addEdge(vertex_t &v1, vertex_t &v2, Edge &e) {
//            boost::add_edge(v1, v2, e, graph_);
//        }
//
//        graph_t graph() {
//            return graph_;
//        }

    };

}


#endif //MPLAMBDA_GRAPH_HPP
