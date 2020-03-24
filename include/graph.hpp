//
// Created by Raghav Anand on 2020-03-19.
//



#ifndef MPLAMBDA_GRAPH_HPP
#define MPLAMBDA_GRAPH_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <iterator>

namespace mpl {


    template <class Vertex, class Edge>
    class UndirectedGraph {
    public:
        using graph_t = typename boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, Vertex, Edge>;
        using vertex_t = typename boost::graph_traits<graph_t>::vertex_descriptor;
        using edge_t = typename boost::graph_traits<graph_t>::edge_descriptor;
        struct VertexWithRef {
            vertex_t vertex_ref;
            Vertex vertex;
        };
    private:
        graph_t graph_;
    public:
        UndirectedGraph() = default;
        UndirectedGraph(UndirectedGraph<Vertex, Edge> &other) { merge(other); }

        void merge(UndirectedGraph<Vertex, Edge> &other) {
            // Merge another graph into the current one
        };

        VertexWithRef addVertex(Vertex &v) {
            auto vertex_ref = boost::add_vertex(v, graph_);
            return VertexWithRef{vertex_ref, v};
        }

        void addEdge(vertex_t &v1, vertex_t &v2, Edge &e) {
            boost::add_edge(v1, v2, e, graph_);
        }

        graph_t graph() {
            return graph_;
        }

    };

}

#endif //MPLAMBDA_GRAPH_HPP
