///
// Created by Raghav Anand on 2020-03-19.
//



#ifndef MPLAMBDA_GRAPH_HPP
#define MPLAMBDA_GRAPH_HPP

#include <iterator>
#include <vector>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <queue>
#include <util.hpp>
#include <sstream>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>


namespace std {
    template <class T1, class T2>
        struct hash<std::pair<T1, T2>> {
            size_t operator()(const std::pair<T1, T2>& pair) const noexcept {
                return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
            }
        };
}

namespace mpl {
    //template<class T1, class T2>
    //struct pair_hash {
    //    std::size_t operator() (const std::pair<T1, T2> &pair) const
    //    {
    //        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    //    }
    //};


    template <class State>
        struct Vertex {
            //using ID = std::string;
            using ID = std::pair<std::uint16_t, std::uint32_t>; // <lambda_id, vertex_id>
            ID id_;
            State state_;

            const ID& id() const { return id_; }
            ID id() { return id_; }

            const State& state() const { return state_; }
            State state() { return state_; }

            inline void serialize(std::ofstream &file) {
                file << "id=" << id_.first << "," << id_.second << ";" 
                    << "state=" << state_.format(mpl::util::FullPrecisionCommaInitFormat) 
                    << "\n"; 
            }

            inline static Vertex deserialize(std::string& line) {
                std::vector<std::string> vec_split;
                auto pos = line.find(";state=");
                auto vec_portion = line.substr(pos + 7);
                mpl::util::string_split(vec_split, vec_portion, ",");
                State q;
                assert(q.size() == vec_split.size());
                for (int i=0; i < vec_split.size(); ++i) {
                    q[i] = std::stod(vec_split[i]);
                }

                std::string id_portion = line.substr(3, pos);
                ID curr_id = deserializeID(id_portion);
                return Vertex{curr_id, q};
            }

            inline static ID deserializeID(std::string &id_str) {
                std::vector<std::string> id_split;
                mpl::util::string_split(id_split, id_str, ",");
                return std::make_pair(std::stoul(id_split[0]), std::stoul(id_split[1]));
            }
        };

    template<class VertexID, class Distance>
        struct Edge {
            using Distance_t = Distance;
            Distance distance_;
            VertexID u_;
            VertexID v_;

            const VertexID& u() const { return u_; }
            const VertexID& v() const { return v_; }

            const Distance& distance() const { return distance_; }
            Distance distance() { return distance_; }

            inline void serialize(std::ofstream &file) {
                file << "u=" << u_
                    << ";v=" << v_
                    << ";distance=" << distance_
                    << "\n";
            }

            inline static Edge deserialize(std::string &v) {
                std::vector<std::string> results;
                mpl::util::string_split(results, v, ";");
                auto u_val = results[0].substr(2);
                auto v_val = results[1].substr(2);
                auto dist_val = std::stod(results[2].substr(9));
                return Edge{dist_val, deserializeID(u_val), deserializeID(v_val)};
            }

            inline static VertexID deserializeID(std::string &id_str) { // Maybe need a separate class for ID, duplicated method for now
                std::vector<std::string> id_split;
                mpl::util::string_split(id_split, id_str, ",");
                return std::make_pair(std::stoul(id_split[0]), std::stoul(id_split[1]));
            }
        };

    template <class State>
        struct TimedVertex {
            //using ID = std::string;
            using ID = std::pair<std::uint16_t, std::uint32_t>; // <lambda_id, vertex_id>
            ID id_;
            State state_;
            std::uint64_t timestamp_millis_;

            const ID& id() const { return id_; }
            ID id() { return id_; }

            const State& state() const { return state_; }
            State state() { return state_; }

            const std::uint64_t& timestamp_millis() const { return timestamp_millis_; }
            std::uint64_t& timestamp_millis() { return timestamp_millis_; }

            inline void serialize(std::ofstream &file) {
                file << "id=" << id_.first << "," << id_.second
                     << ";state=" << mpl::util::state_format(state_) 
                     << ";timestamp=" << timestamp_millis_
                     << "\n"; 
            }

            inline static TimedVertex deserialize(std::string& line) {
                std::vector<std::string> results;
                mpl::util::string_split(results, line, ";");

                std::vector<std::string> vec_split;
                auto vec_portion = results[1].substr(6);
                mpl::util::string_split(vec_split, vec_portion, ",");
                State q;
                assert(q.size() == vec_split.size());
                for (int i=0; i < vec_split.size(); ++i) {
                    q[i] = std::stod(vec_split[i]);
                }

                auto timestamp_millis_val = std::stoull(results[2].substr(10));

                std::string id_portion = results[0].substr(3);
                ID curr_id = deserializeID(id_portion);
                return TimedVertex{curr_id, q, timestamp_millis_val};
            }

            inline static ID deserializeID(std::string &id_str) {
                std::vector<std::string> id_split;
                mpl::util::string_split(id_split, id_str, ",");
                return std::make_pair(std::stoul(id_split[0]), std::stoul(id_split[1]));
            }
        };

    // Specialization for SE3
    template <>
        struct TimedVertex< std::tuple<Eigen::Quaternion<double, 0>, Eigen::Matrix<double, 3, 1, 0, 3, 1>>>{
            //using ID = std::string;
            using ID = std::pair<std::uint16_t, std::uint32_t>; // <lambda_id, vertex_id>
            using State = std::tuple<Eigen::Quaternion<double, 0>, Eigen::Matrix<double, 3, 1, 0, 3, 1>>;
            ID id_;
            State state_;
            std::uint64_t timestamp_millis_;

            const ID& id() const { return id_; }
            ID id() { return id_; }

            const State& state() const { return state_; }
            State state() { return state_; }

            const std::uint64_t& timestamp_millis() const { return timestamp_millis_; }
            std::uint64_t& timestamp_millis() { return timestamp_millis_; }

            inline void serialize(std::ofstream &file) {
                file << "id=" << id_.first << "," << id_.second
                     << ";state=" << mpl::util::state_format(state_) 
                     << ";timestamp=" << timestamp_millis_
                     << "\n"; 
            }

            inline static TimedVertex deserialize(std::string& line) {
                std::vector<std::string> results;
                mpl::util::string_split(results, line, ";");

                std::vector<std::string> vec_split;
                auto vec_portion = results[1].substr(6);
                mpl::util::string_split(vec_split, vec_portion, ",");
                State q;
                auto& [quat, pos] = q;
                assert(7 == vec_split.size());
                quat.w() = std::stod(vec_split[0]);
                quat.x() = std::stod(vec_split[1]);
                quat.y() = std::stod(vec_split[2]);
                quat.z() = std::stod(vec_split[3]);
                for (int i=4; i < 7; ++i) {
                    pos[i] = std::stod(vec_split[i]);
                }

                auto timestamp_millis_val = std::stoull(results[2].substr(10));

                std::string id_portion = results[0].substr(3);
                ID curr_id = deserializeID(id_portion);
                return TimedVertex{curr_id, q, timestamp_millis_val};
            }

            inline static ID deserializeID(std::string &id_str) {
                std::vector<std::string> id_split;
                mpl::util::string_split(id_split, id_str, ",");
                return std::make_pair(std::stoul(id_split[0]), std::stoul(id_split[1]));
            }
        };

    template<class VertexID, class Distance>
        struct TimedEdge {
            using Distance_t = Distance;
            Distance distance_;
            VertexID u_;
            VertexID v_;
            std::uint64_t timestamp_millis_;

            const VertexID& u() const { return u_; }
            const VertexID& v() const { return v_; }

            const Distance& distance() const { return distance_; }
            Distance distance() { return distance_; }

            const std::uint64_t& timestamp_millis() const { return timestamp_millis_; }
            std::uint64_t& timestamp_millis() { return timestamp_millis_; }

            inline void serialize(std::ofstream &file) {
                file << "u=" << u_
                    << ";v=" << v_
                    << ";distance=" << distance_
                    << ";timestamp=" << timestamp_millis_
                    << "\n";
            }

            inline static TimedEdge deserialize(std::string &v) {
                std::vector<std::string> results;
                mpl::util::string_split(results, v, ";");
                auto u_val = results[0].substr(2);
                auto v_val = results[1].substr(2);
                auto dist_val = std::stod(results[2].substr(9));
                auto timestamp_millis_val = std::stoull(results[3].substr(10));
                return TimedEdge{dist_val, deserializeID(u_val), deserializeID(v_val), timestamp_millis_val};
            }

            inline static VertexID deserializeID(std::string &id_str) { // Maybe need a separate class for ID, duplicated method for now
                std::vector<std::string> id_split;
                mpl::util::string_split(id_split, id_str, ",");
                return std::make_pair(std::stoul(id_split[0]), std::stoul(id_split[1]));
            }
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
                //std::unordered_map<EdgeID, Edge, pair_hash<VertexID, VertexID>> edge_properties;
                std::unordered_map<EdgeID, Edge> edge_properties;

            public:
                UndirectedGraph() = default;
                //        explicit UndirectedGraph(UndirectedGraph &other) { merge(other); }

                const std::unordered_map<VertexID, std::unordered_set<VertexID>>& getAdjacencyList() const {
                    return adjacency_list;
                }

                const std::unordered_map<VertexID, Vertex>& getVertices() const {
                    return vertex_properties;
                }

                const int edgeCount() {
                    return edge_properties.size();
                }

                const int vertexCount() {
                    return vertex_properties.size();
                }

                void merge(UndirectedGraph &other) {
                    for (auto& [v_id, u_ids] : other.getAdjacencyList()) {
                        adjacency_list[v_id].merge(u_ids);
                    }
                }

                void addVertex(Vertex v) {
                    if (vertex_properties.find(v.id()) == vertex_properties.end()) // Only add if it doesn't exist already. No overwriting.
                        vertex_properties[v.id()] = v;
                }

                const Vertex& getVertex(const VertexID& id) const {
                    return vertex_properties.at(id);
                }

                const Edge& getEdge(const VertexID& u, const VertexID& v) const {
                    EdgeID forward(u, v);
                    return edge_properties.at(forward);
                }
                
                void serialize(std::ofstream &file) {
                    std::map<VertexID, Vertex> vertex_properties_sorted(vertex_properties.begin(), vertex_properties.end());
                    std::map<EdgeID, Edge> edge_properties_sorted(edge_properties.begin(), edge_properties.end());
                    file << "vertices\n";
                    for (auto& [id, vertex]: vertex_properties_sorted) {
                        vertex.serialize(file);
                    }
                    file << "edges\n";
                    for (auto& [id, edge]: edge_properties_sorted) {
                        edge.serialize(file);
                    }
                    //file.close();
                }
                
                static UndirectedGraph deserialize(std::ifstream &file) {
                    UndirectedGraph g;
                    bool vertices{false}, edges{false};
                    for (std::string line; std::getline(file, line); ) {
                        if (line == "vertices") {
                            vertices = true;
                            continue;
                        }
                        if (vertices) {
                            if (line == "edges") {
                                vertices = false;
                                edges = true;
                                continue;
                            }
                            g.addVertex(Vertex::deserialize(line));
                        }
                        if (edges) {
                            g.addEdge(Edge::deserialize(line));
                        }
                    }
                    //file.close();
                    return g;
                }

                void addEdge(Edge e) {
                    // TODO: checks for existence of vertices
                    auto u = e.u();
                    auto v = e.v();
                    EdgeID forward(u, v);
                    //            JI_LOG(TRACE) << "Adding u: " << u << " v: " << v;
                    edge_properties[forward] = e;
                    EdgeID backward(v, u);
                    //            JI_LOG(TRACE) << "Adding u: " << v << " v: " << u;
                    Edge backward_e = e;
                    e.u_ = v;
                    e.v_ = u;
                    edge_properties[backward] = e;
                    adjacency_list[v].insert(u);
                    adjacency_list[u].insert(v);
                }

                std::pair<bool, std::vector<VertexID>> djikstras(const VertexID& start, const VertexID& end) const {
                    using Distance = typename Edge::Distance_t;
                    using DistVertexPair = typename std::pair<Distance, VertexID>;
                    std::priority_queue<DistVertexPair, std::vector<DistVertexPair>, std::greater<DistVertexPair>> pq;
                    std::unordered_map<VertexID, Distance> dists;
                    std::unordered_map<VertexID, VertexID> prev;
                    pq.push(std::make_pair(0, start));
                    //            JI_LOG(INFO) << "Num edges " << edge_properties.size();
                    while (!pq.empty()) {
                        auto [curr_dist, u] = pq.top();
                        pq.pop();
                        if (u == end) break;
                        //                JI_LOG(TRACE) << "Len of adjacency list for " << u << " is " << std::to_string(adjacency_list[u].size());
                        auto outgoing_edges = adjacency_list.find(u);
                        if (outgoing_edges == adjacency_list.end()) continue;
                        for (auto& v : outgoing_edges->second) {
                            //                    JI_LOG(TRACE) << "u: " << u << " v: " << v;
                            auto edge = getEdge(u, v);
                            if (dists.find(v) == dists.end()) dists[v] = std::numeric_limits<Distance>::max();

                            if (dists[v] > dists[u] + edge.distance()) {
                                dists[v] = dists[u] + edge.distance();
                                prev[v] = u;
                                pq.push(std::make_pair(dists[v], v));
                            }
                        }
                    }
                    if (prev.find(end) == prev.end()) {
                        return std::make_pair(false, std::vector<VertexID>());
                    }
                    auto curr = end;
                    std::vector<VertexID> path;
                    while (curr != start) {
                        path.push_back(curr);
                        curr = prev[curr];
                    }
                    path.push_back(start);
                    std::reverse(path.begin(), path.end());
                    return std::make_pair(true, path);
                }

                template <class GoalFn> // TODO: replace this with a lambda instead
                    std::pair<bool, std::vector<VertexID>> djikstras(const VertexID& start, const GoalFn&& goalFn) const {
                        using Distance = typename Edge::Distance_t;
                        using DistVertexPair = typename std::pair<Distance, VertexID>;
                        std::priority_queue<DistVertexPair, std::vector<DistVertexPair>, std::greater<DistVertexPair>> pq;
                        std::unordered_map<VertexID, Distance> dists;
                        std::unordered_map<VertexID, VertexID> prev;
                        pq.push(std::make_pair(0, start));
                        VertexID end;
                        bool goalFound{false};
                        while (!pq.empty()) {
                            auto [curr_dist, u] = pq.top();
                            pq.pop();
                            if (goalFn(getVertex(u))) {
                                end = u;
                                goalFound = true;
                                break;
                            }
                            auto outgoing_edges = adjacency_list.find(u);
                            if (outgoing_edges == adjacency_list.end()) continue;
                            for (auto& v : outgoing_edges->second) {
                                auto edge = getEdge(u, v);
                                if (dists.find(v) == dists.end()) dists[v] = std::numeric_limits<Distance>::max();
                                if (dists[v] > dists[u] + edge.distance()) {
                                    dists[v] = dists[u] + edge.distance();
                                    prev[v] = u;
                                    pq.push(std::make_pair(dists[v], v));
                                }
                            }
                        }
                        if (!goalFound) {
                            return std::make_pair(false, std::vector<VertexID>());
                        }
                        auto curr = end;
                        std::vector<VertexID> path;
                        while (curr != start) {
                            path.push_back(curr);
                            curr = prev[curr];
                        }
                        path.push_back(start);
                        std::reverse(path.begin(), path.end());
                        return std::make_pair(true, path);
                    }

        };

}


#endif //MPLAMBDA_GRAPH_HPP
