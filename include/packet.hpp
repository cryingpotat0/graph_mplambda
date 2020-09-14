#pragma once
#ifndef MPL_PACKET_HPP
#define MPL_PACKET_HPP

#include "buffer.hpp"
#include <jilog.hpp>
#include <iostream>

namespace mpl::packet {

    using Type = std::uint32_t;
    using Size = std::uint32_t;

    // hexdump -n 4 -e '"0x" 1 "%08x" "\n"' /dev/urandom
    //    static constexpr Type PROBLEM = 0x8179e3f1;
    static constexpr Type HELLO = 0x3864caca;
    //    static constexpr Type PATH_SE3 = 0xa9db6e7d;
    //    static constexpr Type PATH_RVF = 0xb11b0c45;
    //    static constexpr Type PATH_RVD = PATH_RVF + 0x100;
    static constexpr Type DONE = 0x6672e31a;
    static constexpr Type VERTICES = 0xf112a36b;
    static constexpr Type EDGES = 0xe163990c;
    static constexpr Type NUM_SAMPLES = 0xd4e233c4;
    static constexpr Type RANDOM_SEED_WORK = 0x48cc060c;


    static constexpr std::size_t MAX_PACKET_SIZE = 1024*1024;

    static constexpr std::uint32_t ALGORITHM_RRT = 1;
    static constexpr std::uint32_t ALGORITHM_CFOREST = 2;

    class protocol_error : public std::runtime_error {
        public:
            protocol_error(const std::string& msg)
                : std::runtime_error(msg)
            {
            }
    };

    class Hello {
        std::uint64_t id_;
        std::uint64_t start_time_;

        public:
        static std::string name() {
            return "Hello";
        }


        explicit Hello(std::uint64_t id, std::uint64_t start_time)
                : id_(id), start_time_(start_time)

        {
        }

        explicit Hello(Type type, BufferView buf)
                : id_(buf.get<std::uint64_t>()), start_time_(buf.get<std::uint64_t>())
        {
        }

        std::uint64_t id() const {
            return id_;
        }

        std::uint64_t start_time() const {
            return start_time_;
        }

        operator Buffer () const {
            Size size = 24;
            Buffer buf{size};
            buf.put(HELLO);
            buf.put(size);
            buf.put(id_);
            buf.put(start_time_);
            buf.flip();
            return buf;
        }
    };

    class NumSamples {
        std::uint64_t num_samples_;

        public:
        static std::string name() {
            return "NumSamples";
        }

        explicit NumSamples(std::uint64_t num_samples)
            : num_samples_(num_samples)
        {
        }

        explicit NumSamples(Type type, BufferView buf)
            : num_samples_(buf.get<std::uint64_t>())
        {
            JI_LOG(INFO) << "NUM_SAMPLES received";
        }

        std::uint64_t num_samples() const {
            return num_samples_;
        }

        operator Buffer () const {
            Size size = 16;
            Buffer buf{size};
            buf.put(NUM_SAMPLES);
            buf.put(size);
            buf.put(num_samples_);
            buf.flip();
            return buf;
        }
    };

    class Done {
        std::uint64_t id_;

        public:
        static std::string name() {
            return "Done";
        }

        explicit Done(std::uint64_t id)
            : id_(id)
        {
        }

        explicit Done(Type type, BufferView buf)
            : id_(buf.get<std::uint64_t>())
        {
        }

        std::uint64_t id() const {
            return id_;
        }

        operator Buffer () const {
            Size size = 16;
            Buffer buf{size};
            buf.put(DONE);
            buf.put(size);
            buf.put(id_);
            buf.flip();
            return buf;
        }
    };

    class RandomSeedWork {
        /* Start and end vertex ids to assign work to lambdas */

        std::uint64_t start_id_;
        std::uint64_t end_id_;

        public:
        static std::string name() {
            return "RandomSeedWork";
        }

        explicit RandomSeedWork(std::uint64_t start_id, std::uint64_t end_id) 
            : start_id_(start_id), end_id_(end_id) {}


        inline RandomSeedWork(Type type, BufferView buf) 
            : start_id_(buf.get<std::uint64_t>()), 
            end_id_(buf.get<std::uint64_t>()) {}

        std::uint64_t start_id() const {
            return start_id_;
        }

        std::uint64_t end_id() const {
            return end_id_;
        }

        operator Buffer () const {
            Size size = buffer_size_v<Type> + buffer_size_v<Size> + 
                2 * buffer_size_v<std::uint64_t>;
            Buffer buf{size};
            buf.put(RANDOM_SEED_WORK);
            buf.put(size);
            buf.put(start_id_);
            buf.put(end_id_);
            buf.flip();
            return buf;
        }
    };

    template <class Vertex, class State>
        class Vertices {
            private:
                // Assume each vertex has ID of form {lambdaId}_{vertexId} where both IDs are std::uint64_t
                static constexpr std::size_t vertexSize_ = buffer_size_v<State> + buffer_size_v<std::uint16_t> + buffer_size_v<std::uint32_t>;
                std::vector<Vertex> vertices_;
                bool destination_; // 0 means append to coordinator graph, 1 means send to lambdaId
                std::uint16_t destination_lambdaId_;

                //static inline std::pair<std::uint64_t, std::uint64_t> stringIdToNumerics(const std::string &id) {
                //    int pos = id.find("_");
                //    std::uint64_t lambdaId = std::stoull(id.substr(0, pos));
                //    std::uint64_t vertexId = std::stoull(id.substr(pos + 1));
                //    return std::make_pair(lambdaId, vertexId);
                //}

                //static inline std::string numericalIdToString(const std::uint64_t &lambdaId, const std::uint64_t &vertexId) {
                //    std::ostringstream oStream;
                //    oStream << lambdaId << "_" << vertexId;
                //    return oStream.str();
                //}

            public:

                static std::string name() {
                    return "Vertices";
                }

                explicit Vertices(bool destination, std::uint64_t destination_lambdaId, std::vector<Vertex>&& vertices)
                    : destination_(destination)
                      , destination_lambdaId_(destination_lambdaId)
                      , vertices_(std::move(vertices))
            {
            }

                inline Vertices(Type type, BufferView buf)
                    : destination_(buf.get<bool>())
                      , destination_lambdaId_(buf.get<std::uint16_t>())
            {
                if (buf.remaining() % vertexSize_ != 0)
                    throw protocol_error("invalid vertices packet size: " + std::to_string(buf.remaining()));

                std::size_t n = buf.remaining() / vertexSize_;
                vertices_.reserve(n);
                while (vertices_.size() < n) {
                    State state = buf.get<State>();
                    auto lambdaId = buf.get<std::uint16_t>();
                    auto vertexId = buf.get<std::uint32_t>();
                    vertices_.emplace_back(Vertex{std::make_pair(lambdaId, vertexId), state});
                }
            }

                inline operator Buffer () const {
                    Size size = buffer_size_v<Type> + buffer_size_v<Size>
                        + buffer_size_v<bool> // destination
                        + buffer_size_v<std::uint16_t> // destinationLambdaId
                        + vertexSize_ * vertices_.size();
                    Buffer buf{size};
                    buf.put(VERTICES);
                    buf.put(size);
                    buf.put(destination_);
                    buf.put(destination_lambdaId_);
                    for (const Vertex& v : vertices_) {
                        buf.put(v.state());
                        auto& [lambdaId, vertexId] = v.id();
                        buf.put(lambdaId);
                        buf.put(vertexId);
                    }
                    buf.flip();
                    return buf;
                }

                bool destination() const {
                    return destination_;
                };

                std::uint16_t destinationLambdaId() const {
                    return destination_lambdaId_;
                };

                const std::vector<Vertex>& vertices() const & {
                    return vertices_;
                }

                std::vector<Vertex>&& vertices() && {
                    return std::move(vertices_);
                }

        };

    template <class Edge, class Distance>
        class Edges {
            // Edges always go to location they are sent to, no rerouting needed coordinator
            private:
                std::vector<Edge> edges_;

                //static inline std::pair<std::uint64_t, std::uint64_t> stringIdToNumerics(const std::string &id) {
                //    int pos = id.find("_");
                //    std::uint64_t lambdaId = std::stoull(id.substr(0, pos));
                //    std::uint64_t vertexId = std::stoull(id.substr(pos + 1));
                //    return std::make_pair(lambdaId, vertexId);
                //}

                //static inline std::string numericalIdToString(const std::uint64_t &lambdaId, const std::uint64_t &vertexId) {
                //    std::ostringstream oStream;
                //    oStream << lambdaId << "_" << vertexId;
                //    return oStream.str();
                //}
            public:
                static constexpr std::size_t edgeSize_ = buffer_size_v<Distance> + 2 * buffer_size_v<std::uint16_t> + 2 * buffer_size_v<std::uint32_t>;
                static constexpr std::size_t edgeHeaderSize_ = buffer_size_v<Type> + buffer_size_v<Size>;
                static std::string name() {
                    return "Edges";
                }

                explicit Edges(std::vector<Edge>&& edges)
                    : edges_(std::move(edges))
                {
                }

                inline Edges(Type type, BufferView buf)
                {
                    if (buf.remaining() % edgeSize_ != 0)
                        throw protocol_error("invalid vertices packet size: " + std::to_string(buf.remaining()));

                    std::size_t n = buf.remaining() / edgeSize_;
                    edges_.reserve(n);
                    while (edges_.size() < n) {
                        Distance distance = buf.get<Distance>();
                        auto lambdaId = buf.get<std::uint16_t>();
                        auto vertexId = buf.get<std::uint32_t>();
                        auto u_id = std::make_pair(lambdaId, vertexId);
                        lambdaId = buf.get<std::uint16_t>();
                        vertexId = buf.get<std::uint32_t>();
                        auto v_id = std::make_pair(lambdaId, vertexId);
                        edges_.emplace_back(Edge{distance, u_id, v_id});
                    }
                }

                inline operator Buffer () const {
                    Size size = buffer_size_v<Type> + buffer_size_v<Size>
                        + edgeSize_ * edges_.size();
                    Buffer buf{size};
                    buf.put(EDGES);
                    buf.put(size);
                    std::string id;
                    for (const Edge& e : edges_) {
                        buf.put(e.distance());
                        auto& [u_lambdaId, u_vertexId] = e.u();
                        buf.put(u_lambdaId);
                        buf.put(u_vertexId);
                        auto& [v_lambdaId, v_vertexId] = e.v();
                        buf.put(v_lambdaId);
                        buf.put(v_vertexId);
                    }
                    buf.flip();
                    return buf;
                }


                const std::vector<Edge>& edges() const & {
                    return edges_;
                }

                std::vector<Edge>&& edges() && {
                    return std::move(edges_);
                }
        };

    template <class Packet>
        struct is_vertices: std::false_type {};

    template <class Vertex, class State>
        struct is_vertices<Vertices<Vertex, State>> : std::true_type {};

    template <class Packet>
        struct is_edges: std::false_type {};

    template <class Edge, class Distance>
        struct is_edges<Edges<Edge, Distance>> : std::true_type {};

    template <class Packet>
        struct is_num_samples: std::false_type {};

    template<>
        struct is_num_samples<NumSamples> : std::true_type {};

    template <class Packet>
        struct is_random_seed_work: std::false_type {};

    template<>
        struct is_random_seed_work<RandomSeedWork> : std::true_type {};

    template <class Edge, class Distance, class Vertex, class State, class Fn>
        std::size_t parse(Buffer& buf, Fn fn) {
            static constexpr auto head = buffer_size_v<Type> + buffer_size_v<Size>;

            if (buf.remaining() < head)
                return 8; // head - buf.remaining();

            // bounds checking
            char *start = buf.begin();

            Type type = buf.peek<Type>(0);
            Size size = buf.peek<Size>(buffer_size_v<Type>);

            if (size > MAX_PACKET_SIZE) {
                JI_LOG(INFO) << MAX_PACKET_SIZE << " " << Edges<Edge, Distance>::edgeSize_;
                throw protocol_error("maximum packet size exceeded: " + std::to_string(size) + " " + std::to_string(type == EDGES));
            }

            if (buf.remaining() < size) {
                //JI_LOG(TRACE) << "short packet recv, have " << buf.remaining() << ", need " << size;
                return size; // size - buf.remaining();
            }

            buf += head;
            size -= head;

            switch (type) {
                case HELLO:
                    fn(Hello(type, buf.view(size)));
                    break;
                case DONE:
                    fn(Done(type, buf.view(size)));
                    break;
                case VERTICES:
                    fn(Vertices<Vertex, State>(type, buf.view(size)));
                    break;
                case EDGES:
                    fn(Edges<Edge, Distance>(type, buf.view(size)));
                    break;
                case NUM_SAMPLES:
                    fn(NumSamples(type, buf.view(size)));
                    break;
                case RANDOM_SEED_WORK:
                    fn(RandomSeedWork(type, buf.view(size)));
                    break;
                        // case PROBLEM_SE3:
                        //     fn(ProblemSE3<float>(type, buf.view(size)));
                        //     break;
                        // case PROBLEM_SE3+1:
                        //     fn(ProblemSE3<double>(type, buf.view(size)));
                        //     break;
                        //            case PROBLEM:
                        //                fn(Problem(type, buf.view(size)));
                        //                break;
                        //            case PATH_SE3:
                        //                fn(Path<std::tuple<Eigen::Quaternion<float>, Eigen::Matrix<float, 3, 1>>>(type, buf.view(size)));
                        //                break;
                        //            case PATH_SE3+1:
                        //                fn(Path<std::tuple<Eigen::Quaternion<double>, Eigen::Matrix<double, 3, 1>>>(type, buf.view(size)));
                        //                break;
                        //            case PATH_RVF+8:
                        //                fn(Path<Eigen::Matrix<float, 8, 1>>(type, buf.view(size)));
                        //                break;
                        //            case PATH_RVD+8:
                        //                fn(Path<Eigen::Matrix<double, 8, 1>>(type, buf.view(size)));
                        //                break;
                default:
                        throw protocol_error("bad packet type: " + std::to_string(type));
            }

            buf += size;

            return 0;
        }
}

#endif

